/** PerceptIn license **/
#include <opencv2/highgui/highgui.hpp>
#include <interface/perceptin_v1_interface.h>
#include <model/calib_io.h>
#include <model/calibration_tree.h>
#include <model/camera_model.h>
#include <marker/chessboard_detector.h>
#include <marker/solve_pnp.h>
#include <marker/correspondence3d2d.h>
#include <marker/marker.h>
#include <marker/marker_detector.h>
#include <config/config.h>
#include <config/config_io.h>
#include <sensor/sensors.h>
#include <util/statistics.h>
#include <iostream>
const std::string kWindowName1("PerceptIn V1 Stereo Left");
const std::string kWindowName2("PerceptIn V1 Stereo Right");

void ExposureTrackerBarCallback(int value, void *interface_ptr) {
    static_cast<PI::interface::PerceptInV1Interface*>(interface_ptr)->SetExposure(
                uint32_t(value));
}
class MarkerLocator{
public:
    explicit MarkerLocator(std::shared_ptr<PI::config::Config> cfg);
    virtual ~MarkerLocator() {}
    virtual bool getModels(std::string file_calib) = 0;
    virtual bool getCameraPose(std::shared_ptr<PI::interface::Data> data) = 0;
protected:
    std::shared_ptr<PI::marker::MarkerDetector> detector_ptr;
    double max_rms_value = 5.0;
    cv::Affine3d *device_T_pattern = new cv::Affine3d;
    std::map<PI::sensor::Id, cv::Affine3d> sensors_Ts_device;
};
MarkerLocator::MarkerLocator(std::shared_ptr<PI::config::Config> cfg){
        detector_ptr = PI::marker::CreateMarkerDetector(cfg);
}
class ChessStereoLocator : public MarkerLocator{
public:
    explicit ChessStereoLocator(std::shared_ptr<PI::config::Config> cfg);
    ~ChessStereoLocator(){}
    bool getModels(std::string file_calib) override;
    bool getCameraPose(std::shared_ptr<PI::interface::Data> data) override;
    cv::Affine3d getDevice_T_pattern(){return *device_T_pattern;}
    cv::Affine3d *bot_T_device = new cv::Affine3d;
private:
    std::shared_ptr<PI::marker::ChessboardDetector> cbDetectorptr;
    std::map<PI::sensor::Id, std::shared_ptr<const PI::model::CameraModel>> camera_models;
    std::map<PI::sensor::Id, std::vector<size_t>> *inliers = nullptr;
    PI::util::StatisticWatcher<double> *errors = nullptr;

};
ChessStereoLocator::ChessStereoLocator(std::shared_ptr<PI::config::Config> cfg) : MarkerLocator(cfg){
    cbDetectorptr = std::dynamic_pointer_cast<PI::marker::ChessboardDetector>(detector_ptr);
}


bool ChessStereoLocator::getModels(const std::string file_calib){
    std::shared_ptr<PI::model::CalibrationTree> calibTreeptr =  PI::model::ReadCalibration(file_calib);
    std::set<PI::sensor::Id> sensors;
    sensors.insert(PI::sensor::ID_STEREO_LEFT);
    sensors.insert(PI::sensor::ID_STEREO_RIGHT);
    std::map<PI::sensor::Id, std::shared_ptr<const PI::model::Model>> models;
    if (!PI::model::CalibrationTree::GetTransform(*calibTreeptr,PI::sensor::ID_ROOT,PI::sensor::ID_BOT,bot_T_device))
        return false;
    PI::model::CalibrationTree::GetCalibrationForSensors(*calibTreeptr,PI::sensor::ID_ROOT,sensors,&models,&sensors_Ts_device);
    if (models.find(PI::sensor::ID_STEREO_LEFT) == models.end() ||
            models.find(PI::sensor::ID_STEREO_RIGHT) == models.end())
        return false;
    for (auto const &id_model : models){
        std::shared_ptr<const PI::model::CameraModel> camera_model =
            std::dynamic_pointer_cast<const PI::model::CameraModel>(id_model.second);
        camera_models.insert({id_model.first,camera_model});
    }
    return true;
}

bool ChessStereoLocator::getCameraPose(const std::shared_ptr<PI::interface::Data> data){
    std::shared_ptr<PI::interface::StereoData> d = std::dynamic_pointer_cast<PI::interface::StereoData>(data);
    std::vector<PI::marker::Correspondence3d2d>* leftcorrespondences = new std::vector<PI::marker::Correspondence3d2d>;
    std::vector<PI::marker::Correspondence3d2d>* rightcorrespondences = new std::vector<PI::marker::Correspondence3d2d>;
    bool leftFound = cbDetectorptr->Detect(d->image_left, leftcorrespondences);
    bool rightFound = cbDetectorptr->Detect(d->image_right, rightcorrespondences);
    std::map<PI::sensor::Id,std::vector<PI::marker::Correspondence3d2d>> correspondences;
    if(leftFound)
        correspondences[PI::sensor::ID_STEREO_LEFT] = *leftcorrespondences;
    else if(rightFound)
        correspondences[PI::sensor::ID_STEREO_RIGHT] = *rightcorrespondences;
    if (!leftFound && !rightFound){
        return false;
    }
    return PI::marker::SolvePnp(correspondences,camera_models,sensors_Ts_device,max_rms_value,device_T_pattern,inliers,errors);
}




void printAffine3D(cv::Affine3d T){
    cv::Vec3d rvec = T.rvec();
    cv::Vec3d tvec = T.translation();
    std::cout << "tvec = " << std::endl << " " << tvec << std::endl;
    std::cout << "rvec = " << std::endl << " " << rvec << std::endl;
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        printf("Not enough input argument.\nUsage:\n%s [calib JSON] [config JSON] \n", argv[0]);
        return -1;
    }
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::ERROR);
    google::SetLogDestination(google::INFO, std::string("/tmp/" + std::string(argv[0]) + "_").c_str());
    const std::string file_calib(argv[1]);
    const std::string file_cfg(argv[2]);
    PI::interface::PerceptInV1Interface interface(true, false);
    //TODO: add bot frame!!
    std::shared_ptr<PI::config::Config> cfg = PI::config::ReadConfig(file_cfg,"marker");
    ChessStereoLocator csl(cfg);
    if (!csl.getModels(file_calib)){
        std::cout << "can not find camera model in JSON file!" << std::endl;
        return -1;
    }
    printAffine3D(*csl.bot_T_device);
    // Start streaming.
    if (!interface.StartStreaming()) {
        return -1;
    }
    interface.SetExposure(250);
    int exposure_value;

    // Create two windows with tracker bar to control exposure.
    cv::namedWindow(kWindowName1);
    cv::namedWindow(kWindowName2);
    {
        uint32_t exposure_value_32;
        if (interface.GetExposure(&exposure_value_32)) {
            exposure_value = static_cast<int>(exposure_value_32);
            cv::createTrackbar("Exposure", kWindowName1, &exposure_value, 800,
                               ExposureTrackerBarCallback, &interface);
        }
    }
    while (1) {
        std::shared_ptr<PI::interface::Data> data = interface.GetNewestData();
        if (!data) {
            continue;
        }

        // See if this an IMU data.
        std::shared_ptr<PI::interface::ImuData> imu_data =
                std::dynamic_pointer_cast<PI::interface::ImuData>(data);
        if (imu_data) {
            continue;
        }

        // This must be an stereo data.
        std::shared_ptr<PI::interface::StereoData> stereo_data =
                std::dynamic_pointer_cast<PI::interface::StereoData>(data);
        if (stereo_data) {
            cv::imshow(kWindowName1, stereo_data->image_left);
            cv::imshow(kWindowName2, stereo_data->image_right);
            int key = cv::waitKey(5);
            if (key == 27) {
                break;
            }
            if (csl.getCameraPose(stereo_data)){
                printAffine3D(csl.getDevice_T_pattern());
            }
            continue;
        }
        // This is not an IMU data nor a stereo data. Who are you?
        printf("Something weird has happened.\n");
        break;
    }

    if (!interface.StopStreaming()) {
        return -1;
    }

    return 0;
}


