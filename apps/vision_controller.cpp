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
#include <planning/command.h>

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
    virtual bool getBotPose(std::shared_ptr<PI::interface::Data> data,cv::Affine3d *bot_T_pattern) = 0;
    cv::Affine3d* getbot_T_device(){return bot_T_device;}
protected:
    std::shared_ptr<PI::marker::MarkerDetector> detector_ptr;
    double max_rms_value = 5.0; //TODO:need add to config file
    cv::Affine3d *device_T_pattern = new cv::Affine3d;
    std::map<PI::sensor::Id, cv::Affine3d> sensors_Ts_device;
    cv::Affine3d *bot_T_device = new cv::Affine3d;



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
    bool getBotPose(std::shared_ptr<PI::interface::Data> data,cv::Affine3d *pattern_T_bot) override;
    cv::Affine3d getDevice_T_pattern(){return *device_T_pattern;}
private:
    std::shared_ptr<PI::marker::ChessboardDetector> cbDetectorptr;
    std::map<PI::sensor::Id, std::shared_ptr<const PI::model::CameraModel>> camera_models;
    std::map<PI::sensor::Id, std::vector<size_t>> *inliers = nullptr;
    PI::util::StatisticWatcher<double> *errors = nullptr;

};





ChessStereoLocator::ChessStereoLocator(std::shared_ptr<PI::config::Config> cfg) : MarkerLocator(cfg){
    cbDetectorptr = std::dynamic_pointer_cast<PI::marker::ChessboardDetector>(detector_ptr);
}

bool ChessStereoLocator::getBotPose(std::shared_ptr<PI::interface::Data> data,cv::Affine3d *pattern_T_bot){
   if (!getCameraPose(data)){
       return false;
   }
    *pattern_T_bot = *bot_T_device * *device_T_pattern ;
   return true;
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

    cv::Matx<double,4,1> pose3d= T.matrix * cv::Matx<double,4,1>(0,0,0,1);
    std::cout << "pose = " << std::endl << " " << pose3d << std::endl;

}
/**                                                             y
 *  coordinate system for bot                                  /|\
 *  x: perpendicular to bot move forward direction              |                             |
 *  y: direction for the bot move forward                       |_______\ x                                                      /
 *  z: perpendicular to x-y plane (point down to the ground )
 */
class Controller{
public:
    struct Parameters{
        double x_thresh,y_thresh,angle_thresh;
        double steer_step, move_step;
    };
    enum State{IDLE,LEFT,RIGHT,FORWARD,BACKWARD,HALT};
    Controller(Parameters p);
    PI::planning::Command getCommand(cv::Affine3d* pose);
    PI::planning::Command getCommand();
    std::string command2Str();
private:
    PI::planning::Command cmd;
    PI::planning::Command prev_cmd;
    State state,prev_state;
    double x_thresh,y_thresh,angle_thresh;
    double steer_step, move_step;
    double lost_count;
};
Controller::Controller(Controller::Parameters p){
    x_thresh = p.x_thresh;
    y_thresh = p.y_thresh;
    angle_thresh = p.angle_thresh;
    steer_step = p.steer_step;
    move_step = p.move_step;
    state = IDLE;
    prev_state = IDLE;
}
std::string Controller::command2Str(){
    std::string ret;
    switch(state){
        case LEFT:    ret ="LEFT";
            break;
        case RIGHT:   ret ="RIGHT";
            break;
        case HALT:    ret ="HALT";
            break;
        case FORWARD: ret ="FORWARD";
            break;
        case BACKWARD:ret ="BACKWARD";
            break;
        case IDLE:    ret = "IDLE";
            break;
    }
    return ret;
}

PI::planning::Command Controller::getCommand(){
    if (state == IDLE)
        return PI::planning::Command(PI::planning::command::MOVE,0);
    if (state == RIGHT || state == LEFT){
        lost_count++;
        if (lost_count < 5){
            return PI::planning::Command(PI::planning::command::MOVE,0);
        }
        else{
            state = state == RIGHT ? LEFT:RIGHT;
            return PI::planning::Command(PI::planning::command::TURN,state == RIGHT ? steer_step:-steer_step);
        }
    }
    return PI::planning::Command(PI::planning::command::MOVE,0);
}
PI::planning::Command Controller::getCommand(cv::Affine3d* pose){
    cmd = PI::planning::Command(PI::planning::command::MOVE,0);
    lost_count = 0;
    cv::Vec3d tvec = pose->translation();
    cv::Vec3d rvec =  pose->rvec();
    double x = tvec[0];
    double y = tvec[1];
    double angle_z = rvec[2];
    if(fabs(y) < y_thresh && fabs(angle_z) > angle_thresh && fabs(x) < x_thresh)
        return cmd;
    else if(fabs(x) > x_thresh){
        if (x < 0){
            cmd = PI::planning::Command(PI::planning::command::TURN,steer_step);
            state = LEFT;
        }
        else{
            cmd = PI::planning::Command(PI::planning::command::TURN,-steer_step);
            state = RIGHT;
        }
    }
    else{
        cmd =  PI::planning::Command(PI::planning::command::MOVE,move_step);
        state = FORWARD;
    }
    return PI::planning::Command(PI::planning::command::MOVE,0);
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
    Controller::Parameters p;
    p.angle_thresh = 0.3;
    p.move_step = 1;
    p.steer_step = 0.1;
    p.x_thresh = 0.1;
    p.y_thresh = 1;
    Controller c(p);
    if (!csl.getModels(file_calib)){
        std::cout << "can not find camera model in JSON file!" << std::endl;
        return -1;
    }
    std::cout << "transform from bot to device" << std::endl << csl.getbot_T_device()->matrix << std::endl;
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
            cv::Affine3d *botPose = new cv::Affine3d;
            if (csl.getBotPose(stereo_data,botPose)){
                std::cout << "cam:" << std::endl;
                printAffine3D(csl.getDevice_T_pattern());
                std::cout << "bot:" << std::endl;
                printAffine3D(*botPose);
                c.getCommand(botPose);
            }else{
                c.getCommand();
            }

            std::cout << "state:" <<c.command2Str() << std::endl;
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


