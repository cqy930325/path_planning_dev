//
// Created by root on 4/11/17.
//

#include "../src/vision_controller.h"
const std::string kWindowName1("PerceptIn V1 Stereo Left");
const std::string kWindowName2("PerceptIn V1 Stereo Right");

void ExposureTrackerBarCallback(int value, void *interface_ptr) {
  static_cast<PI::interface::PerceptInV1Interface*>
  (interface_ptr)->SetExposure(
    uint32_t(value));
}
int main(int argc, char *argv[]) {
  if (argc < 3) {
    printf("Not enough input argument."
             "\nUsage:\n%s [calib JSON] [config JSON] \n", argv[0]);
    return -1;
  }
  /*google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::ERROR);
  google::SetLogDestination(google::INFO,
                            std::string("/tmp/" + std::string(argv[0]) + "_").c_str());*/
  const std::string file_calib(argv[1]);
  const std::string file_cfg(argv[2]);
  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 3;
  std::string status;
  cv::Point textOrg((640 - 400)/2,
                    (480 + 10)/2);
  PI::interface::PerceptInV1Interface interface(true, false);
  //TODO: add bot frame!!
  std::shared_ptr<PI::config::Config> cfg
    = PI::config::ReadConfig(file_cfg,"marker");
  ChessStereoLocator csl(cfg);
  Controller::Parameters p;
  p.angle_thresh = 3;
  p.linear_step = 0.05;
  p.angular_step = 0.03;
  p.x_thresh = 0.03;
  p.y_thresh = 0.03;
  p.y_target = 0.4;
  p.linear_max_v = 0.1;
  p.angular_max_v = 0.1;
  Controller c(p);
  if (!c.InitBot()) {
    std::cerr << "can not communicate with robot , exit!";
    return -1;
  }
  if (!csl.GetModel(file_calib)){
    std::cerr << "can not find camera model in JSON file!" << std::endl;
    return -1;
  }
  LOG(INFO)<< "transform from device to bot"
           << std::endl << csl.GetBot_T_Device()->matrix << std::endl;
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
      cv::Mat left_res = stereo_data->image_left.clone();
      cv::putText(left_res,status,textOrg,fontFace,fontScale,cv::Scalar::all(0));
      cv::imshow(kWindowName1, left_res);
      cv::imshow(kWindowName2, stereo_data->image_right);
      int key = cv::waitKey(5);
      if (key == 27) {
        break;
      }
      cv::Affine3d *botPose = new cv::Affine3d;
      // Get robot pose and use pose to generate robot command
      if (csl.GetBotPose(stereo_data, botPose)) {
        c.GetCommand(botPose);
      }else {
        c.GetCommand();
      }
      status = c.Command2Str();
      LOG(INFO) << "state:" <<status << std::endl;
      c.CmdUpdate(); //Send out the cmd
      continue;
    }
    // This is not an IMU data nor a stereo data. Who are you?
    printf("Something weird has happened.\n");
    break;
  }

  if (!interface.StopStreaming()) {
    return -1;
  }
  c.Close();
  return 0;
}



