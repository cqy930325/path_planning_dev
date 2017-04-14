/** PerceptIn license **/
#include "vision_controller.h"
/**
 * @brief Constructor for MarkerLocator
 * @param cfg configure setting for MarkerLocator
 */
MarkerLocator::MarkerLocator(std::shared_ptr<PI::config::Config> cfg) {
  detector_ptr = PI::marker::CreateMarkerDetector(cfg);
}

/**
 * @brief Constructor for chessboard locator with stereo camera
 * @param cfg configuraion parameter
 */
ChessStereoLocator::ChessStereoLocator(
  std::shared_ptr<PI::config::Config> cfg) : MarkerLocator(cfg) {
  chessboard_detector_ptr = std::dynamic_pointer_cast
    <PI::marker::ChessboardDetector>(detector_ptr);
}

/**
 * @brief Calculate transform from pattern to bot
 * @param data Stereo camera data
 * @param bot_T_pattern
 * @return True if get the transform , false if can not calculate
 */
bool ChessStereoLocator::GetBotPose(std::shared_ptr<PI::interface::Data> data,
                                    cv::Affine3d *bot_T_pattern) {
  if (!GetCameraPose(data)) {
    return false;
  }
  *bot_T_pattern =  *bot_T_device * *device_T_pattern;
  //Coordinate transform from patter to bot
  return true;
}
/**
 * @brief Load the configuration from calibration file
 * @param file_calib Path for calibration file
 * @return True if load successfully
 */
bool ChessStereoLocator::GetModel(std::string file_calib) {
  std::shared_ptr<PI::model::CalibrationTree> calibTreeptr =
    PI::model::ReadCalibration(file_calib);
  std::set<PI::sensor::Id> sensors;
  sensors.insert(PI::sensor::ID_STEREO_LEFT);
  sensors.insert(PI::sensor::ID_STEREO_RIGHT);  //add left and right camera
  std::map<PI::sensor::Id, std::shared_ptr<const PI::model::Model>> models;
  if (!PI::model::CalibrationTree::GetTransform(
    *calibTreeptr, PI::sensor::ID_BOT, PI::sensor::ID_ROOT, bot_T_device))
    // Get the transform from device to bot
    return false;
  PI::model::CalibrationTree::GetCalibrationForSensors(
    *calibTreeptr, PI::sensor::ID_ROOT, sensors, &models, &sensors_Ts_device);
  if (models.find(PI::sensor::ID_STEREO_LEFT) == models.end() ||
    models.find(PI::sensor::ID_STEREO_RIGHT) == models.end())
    return false;  // return false if can not find left or right camera
  for (auto const &id_model : models) {
    std::shared_ptr<const PI::model::CameraModel> camera_model =
      std::dynamic_pointer_cast<const PI::model::CameraModel>(id_model.second);
    camera_models.insert({id_model.first, camera_model});
    //Put left right camera model into map
  }
  return true;
}

/**
 * @brief Get the transform from pattern to device frame
 * @param data Stereo camera data
 * @return True if get the transform
 */
bool ChessStereoLocator::GetCameraPose(
  std::shared_ptr<PI::interface::Data> data) {
  std::shared_ptr<PI::interface::StereoData> d =
    std::dynamic_pointer_cast<PI::interface::StereoData>(data);
  std::vector<PI::marker::Correspondence3d2d>* left_correspondences =
    new std::vector<PI::marker::Correspondence3d2d>;
  // correspondences for left camera
  std::vector<PI::marker::Correspondence3d2d>* right_correspondences =
    new std::vector<PI::marker::Correspondence3d2d>;
  // correspondences for right camera
  bool leftFound = chessboard_detector_ptr->Detect(
    d->image_left, left_correspondences);
  // Check if detect chessboard from left camera
  bool rightFound = chessboard_detector_ptr->Detect(
    d->image_right, right_correspondences);
  // Check if detect chessboard from right camera
  std::map<PI::sensor::Id, std::vector<PI::marker::Correspondence3d2d>>
    correspondences;
  if (leftFound)
    correspondences[PI::sensor::ID_STEREO_LEFT] = *left_correspondences;
  else if (rightFound)
    correspondences[PI::sensor::ID_STEREO_RIGHT] = *right_correspondences;
  /* TODO(qchen): ceres return segfault for dual camera detection optimization ,
   test on other machine later. Currently use one camera detection */
  if (!leftFound && !rightFound)
    return false;  // return false if neither left or right can detect board
  return PI::marker::SolvePnp(
    correspondences, camera_models, sensors_Ts_device,
    max_rms_value, device_T_pattern, inliers, errors);
}

/**
 * @brief Controller constructor
 * @param p Configuration parameter depend on robot
 */
Controller::Controller(Controller::Parameters p) {
  x_thresh = p.x_thresh;
  y_thresh = p.y_thresh;
  angle_thresh = p.angle_thresh;
  step_angular_v = p.angular_step;
  step_linear_v = p.linear_step;
  state = SEARCH;
  y_target = p.y_target;
  linear_max_v = p.linear_max_v;
  angular_max_v = p.angular_max_v;
}

/**
 * @brief Util function to print state and command
 * @return String contain current controller state and
 * command speed
 */
std::string Controller::Command2Str() {
  std::string ret;
  switch (state) {
    case FAIL:    ret ="FAIL";
      break;
    case FORWARD: ret ="FORWARD";
      break;
    case BACKWARD:ret ="BACKWARD";
      break;
    case SEARCH:    ret = "SEARCH";
      break;
    case ROTATE_HEAD: ret = "ROTATE_HEAD";
      break;
    case ROTATE: ret = "ROTATE";
      break;
    case SUCCESS: ret = "SUCCESS";
      break;
  }
  ret += std::to_string(cmd.speed);  // append speed into string
  return ret;
}

/**
 * @brief Get command when lost of track
 * @return Command send to robot (move/turn)
 */
PI::planning::Command Controller::GetCommand() {
  if (state == SEARCH) {
    lost_count++;  // try to find the marker by rotating robot
    if (lost_count < (M_PI / step_linear_v)) {
      cmd = PI::planning::Command(PI::planning::command::TURN
        , 2*step_angular_v);  //rotate robot at most 720 degrees to find markers
    }
    else {
      state = FAIL;
      cmd = PI::planning::Command(PI::planning::command::MOVE, 0);
    }
  } else if (state == ROTATE) {
    lost_count++;
    if (lost_count == 1) {
      // record the rotation direction when lost of track
      last_angular_v = cmd.speed;
    } else if (lost_count < 5) {
      // wait for more detection attempt
      cmd = PI::planning::Command(PI::planning::command::MOVE, 0);
    } else if (lost_count < 10) {
      // detection attempt fail, rotate the robot reversely
      cmd = PI::planning::Command(PI::planning::command::TURN,
                                  -last_angular_v);
      recovered = false;
    } else {
      // exceed max attempt return fail
      state = FAIL;
    }
  } else if (state == FORWARD) {
    lost_count++;
    if (lost_count < 5) {
      // wait for more detection attempt
      cmd = PI::planning::Command(PI::planning::command::MOVE, 0);
    } else if (lost_count < 10) {
      // go backward for detection
      cmd = PI::planning::Command(PI::planning::command::MOVE,
                                  -2*step_linear_v);
      recovered = false;
    } else {
      state = SEARCH;
      cmd = PI::planning::Command(PI::planning::command::TURN, step_angular_v);
    }
  } else if (state == BACKWARD) {
    if (lost_count < 5) {
      cmd = PI::planning::Command(PI::planning::command::MOVE, 0);
    } else {
      state = SEARCH;
      // / in BACKWARD state, when lost of track, it is no use reversing
      cmd = PI::planning::Command(PI::planning::command::MOVE, step_linear_v);
      recovered = false;
    }
  } else {
    state = SEARCH;
  }

  LOG(ERROR) << "Lost of track:" << lost_count << std::endl;
  return cmd;
}
/**
 * @brief Clip the value if its absolute value is great than a threshold
 * @param val Value want to clip
 * @param threshold Threshold value
 * @return Clipped value
 */
double Clip(double val, double threshold) {
  CHECK(threshold > 0) << "threshold should always greater than 0!";
  return fabs(val) < threshold ? threshold * val / fabs(val) : val;
}
/**
 * @brief Generate robot command given its current transform
 * @param bot_T_pattern Transform from marker to robot
 * @return Robot command to move or turn
 */
PI::planning::Command Controller::GetCommand(cv::Affine3d *bot_T_pattern) {
  lost_count = 0;
  cv::Affine3d pattern_T_bot(bot_T_pattern->inv());
  cv::Vec3d tvec = bot_T_pattern->translation();
  auto rot = pattern_T_bot.rotation() * cv::Vec3d(1, 0, 0);
  // angle between bot heading and tag in 3d in tag frame
  double x = pattern_T_bot.translation()[0];
  double y = -pattern_T_bot.translation()[2];
  // clip the rotate angle no more than 30 degrees
  double bot_rot = -atan2(rot[0], rot[2]) / M_PI * 180;
  // angle between bot heading and tag project in 2d
  // ROTATE: rotate bot due to x offset
  double angle_diff = target_rot - bot_rot;
  // difference between current heading and desired angle to reach 0 x offset
  LOG(INFO) << "x:" << x << "y:" << y << std::endl;
  LOG(INFO) << "target rot:" << target_rot << std::endl;
  LOG(INFO) << "bot_rot:" << bot_rot << std::endl;
  // state machine
  if (fabs(y - y_target) < y_thresh && fabs(x) < x_thresh
    && fabs(bot_rot) < angle_thresh) {
    // goal reached criteria: x,y, angle in threshold
    state = SUCCESS;
  } else if (state == SEARCH) {
    // SEARCH: loss track of tag
    search_count++;
    target_rot = atan2(x, y - y_target) / M_PI * 180;
    // desired rotate angle to reach desired x
    target_rot = fabs(target_rot) > 30
                 ? 30 * target_rot / fabs(target_rot) : target_rot;
    if (search_count < 3)
      state = SEARCH;
    else
      state = ROTATE;
  } else if (state == ROTATE) {
    search_count = 0;
    if (fabs(x) > x_thresh &&
      fabs(angle_diff) > angle_thresh && y-y_target > 2*y_thresh)
      state = ROTATE;
    else if (fabs(x) > x_thresh && y-y_target < 2*y_thresh)
      state = ROTATE_HEAD;
    else
      state = y-y_target > 2*y_thresh ? FORWARD : BACKWARD;
  } else if (state == ROTATE_HEAD) {
    if (fabs(bot_rot) > angle_thresh)
      state = ROTATE_HEAD;
    else if (fabs(x) > x_thresh)
      state = BACKWARD;
    else
      state = FORWARD;
  } else if (state == FORWARD) {
    if (fabs(x) > x_thresh && angle_diff > angle_thresh)
      state = ROTATE;  // x-diff to to rotate for move left/right
    else if (fabs(x) > x_thresh && angle_diff < angle_thresh && !recovered)
      state = FORWARD; // move forward to reduce x offset
    else if (fabs(bot_rot) > angle_thresh)
      state = ROTATE_HEAD;
    else if (y - y_target < y_thresh)
      state = BACKWARD;  // exceed go backward
  } else if (state == BACKWARD) {
    if (y - y_target < 2*y_thresh && fabs(x) > x_thresh || y - y_target < 0)
      state = BACKWARD;
    else if (fabs(x) > x_thresh)
      state = ROTATE;
    else if (y - y_target > y_thresh)
      state = FORWARD;
  }
  recovered = true;
  // command based on state machine
  switch (state) {
    case ROTATE: cmd = PI::planning::Command(
        PI::planning::command::TURN,
        Clip(angle_diff*0.005, step_angular_v));
      break;
    case ROTATE_HEAD: cmd = PI::planning::Command(
        PI::planning::command::TURN,
        Clip(-bot_rot*0.005, step_angular_v));
      break;
    case FORWARD: cmd = PI::planning::Command(
        PI::planning::command::MOVE,
        Clip(fabs(y - y_target) * 0.5, step_linear_v));
      break;
    case BACKWARD: cmd = PI::planning::Command(
        PI::planning::command::MOVE,
        Clip(fabs(y - y_target) * -0.5, step_linear_v*2));
      break;
    default: cmd = PI::planning::Command(PI::planning::command::MOVE, 0);
  }
  return cmd;
}

/**
 * Init the serial connect to the robot
 * @return True if connection established
 */
bool Controller::InitBot() {
  pBot = new PIRoBot::HBot();
  pHbot = dynamic_cast<PIRoBot::HBot*>(pBot);
  bool ret = pBot->InitBot("/dev/ttyUSB0");
  if (!ret) {
    std::cerr << "can not open usb, check connection" << std::endl;
    return false;
  }
  pBot->StartBotThread();
  return true;
}

/**
 * @brief Send the command to robot
 */
void Controller::CmdUpdate() {
  double linear_v = 0, angular_v = 0;
  if (cmd.type == PI::planning::command::MOVE)
    linear_v = fabs(cmd.speed) > linear_max_v
               ? linear_max_v * cmd.speed / fabs(cmd.speed) : cmd.speed;
  else if (cmd.type == PI::planning::command::TURN)
    angular_v = fabs(cmd.speed) > angular_max_v
                ? angular_max_v * cmd.speed / fabs(cmd.speed) : cmd.speed;
  pBot->SetLinearAndAugularVelocity(linear_v, angular_v);
  BotStateUpdate();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  pBot->SetLinearAndAugularVelocity(0, 0);
  BotStateUpdate();
}

/**
 *  @brief Robot odometry update copy from clean_robot repo
 */
void Controller::BotStateUpdate() {
  pBot->mutexStateUpdate.lock();
  if (pBot->isNewState) {
    pBot->isNewState = false;
    pHbot = dynamic_cast<PIRoBot::HBot *>(pBot);
    CHECK(pHbot);
    size_t timestamp;
    timestamp = pHbot->timestamp;
    if(notFirst) {
      bot_file<<timestamp<<" "<<pHbot->pos.x<<" "<<pHbot->pos.y<<
              " "<<pHbot->pos.th<<" "<<pHbot->leftEncoderPos<<" "
              <<pHbot->rightEncoderPos<<" "<<pHbot->gyrZ<<" "
              <<pHbot->headingFromImu<<std::endl;
      double lastTime, curTime;
      lastTime = lastTimestamp;
      curTime = timestamp;
      double deltaTime = (curTime - lastTime) / 1000.0;
      if(deltaTime < 0.010 || deltaTime > 0.030) {
        std::cout<<"wrong dt:"<<deltaTime<<" "<<curTime<<" "<<lastTime<<std::endl;
      }
      leftVel = (pHbot->leftEncoderPos - lastLeftEncoder) * mechParam / deltaTime;
      rightVel = (pHbot->rightEncoderPos - lastRightEncoder) * mechParam / deltaTime;
      bot_vel_file<<timestamp<<" "<<pHbot->pos.x<<" "<<pHbot->pos.y
                  <<" "<<pHbot->pos.th<<" "<<leftVel<<" "<<rightVel
                  <<" "<<pHbot->gyrZ<<" "<<pHbot->headingFromImu<<std::endl;
    }
    else {
      notFirst = true;
    }
    lastTimestamp = timestamp;
    lastLeftEncoder = pHbot->leftEncoderPos;
    lastRightEncoder = pHbot->rightEncoderPos;
  }
  pBot->mutexStateUpdate.unlock();
}
 /**
  * @brief stop the robot and close the connection to robot
  */
void Controller::Close() {
  pBot->SetLinearAndAugularVelocity(0, 0);
  BotStateUpdate();
  pBot->mutexExitThread.lock();
  pBot->isExitThread = true;
  pBot->mutexExitThread.unlock();
  bot_file.close();
  pBot->EndBotThread();
  pBot->Finish();
}





