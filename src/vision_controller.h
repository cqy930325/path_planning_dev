//
// Created by root on 4/11/17.
//

#ifndef PATH_PLANNING_DEV_VISION_CONTROLLER_H
#define PATH_PLANNING_DEV_VISION_CONTROLLER_H
#include <opencv2/highgui/highgui.hpp>
#include <interface/perceptin_v1_interface.h>
#include <model/calib_io.h>
#include <model/calibration_tree.h>
#include <model/camera_model.h>
#include <marker/chessboard_detector.h>
#include <marker/solve_pnp.h>
#include <marker/marker.h>
#include <config/config.h>
#include <config/config_io.h>
#include <util/statistics.h>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <set>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>
#include <planning/command.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <h_bot/h_bot.h>
#endif //PATH_PLANNING_DEV_VISION_CONTROLLER_H


class MarkerLocator{
 public:
  /**
 * @brief Constructor for MarkerLocator
 * @param cfg configure setting for MarkerLocator
 */
  explicit MarkerLocator(std::shared_ptr<PI::config::Config> cfg);
  virtual ~MarkerLocator() {}
  /**
 * @brief Load the configuration from calibration file
 * @param file_calib Path for calibration file
 * @return True if load successfully
 */
  virtual bool GetModel(std::string file_calib) = 0;
  /**
 * @brief Get the transform from pattern to device frame
 * @param data Stereo camera data
 * @return True if get the transform
 */
  virtual bool GetCameraPose(std::shared_ptr<PI::interface::Data> data) = 0;
  /**
 * @brief Calculate transform from pattern to bot
 * @param data Stereo camera data
 * @param bot_T_pattern
 * @return True if get the transform , false if can not calculate
 */
  virtual bool GetBotPose(std::shared_ptr<PI::interface::Data> data,
                          cv::Affine3d *bot_T_pattern) = 0;
  cv::Affine3d* GetBot_T_Device() {return bot_T_device;}
 protected:
  std::shared_ptr<PI::marker::MarkerDetector> detector_ptr;
  double max_rms_value = 5.0; //TODO(qchen):need add to config file
  cv::Affine3d *device_T_pattern = new cv::Affine3d;
  std::map<PI::sensor::Id, cv::Affine3d> sensors_Ts_device;
  cv::Affine3d *bot_T_device = new cv::Affine3d;
};

class ChessStereoLocator : public MarkerLocator{
 public:
  explicit ChessStereoLocator(std::shared_ptr<PI::config::Config> cfg);
  ~ChessStereoLocator() {}
  bool GetModel(std::string file_calib) override;
  bool GetCameraPose(std::shared_ptr<PI::interface::Data> data) override;
  bool GetBotPose(std::shared_ptr<PI::interface::Data> data,
                  cv::Affine3d *bot_T_pattern) override;
 private:
  std::shared_ptr<PI::marker::ChessboardDetector> chessboard_detector_ptr;
  std::map<PI::sensor::Id, std::shared_ptr<const PI::model::CameraModel>>
    camera_models;
  std::map<PI::sensor::Id, std::vector<size_t>> *inliers = nullptr;
  PI::util::StatisticWatcher<double> *errors = nullptr;

};

class Controller{
 public:
  struct Parameters{
    double x_thresh, y_thresh, angle_thresh; //rotation and translation tolerance
    double angular_step, linear_step; // minimum command speed
    double y_target; // target y direction
    double angular_max_v, linear_max_v;  // maximum command speed
  };
  enum State{SEARCH, FORWARD, BACKWARD,
    FAIL, SUCCESS, ROTATE, ROTATE_HEAD};  //controller state
  /**
 * @brief Controller constructor
 * @param p Configuration parameter depend on robot
 */
  explicit Controller(Parameters p);
 /**
  * @brief Generate robot command given its current transform
  * @param bot_T_pattern Transform from marker to robot
  * @return Robot command to move or turn
  */
  PI::planning::Command GetCommand(cv::Affine3d *bot_T_pattern);
  /**
 * @brief Get command when lost of track
 * @return Command send to robot (move/turn)
 */
  PI::planning::Command GetCommand();
  /**
 * @brief stop the robot and close the connection to robot
 */
  void Close();
  /**
 * @brief Send the command to robot
 */
  void CmdUpdate();
  /**
 * Init the serial connect to the robot
 * @return True if connection established
 */
  bool InitBot();
  /**
 * @brief Util function to print state and command
 * @return String contain current controller state and
 * command speed
 */
  std::string Command2Str();

 private:
  /**
 *  @brief Robot odometry update copy from clean_robot repo
 */
  void BotStateUpdate();
  PIRoBot::Bot * pBot;
  PIRoBot::HBot * pHbot;
  std::ofstream bot_file, bot_vel_file;
  PI::planning::Command cmd;
  State state;
  bool recovered = false;
  double x_thresh, y_thresh, angle_thresh;
  double step_angular_v, step_linear_v;
  double y_target, target_rot = 0;
  double lost_count;
  double leftVel, rightVel;
  double linear_max_v , angular_max_v;
  double last_angular_v = 0;
  double lastLeftEncoder = 0, lastRightEncoder = 0;
  double search_count = 0;
  size_t lastTimestamp;
  double mechParam = 2*3.14*0.0325/(48*46.852);
  bool notFirst = false;
};


