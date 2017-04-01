/** PerceptIn license **/

#ifndef SENSOR_INCLUDE_SENSOR_DATA_BOT_IMU_DATA_H_
#define SENSOR_INCLUDE_SENSOR_DATA_BOT_IMU_DATA_H_

#include <Eigen/Core>

#include <sensor/data/data.h>

namespace PI {
namespace sensor {

/**
 * BotData stores data captured from an robot.
 */
// TODO(gtsai): Refactor BotData. Define WheelData for the left and right wheel
// encoder, and BotData is a derived class of WheelData and Pose2dData.
class BotData : public Data {
 public:
  BotData(double vel_left_wheel,
          double vel_right_wheel,
          const Eigen::Vector3d & pos_2d_heading,
          double gyr_yaw,
          double gyr_heading);

  BotData(const BotData &other);
  Type type() const;
  std::shared_ptr<Data> clone() const;

  /** Getters **/
  const double vel_left_wheel() const;
  const double vel_right_wheel() const;
  const Eigen::Vector3d pos_2d_heading() const;
  const double gyr_yaw() const;
  const double gyr_heading() const;

 private:
  // different bot may have different data, hard to process.
  const double vel_left_wheel_;
  const double vel_right_wheel_;
  const Eigen::Vector3d pos_2d_heading_;
  const double gyr_yaw_;
  const double gyr_heading_; // heading from gyr, we may don't use this data.
};

}  // namespace sensor
}  // namespace PI

#endif  // SENSOR_INCLUDE_SENSOR_DATA_BOT_IMU_DATA_H_
