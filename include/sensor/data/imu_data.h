/** PerceptIn license **/

#ifndef SENSOR_INCLUDE_SENSOR_DATA_IMU_DATA_H_
#define SENSOR_INCLUDE_SENSOR_DATA_IMU_DATA_H_

#include <Eigen/Core>

#include <sensor/data/data.h>

namespace PI {
namespace sensor {

/**
 * ImuData stores data captured from an IMU (gyroscope + accelerometer)
 */
class ImuData : public Data {
 public:
  ImuData(const Eigen::Vector3d &accel,
          const Eigen::Vector3d &gyro,
          const size_t timestamp_ms_accel = 0,
          const size_t timestamp_ms_gyro = 0);

  ImuData(const ImuData &other);
  Type type() const;
  std::shared_ptr<Data> clone() const;

  /** Getters **/
  const Eigen::Vector3d &accel() const;
  const Eigen::Vector3d &gyro() const;
  const size_t timestamp_ms_accel() const;
  const size_t timestamp_ms_gyro() const;
  // Timestamp of the accelerometer in second.
  const double timestamp_sec_accel() const;
  // Timestamp of the gyroscope in second.
  const double timestamp_sec_gyro() const;

  /** Setters **/
  Eigen::Vector3d &accel();
  Eigen::Vector3d &gyro();

 private:
  // Convert timestamp from ms to sec.
  inline double ms_to_sec(const size_t ms) const {
    return static_cast<double>(ms) * 1e-3;
  }
  /// Readings from the accelerometer.
  Eigen::Vector3d accel_;
  /// Readings from the gyroscope.
  Eigen::Vector3d gyro_;
  /// Timestamp (ms) from the accelerometer.
  const size_t timestamp_ms_accel_;
  /// Timestamp (ms) from the gyroscope.
  const size_t timestamp_ms_gyro_;
};

}  // namespace sensor
}  // namespace PI

#endif  // SENSOR_INCLUDE_SENSOR_DATA_IMU_DATA_H_
