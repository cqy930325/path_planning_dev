/** PerceptIn license **/

#ifndef DATA_H_
#define DATA_H_

#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace PI {
namespace interface {

typedef unsigned long Timestamp;

/**
 * Base class of a Data coming out from the interface.
 */
struct Data {
  /// Timestamp when the data is captured.
  Timestamp timestamp;

  Data();
  explicit Data(const Timestamp timestamp);
  virtual ~Data();

  /**
   * @brief Save a data to disk.
   * @param dir Directory to save the data. Directory must exist.
   * @param data Pointer to the data to save.
   * @return True if the data is saved to disk.
   */
  static bool Save(const std::string &dir,
                   std::shared_ptr<const Data> data);
};

/**
 * IMU readings.
 */
struct ImuData : public Data {
  cv::Vec3d accel;  /// Accelerometer reading.
  cv::Vec3d gyro;   /// Gyroscope reading.

  ImuData();
  ImuData(const Timestamp timestamp,
          const double accel_x, const double accel_y, const double accel_z,
          const double gyro_x, const double gyro_y, const double gyro_z);
  ~ImuData();

  /**
   * @brief Save a IMU data to disk by appending the IMU readings to the .csv
   *        file.
   * @param dir Directory to save the data. Directory must exist.
   * @param imu_data Pointer to the IMU data to save.
   * @return True if the data is saved to disk.
   */
  static bool Save(const std::string &dir,
                   std::shared_ptr<const ImuData> imu_data);
};

/**
 * Stereo image.
 */
struct StereoData : public Data {
  /// Stereo image, left and right
  cv::Mat_<uchar> image_left;
  cv::Mat_<uchar> image_right;

  StereoData();
  StereoData(const Timestamp timestamp,
             const cv::Mat_<uchar> &image_left,
             const cv::Mat_<uchar> &image_right);
  ~StereoData();

  /**
   * @brief Save a stereo data to disk as two .png images.
   * @param dir Directory to save the data. Directory must exist.
   * @param stereo_data Pointer to the stereo data to save.
   * @return True if the data is saved to disk.
   */
  static bool Save(const std::string &dir,
                   std::shared_ptr<const StereoData> stereo_data);
};

}  // namespace interface
}  // namespace PI

#endif  // DATA_H_
