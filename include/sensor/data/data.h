/** PerceptIn license **/

#ifndef SENSOR_INCLUDE_SENSOR_DATA_DATA_H_
#define SENSOR_INCLUDE_SENSOR_DATA_DATA_H_

#include <memory>

namespace PI {
namespace sensor {

/**
 * API for data passing into and out from the device (or logger).
 */
class Data {
 public:
  /**
   * Types for the completed Data classes.
   */
  enum Type {
    DEPTH_DATA,  // DepthData
    IMAGE_DATA,  // ImageData
    IMU_DATA,     // IMUData
    BOT_DATA, // BotData
    POSE_2D_DATA,  // Pose2dData
  };

  Data();
  Data(const Data &other);
  virtual ~Data();
  virtual Type type () const = 0;
  virtual std::shared_ptr<Data> clone() const = 0;

};

}  // namespace sensor
}  // namespace PI

#endif  // SENSOR_INCLUDE_SENSOR_DATA_DATA_H_
