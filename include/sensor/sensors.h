/** PerceptIn license **/

#ifndef SENSOR_INCLUDE_SENSOR_SENSORS_H_
#define SENSOR_INCLUDE_SENSOR_SENSORS_H_

#include <string>

namespace PI {
namespace sensor {

/**
 * Defines all possible sensors.
 */
enum Id {
  /// IMU
  ID_IMU,
  /// Monocular camera.
  ID_MONO_CAMERA,
  /// Left stereo camera
  ID_STEREO_LEFT,
  /// Right stereo camera.
  ID_STEREO_RIGHT,
  /// Depth camera.
  ID_DEPTH_CAMERA,
  /// Data from robot.
  ID_BOT,
  // An abstract sensor placed at the origin of the entire device.
  ID_ROOT,
};

/**
 * @brief Convert sensor Id to its human-readable name.
 * @param id Id of the sensor.
 * @return Human-readable name of the sensor.
 */
std::string ConvertSensorIdToName(const Id id);

/**
 * @brief Convert human-readable sensor name to its Id.
 * @param name Human-readable name of the sensor.
 * @return Id of the sensor.
 */
Id ConvertSensorNameToId(const std::string &name);

}  // namespace PI
}  // namespace sensor

#endif  // SENSOR_INCLUDE_SENSOR_SENSORS_H_
