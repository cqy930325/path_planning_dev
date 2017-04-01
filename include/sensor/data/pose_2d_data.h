/** PerceptIn license **/

#ifndef SENSOR_INCLUDE_SENSOR_DATA_POSE_DATA_H_
#define SENSOR_INCLUDE_SENSOR_DATA_POSE_DATA_H_

#include <sensor/data/data.h>
#include <util/affine2d.h>

namespace PI {
namespace sensor {

/**
 * Pose2dData stores 2D pose data (3-dof).
 * This is the transform from the local to the global coordinate.
 */
class Pose2dData : public Data {
 public:
  Pose2dData(const double theta,
             const cv::Vec2d &tvec);
  Pose2dData(const util::Affine2d &global_T_local);
  Pose2dData(const Pose2dData &other);
  Type type() const;
  std::shared_ptr<Data> clone() const;

  /** Getters **/
  const util::Affine2d &global_T_local() const;

  /** Setters **/
  util::Affine2d &global_T_local();
  void global_T_local(const util::Affine2d &global_T_local);

 private:
  util::Affine2d global_T_local_;
};

}  // namespace sensor
}  // namespace PI

#endif  // SENSOR_INCLUDE_SENSOR_DATA_POSE_DATA_H_
