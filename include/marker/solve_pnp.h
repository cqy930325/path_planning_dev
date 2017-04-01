/** PerceptIn license **/

#ifndef MARKER_INCLUDE_MARKER_SOLVE_PNP_H_
#define MARKER_INCLUDE_MARKER_SOLVE_PNP_H_

#include <map>

#include <opencv2/core/affine.hpp>

#include <marker/correspondence3d2d.h>
#include <sensor/sensors.h>

namespace PI {

// Forward declaration.
namespace model {
class CameraModel;
class CalibrationTree;
}  // namespace model
namespace util {
template <typename T>
class StatisticWatcher;
}  // namespace util

namespace marker {

/**
 * @brief Perform perspective-n-point camera pose estimation from 3d reference
 *        to 2d image observation point correspondences.
 * @param correspondences Correspondences between 3d object and 2d image
 *                        observation.
 * @param camera_model Camera model of the observing camera.
 * @param max_rms_error Maximum RMS of the re-projection errors allowed from
 *                      the method.
 * @param[out] cam_T_pattern Transform from the pattern coordinate to the camera
 *                           coordinate.
 * @param[out] inliers Index of the correspondences that are used to compute the
 *                     pose.
 * @param[out] errors Statistics of the re-projection errors of the inliers
 *                    with the computed pose, \p cam_T_pattern.
 * @return True if the pose estimation was successful and within the allowed
 *         \p max_rms_error. False otherwise.
 */
bool SolvePnp(const std::vector<Correspondence3d2d> &correspondences,
              std::shared_ptr<const model::CameraModel> camera_model,
              const double max_rms_error,
              cv::Affine3d *cam_T_pattern,
              std::vector<size_t> *inliers = nullptr,
              util::StatisticWatcher<double> *errors = nullptr);

/**
 * @brief Perform perspective-n-point camera pose estimation from 3d reference
 *        to 2d image observation point correspondences from multiple cameras.
 * @param correspondences Correspondences between 3d object and 2d image
 *                        observation for each camera.
 * @param camera_models Models of the observing cameras.
 * @param cams_Ts_device Transform from the device's coordinate to each
 *                       observing camera.
 * @param max_rms_error Maximum RMS of the re-projection errors allowed from
 *                      the method.
 * @param[out] device_T_pattern Transform from the pattern coordinate to the
 *                              device coordinate.
 * @param[out] inliers Index of the correspondences that are used to compute the
 *                     pose.
 * @param[out] errors Statistics of the re-projection errors of the inliers
 *                    with the computed pose, \p cam_T_pattern.
 * @return True if the pose estimation was successful and within the allowed
 *         \p max_rms_error. False otherwise.
 */
bool SolvePnp(
    const std::map<sensor::Id, std::vector<Correspondence3d2d>>
        &correspondences,
    const std::map<sensor::Id, std::shared_ptr<const model::CameraModel>>
        &camera_models,
    const std::map<sensor::Id, cv::Affine3d> &cams_Ts_device,
    const double max_rms_error,
    cv::Affine3d *device_T_pattern,
    std::map<sensor::Id, std::vector<size_t>> *inliers = nullptr,
    util::StatisticWatcher<double> *errors = nullptr);

}  // namespace marker
}  // namespace PI

#endif  // MARKER_INCLUDE_MARKER_SOLVE_PNP_H_
