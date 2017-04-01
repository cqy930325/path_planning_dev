/** PerceptIn license **/

#ifndef MODEL_INCLUDE_PINHOLE_CAMERA_MODEL_H_
#define MODEL_INCLUDE_PINHOLE_CAMERA_MODEL_H_

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <model/camera_model.h>

namespace PI {
namespace model {

/**
 * Pinhole camera calibration with radial distortion model based on Opencv.
 * @note Field of view angle needs to be set explicitly through getter. Default
 *       field of view angle is 0.
 */
class PinholeCameraModel : public CameraModel {
 public:
  /**
   * @brief Default constructor setting camera matrix to identity and zero
   *        distortion.
   */
  PinholeCameraModel();
  explicit PinholeCameraModel(const cv::Size &image_size);
  PinholeCameraModel(const cv::Size &image_size,
                     const cv::Matx33d &camera_matrix,
                     const cv::Matx<double, 5, 1> &distortion);
  PinholeCameraModel(const PinholeCameraModel &other);

  /**
   * @copydoc Model::Clone()
   */
  std::shared_ptr<Model> Clone() const override;

  /**
   * @brief A string specifier for the class.
   */
  static std::string class_type();
  /**
   * @copydoc Model::model_type()
   */
  std::string model_type() const override;

  /** Getters **/
  const cv::Matx<double, 5, 1> &distortion() const;
  Eigen::Matrix<double, 5, 1> distortion_eigen() const;
  double field_of_view_angle_degree() const;
  double valid_image_radius() const;

  /** Setters **/
  void distortion(const cv::Matx<double, 5, 1> &distortion);
  void distortion(const cv::Mat &distortion);
  void field_of_view_angle_degree(const double angle);

 private:
  /**
   * @copydoc CameraModel::SetCameraMatrixImpl()
   */
  void SetCameraMatrixImpl(const cv::Matx33d &camera_matrix_org,
                           const cv::Matx33d &camera_matrix_new) override;

  /**
   * @copydoc CameraModel::IsPointInViewImpl()
   */
  bool IsPointInViewImpl(const cv::Point3d &p_cam) const override;

  /**
   * @copydoc CameraModel::IsPointInViewImpl()
   */
  bool IsPointInViewImpl(const cv::Point2d &p_img) const override;

  /**
   * @copydoc CameraModel::ImageToRayImpl()
   */
  void ImageToRayImpl(const cv::Point2d &p_img,
                      cv::Point2d *p_ray) const override;

  /**
   * @copydoc CameraModel::ProjectPointImpl()
   */
  void ProjectPointImpl(const cv::Point3d &p_cam,
                        cv::Point2d *p_img) const override;

  /**
   * @copydoc CameraModel::UndistortImpl()
   */
  void UndistortImpl(const cv::Point2d &p_img,
                     cv::Point2d *p_img_undistorted) const override;

  /**
   * @copydoc CameraModel::UndistortImageImpl()
   */
  void UndistortImageImpl(cv::Mat *image) const override;

  /// Distortion coefficients according to OpenCV.
  cv::Matx<double, 5, 1> distortion_;
  /// tan(fov / 2): Tangent of half of the field of view angle used to determine
  /// whether a 3d point is within the valid field of view or not.
  double tan_half_fov_angle_squared_;
  /// A 2d image point is valid only when its distance to principal point is
  /// within this radius.
  double valid_image_radius_squared_;
};

}  // namespace model
}  // namespace PI

#endif  // MODEL_INCLUDE_PINHOLE_CAMERA_MODEL_H_
