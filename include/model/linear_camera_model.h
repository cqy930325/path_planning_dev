/** PerceptIn license **/

#ifndef MODEL_INCLUDE_LINEAR_CAMERA_MODEL_H_
#define MODEL_INCLUDE_LINEAR_CAMERA_MODEL_H_

#include <model/camera_model.h>

namespace PI {
namespace model {

/**
 * Linear camera model simply models the relation between the image coordinate
 * and the normalized image coordinate through the camera matrix. In other words,
 * there's no distortion in the linear model.
 *
 * The linear camera model assumes the camera matrix is
 *    [ fx  0 cx;
 *       0 fy cy;
 *       0  0  1]
 *
 * The linear camera model is useful when given a set of undistorted points.
 */
class LinearCameraModel : public CameraModel {
 public:
  /**
   * @brief Default constructor setting camera matrix to identity and zero
   *        distortion.
   */
  LinearCameraModel();
  explicit LinearCameraModel(const cv::Size &image_size);
  LinearCameraModel(const cv::Size &image_size,
                    const cv::Matx33d &camera_matrix);
  LinearCameraModel(const LinearCameraModel &other);
  std::shared_ptr<Model> Clone() const override;

  static std::string class_type();
  std::string model_type() const override;

  /**
   * @brief Convert any camera to the linear camera model by ignoring all the
   *        distortion models.
   * @param camera_model Source camera model.
   * @return Newly created linear camera model.
   */
  static std::shared_ptr<LinearCameraModel> Convert(
      std::shared_ptr<const CameraModel> camera_model);

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
};

}  // namespace model
}  // namespace PI

#endif  // MODEL_INCLUDE_LINEAR_CAMERA_MODEL_H_
