/** PerceptIn license **/

#ifndef MODEL_INCLUDE_CAMERA_MODEL_H_
#define MODEL_INCLUDE_CAMERA_MODEL_H_

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <model/model.h>

namespace PI {
namespace model {

class CameraModel : public Model {
 public:
  /**
   * @brief Default constructor setting both image width and height to 0. {
   */
  CameraModel();
  explicit CameraModel(const cv::Size &image_size);
  CameraModel(const CameraModel &other);

  /** Getters **/
  const cv::Size &image_size() const;
  const cv::Matx33d &camera_matrix() const;
  const cv::Matx33d &K() const;
  Eigen::Matrix3d camera_matrix_eigen() const;
  cv::Point2d principal_point() const;
  const double &fx() const;
  const double &fy() const;

  /** Setters **/
  void image_size(const cv::Size &image_size);
  void camera_matrix(const cv::Matx33d &camera_matrix);
  void camera_matrix(const cv::Mat &camera_matrix);

  /**
   * @brief Check if a 3D point is in the valid field of view.
   * @param p_cam The 3D point in the camera coordinate.
   * @return True if the point is in view.
   */
  bool IsPointInView(const cv::Point3d &p_cam) const;

  /**
   * @brief Check if a 2D point is within the valid field of view and within
   *        the image bound.
   * @param p_img The 2D image point.
   * @return True if the point is in view.
   */
  bool IsPointInView(const cv::Point2d &p_img) const;

  /**
   * @brief Transform a point from the 2d image coordinate to a 3d ray
   *        represented as (p_ray.x, p_ray.y, 1).
   * @param p_img The 2D image point.
   * @param[out] p_ray The 3d ray represented as (p_ray.x, p_ray.y, 1).
   * @return False if the point is outside of the valid bound.
   */
  bool ImageToRay(const cv::Point2d &p_img, cv::Point2d *p_ray) const;

  /**
   * @brief Project a point from the 3D camera coordinate to the 2D image
   *        coordinate.
   * @param p_cam The 3D camera point to be projected.
   * @param[out] p_img The projected 3d point.
   * @return False if the point is outside of the valid field of view.
   */
  bool ProjectPoint(const cv::Point3d &p_cam, cv::Point2d *p_img) const;

  /**
   * @brief Undistort a point in the 2d image coordinate.
   * @param p_img The point in the image coordinate.
   * @param p_img_undistorted The point in the undistorted image coordinate.
   * @return False if the point is outside of the valid bound.
   */
  bool Undistort(const cv::Point2d &p_img, cv::Point2d *p_img_undistorted) const;

  /**
   * @brief Undistort an image.
   * @param[in, out] The image to be undistroted.
   */
  void UndistortImage(cv::Mat *image) const;

 protected:
  /**
   * @brief Update components in the camera model (except for camera matrix,
   *        which will be handled by the base class) that are a function of the
   *        camera matrix.
   * @param camera_matrix_org Original camera matrix.
   * @param camera_matrix_new Updated camera matrix.
   */
  virtual void SetCameraMatrixImpl(const cv::Matx33d &camera_matrix_org,
                                   const cv::Matx33d &camera_matrix_new) = 0;

  /**
   * @brief Check if a 3D point is in the valid field of view.
   * @param p_cam The 3D point in the camera coordinate.
   * @return True if the point is in view.
   * @note The point is guaranteed to be in front of the camera (e.g. z >= 0).
   */
  virtual bool IsPointInViewImpl(const cv::Point3d &p_cam) const = 0;

  /**
   * @brief Check if a 2D point is within the valid field of view and within
   *        the image bound.
   * @param p_img The 2D image point.
   * @return True if the point is in view.
   * @note The point is guaranteed to be within the image boundary.
   */
  virtual bool IsPointInViewImpl(const cv::Point2d &p_img) const = 0;

  /**
   * @brief Transform a point from the 2d image coordinate to a 3d ray
   *        represented as (p_ray.x, p_ray.y, 1).
   * @param p_img The 2D image point.
   * @param[out] p_ray The 3d ray represented as (p_ray.x, p_ray.y, 1).
   * @note p_img is guaranteed to be within the field of view of the camera.
   *       Checked by calling IsPointInView(p_img).
   * @note p_ray is guaranteed to be not null.
   */
  virtual void ImageToRayImpl(const cv::Point2d &p_img,
                              cv::Point2d *p_ray) const = 0;

  /**
   * @brief Project a point from the 3D camera coordinate to the 2D image
   *        coordinate.
   * @param p_cam The 3D camera point to be projected.
   * @param[out] p_img The projected 3d point.
   * @note p_cam is guaranteed to be within the field of view of the camera.
   *       Checked by calling IsPointInView(p_cam).
   * @note p_img is guaranteed to be not null.
   */
  virtual void ProjectPointImpl(const cv::Point3d &p_cam,
                                cv::Point2d *p_img) const = 0;

  /**
   * @brief Undistort a point in the 2d image coordinate.
   * @param p_img The point in the image coordinate.
   * @param p_img_undistorted The point in the undistorted image coordinate.
   */
  virtual void UndistortImpl(const cv::Point2d &p_img,
                             cv::Point2d *p_img_undistorted) const = 0;

  /**
   * @brief Undistort an image.
   * @param[in, out] The image to be undistroted.
   * @note Image is guaranteed to be not null, and is guaranteed to be of the
   *       size of the camera model.
   */
  virtual void UndistortImageImpl(cv::Mat *image) const = 0;

 private:
  /// Size of the image captured by this camera.
  cv::Size image_size_;
  /// A 3-by-3 projection matrix.
  cv::Matx33d camera_matrix_;
};

}  // namespace model
}  // namespace PI

#endif  // MODEL_INCLUDE_CAMERA_MODEL_H_
