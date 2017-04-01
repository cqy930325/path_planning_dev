/** PerceptIn license **/

#ifndef UTIL_INCLUDE_UTIL_GEOMETRY_H_
#define UTIL_INCLUDE_UTIL_GEOMETRY_H_

#include <map>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace PI {
namespace util {

/**
 * @brief Find a rotation matrix that transform a vector from the source
 *        coordinate to the destination coordinate.
 * @param v_dst A vector in the destination coordinate.
 * @param v_src A vector in the source coordinate.
 * @note There are multiple solutions, and this function returns one.
 */
cv::Matx33d FindDstRSrc(const cv::Vec3d &v_dst, const cv::Vec3d &v_src);

/**
 * @brief Check if a N-by-N matrix fulfills all criteria of being a covariance
 *        matrix.
 * @tparam DIM Dimension of the covariance matrix.
 * @param matrix The matrix to evaluate.
 * @param eps A small number to compare floating points values.
 * @return True if #matrix is fulfills all criteria of being a covariance matrix.
 */
template<int DIM>
bool IsCovarianceMatrix(const cv::Matx<double, DIM, DIM> &matrix,
                        const double eps) {
  // Trace of a covariance matrix must be positive.
  if (cv::trace(matrix) < 0.0) {
    return false;
  }
  // Determinant of a covariance matrix must be positive.
  if (cv::determinant(matrix) < 0.0) {
    return false;
  }
  // The covariance matrix is symmetric, and the diagonal must be positive.
  for (int i = 0; i < DIM; i++) {
    if (matrix(i, i) < 0.0) {
      return false;
    }
    for (int j = i + 1; j < DIM; j++) {
      if (std::fabs(matrix(i, j) - matrix(j, i)) > eps) {
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief Transform covariance matrix from one parameterization to another.
 * @tparam DIM_DST Dimension of the destination parameterization.
 * @tparam DIM_SRC Dimension of the source parameterization.
 * @param cov_src Covariance matrix of the source parameterization.
 * @param dst_J_src Jacobian that transform the covariance from source to
 *                  destination parameterization.
 * @return Covariance matrix of the destination parameterization.
 */
template<int DIM_DST, int DIM_SRC>
cv::Matx<double, DIM_DST, DIM_DST> TransformCovariance(
    const cv::Matx<double, DIM_SRC, DIM_SRC> &cov_src,
    const cv::Matx<double, DIM_DST, DIM_SRC> &dst_J_src) {
  return dst_J_src * cov_src * dst_J_src.t();
}

/**
 * @brief Obtain the Jacobian from a transform represented by (om, t) to its
 *        (R, t) representation.
 * @param om Rotation vector of the transform.
 * @return The 12-by-6 Jacobian matrix.
 */
cv::Matx<double, 12, 6> GetTransformFormatJacobian(const cv::Vec3d &om);

/**
 * @brief Obtain the Jacobian from a transform represented by (R, t) to its
 *        (om, t) representation.
 * @param R Rotation matrix of the transform.
 * @return The 12-by-6 Jacobian matrix.
 */
cv::Matx<double, 6, 12> GetTransformFormatJacobian(const cv::Matx33d &R);

/**
 * @brief Obtain the Jacobian of a transform (dst_T_src) to its inverse
 *        (src_T_dst).
 * @param dst_T_src The transform to inverse.
 * @return The 6-by-6 Jacobian matrix.
 */
cv::Matx66d GetInverseTransformJacobian(const cv::Affine3d &dst_T_src);

/**
 * @brief Obtain the Jacobian of a transform (src_T_ref) to another transform
 *        (dst_T_ref) given the transform between the two coordinate (dst_T_src),
 *            dst_T_ref = dst_T_src * src_T_ref.
 * @param src_T_ref The source transform with uncertainty.
 * @param dst_T_src Transform from source to destination coordinate.
 * @return The 6-by-6 Jacobian matrix.
 */
cv::Matx66d GetTransformJacobian(const cv::Affine3d &src_T_ref,
                                 const cv::Affine3d &dst_T_src);

/**
 * @brief Orthonomalize a rotation matrix.
 * @param[in, out] rotation Rotation matrix to normalize.
 */
void OrthonormalizeRotationMatrix(cv::Matx33d *rotation);

/**
 * @brief Convert a rotation matrix to Euler angles.
 *
 * Equivalent to MATLAB's dcm2angle with rotationSequence = 'ZYX'.
 *
 * Reference:
 * https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2012/07/euler-angles1.pdf
 *
 * @param R The 3-by-3 rotation matrix.
 * @param[out] euler_rad Eular angles. (Unit: radian).
 */
void RotationMatrixToEulerRadians(const Eigen::Matrix3d &R,
                                  Eigen::Vector3d *euler_rad);

/**
 * @brief Convert Euler angles to a rotation matrix.
 *
 * Equivalent to MATLAB's angle2dcm with rotationSequence = 'ZYX'.
 * Reference:
 * https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2012/07/euler-angles1.pdf
 *
 * @param euler_rad Eular angles. (Unit: radian).
 * @param[out] R The 3-by-3 rotation matrix.
 */
void EulerRadiansToRotationMatrix(const Eigen::Vector3d& euler_rad,
                                  Eigen::Matrix3d *R);

/**
 * @brief Find the rotation matrix that transform a vector from the source to
 *        the destination coordinate, Only the direction of the vector is used.
 * @param v_src Vector in the source coordinate.
 * @param v_dst Vector in the destination coordinate.
 * @return True if the rotation matrix is computed.
 */
bool FindRotationMatrixFromTwoVectors(const cv::Vec3d &v_dst,
                                      const cv::Vec3d &v_src,
                                      cv::Matx33d *dst_R_src);

}  // namespace util
}  // namespace PI

#endif  // UTIL_INCLUDE_UTIL_GEOMETRY_H_
