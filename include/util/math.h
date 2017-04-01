/** PerceptIn license **/

#ifndef UTIL_INCLUDE_UTIL_MATH_H_
#define UTIL_INCLUDE_UTIL_MATH_H_

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace PI {
namespace util {

/**
 * @brief Obtain the 3-by-3 skew symmetric matrix [a]x from the vector,
 *        a = [a_1, a_2, a_3]^T.
 *
 *        [a]x = [   0 -a_3  a_2;
 *                 a_3    0 -a_1;
 *                -a_2  a_1    0]
 *
 * @param a The vector a.
 * @return The 3-by-3 skew symmetric matrix.
 */
Eigen::Matrix3d SkewSymmetricMatrix(const Eigen::Vector3d &a);

/**
 * @brief Obtain the 3-by-3 skew symmetric matrix [a]x from the vector,
 *        a = [a_1, a_2, a_3]^T.
 *
 *        [a]x = [   0 -a_3  a_2;
 *                 a_3    0 -a_1;
 *                -a_2  a_1    0]
 *
 * @param a The vector a.
 * @return The 3-by-3 skew symmetric matrix.
 */
cv::Matx33d SkewSymmetricMatrix(const cv::Vec3d &a);

/**
 * @brief QR decomposition with Gram Schmidt process.
 * @param U The matrix to decompose into U = QR.
 * @param[out] Q The orthogonal matrix Q.
 * @param[out] R The upper triangular matrix R.
 */
void QRDecompositionWithGramSchmidt(const Eigen::MatrixXd &U,
                                    Eigen::MatrixXd *Q,
                                    Eigen::MatrixXd *R);

}  // namespace util
}  // namespace PI

#endif  // UTIL_INCLUDE_UTIL_MATH_H_
