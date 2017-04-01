/** PerceptIn license **/

#ifndef UTIL_INCLUDE_UTIL_AFFINE2D_H_
#define UTIL_INCLUDE_UTIL_AFFINE2D_H_

#include <opencv2/core.hpp>

namespace PI {
namespace util {

/**
 * 2D affine transform between two coordinates:
 *    p_dst = dst_R_src * p_src + dst_t_src
 * where
 *    dst_R_src = [ cosθ sinθ;
 *                 -sinθ cosθ]
 *    dst_t_src = [tx; ty]
 *
 * The transform can be represented in the homogeneous coordinate:
 *    dst_T_src = [dst_R_src dst_t_src;
 *                     0         1     ]
 */
class Affine2d {
 public:
  /**
   * @brief Default constructor initializes θ and (tx, ty) to 0.
   */
  Affine2d();
  /**
   * @brief Constructor.
   * @param theta Unit: Radian
   */
  Affine2d(const double theta,
           const cv::Vec2d &tvec);
  Affine2d(const Affine2d &other);

  /**
   * @brief Create identity transform.
   */
  static Affine2d Identity();

  /**
   * @brief Compute inverse transform.
   */
  Affine2d Inverse() const;

  /** Getters **/
  const double theta() const;
  const cv::Matx22d rotation() const;
  const cv::Vec2d &translation() const;
  const cv::Matx33d matrix() const;

  /** Setters **/
  void theta(const double val);
  void rotation(const cv::Matx22d &mat);
  void translation(const cv::Vec2d &vec);

 private:
  /// Rotation angle.
  double theta_;
  /// X-axis of the translation vector.
  cv::Vec2d tvec_;
};

/**
 * @brief Combine two affine transforms: a = a1 * a2.
 */
Affine2d operator*(const Affine2d &a1, const Affine2d &a2);

}  // namespace util
}  // namespace PI

#endif  // UTIL_INCLUDE_UTIL_AFFINE2D_H_
