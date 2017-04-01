/** PerceptIn license **/

#include <Eigen/Core>
#include <Eigen/Geometry>

#ifndef UTIL_INCLUDE_UTIL_JPL_QUATERNION_H_
#define UTIL_INCLUDE_UTIL_JPL_QUATERNION_H_

namespace PI {
namespace util {

/**
 * Reference:
 * [1] Trawny, Nikolas, and Stergios I. Roumeliotis.
 *     "Indirect Kalman Filter for 3D Attitude Estimation"
 *     University of Minnesota, Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005).
 *
 * [2] W. G. Breckenridge,
 *     “Quaternions - Proposed Standard Conventions,”
 *     JPL, Tech. Rep. INTEROFFICE MEMO- RANDUM IOM 343-79-1199, 1999.
 *
 * [3] E. J. Lefferts, F. L. Markley, and M. D. Shuster,
 *     “Kalman filtering for spacecraft attitude estimation,”
 *     Journal of Guidance, Control, and Dynamics, vol. 5, no. 5, pp. 417–429,
 *     Sept.-Oct. 1982.
 *
 */
class jplQuaternion : public Eigen::Vector4d {
 public:
  // Required by Eigen.
  typedef Eigen::Vector4d Base;

  /**
   * @brief Default constructor. Initialized to [0, 0, 0, 1].
   */
  jplQuaternion();

  /**
   * @brief Constructor.
   */
  template <typename Scalar>
  jplQuaternion(Scalar q0, Scalar q1, Scalar q2, Scalar q3) :
      Base{double(q0), double(q1), double(q2), double(q3)} {
    Eigen::Vector4d& q = *this;
    q = q * (1.f / q.norm());
    if (q(3) < 0) {
      q = -q;
    }
  }

  /**
   * @brief Constructor using Eigen::QuaternionBase.
   */
  template <typename OtherDerived>
  explicit jplQuaternion(const Eigen::QuaternionBase<OtherDerived> &eigen_quat) {
    SetFromEigenQuaterniond(eigen_quat);
  }

  /**
   * @brief Constructor from MatrixBase. (Required by Eigen.)
   */
  template <typename OtherDerived>
  jplQuaternion(const MatrixBase<OtherDerived> &other) : Base{other} {
    Eigen::Vector4d& q = *this;
    q = q * (1.f / q.norm());
    if (q(3) < 0) {
      q = -q;
    }
  }

  /**
   * @brief Assignment operator. (Required by Eigen.)
   */
  template <typename OtherDerived>
  jplQuaternion &operator=(const MatrixBase<OtherDerived> &other) {
    Base::operator=(other);
    Eigen::Vector4d& q = *this;
    q = q * (1.f / q.norm());
    if (q(3) < 0) {
      q = -q;
    }
    return *this;
  }

  /**
   * @brief Convert the JPL Quaternion to Hamiltonian quaternion.
   *        References: [2] and [3].
   * @return The Hamiltonian quaternion.
   */
  Eigen::Quaterniond ToEigenQuaterniond() const;

  /**
   * @brief Convert the JPL Quaternion to a rotation matrix.
   * @return The rotation matrix.
   */
  Eigen::Matrix3d ToRotationMatrix() const;

  /**
   * @brief Convert the JPL Quaternion to a Euler angle representation.
   * @return The Euler angle. (Unit: radian).
   */
  Eigen::Vector3d ToEulerRadians() const;

  /**
   * @brief Set the JPL Quaternion by a Hamiltonian quaternion.
   *        References: [2] and [3].
   * @param eigen_quat Hamiltonian quaternion in Eigen to convert from.
   */
  template <typename OtherDerived>
  void SetFromEigenQuaterniond(
      const Eigen::QuaternionBase<OtherDerived> &eigen_quat) {
    (*this)(0) = -eigen_quat.x();
    (*this)(1) = -eigen_quat.y();
    (*this)(2) = -eigen_quat.z();
    (*this)(3) =  eigen_quat.w();
    // keep scalar value non-negative
    if ((*this)(3) < 0) {
      (*this) = -(*this);
    }
  }

  /**
   * @brief Set the JPL Quaternion by a rotation matrix.
   *
   * Reference:
   *    http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
   *
   * @param R The rotation matrix to convert from.
   */
  void SetFromRotationMatrix(const Eigen::Matrix3d &R);

  /**
   * @brief Set the JPL Quaternion by Eular angle representation.
   * @param euler_rad The Eular angle representation to convert from.
   */
  void SetFromEulerRadians(const Eigen::Vector3d& euler_rad);

  /*
   * @brief Set JPL Quaternion from a small rotation (in Eular angle) using the
   *        small angle approximation.
   */
  void SetFromDeltaTheta(const Eigen::Vector3d &dtheta);

  /**
   * @brief Set JPL Quaternion by the rotation that transform a vector in the
   *        source coordinate to the destination coordinate:
   *            v_dst = R(q) * v_src
   * @param v_src Vector in the source coordinate.
   * @param v_dst Vector in the destination coordinate.
   */
  void SetFromTwoVectors(const Eigen::Vector3d &v_src,
                         const Eigen::Vector3d &v_dst);

  /**
   * @brief Computes the inverse of the JPL Quaternion.
   *        (Override Eigen::MatrixBase<>::inverse.)
   */
  jplQuaternion inverse() const;

  /**
   * @brief Calculate R(q) = R_lhs(q) * R_rhs(q).
   * @param lhs R_lhs(q)
   * @param rhs R_rhs(q)
   * @return R(q)
   */
  static jplQuaternion Multiply(const jplQuaternion &lhs,
                                const jplQuaternion &rhs);

  /**
   * @brief Calculate R(q) = R_this(q) * R_rhs(q).
   * @param rhs R_rhs(q)
   * @return R(q)
   */
  jplQuaternion Multiply(const jplQuaternion& rhs) const;

  /**
   * @brief Calculate quaternion intergration.
   *        Reference: Page 11 [1].
   * @param omega_begin Beginning angular velocity. (Unit: Radian)
   * @param omega_end Ending angular velocity. (Unit: Radian)
   * @param q_begin Starting rotation in JPL Quaternion.
   * @param dt Integration period.
   * @param[out] q_end Ending rotation in JPL Quaternion.
   * @param[out] q_min
   * @param[out] q_fourth
   * @param[out] q_eighth
   */
  // TODO(gtsai, zzhang): What is q_min, q_fourth and q_eighth?
  static void Integrate(const Eigen::Vector3d &omega_begin,
                        const Eigen::Vector3d &omega_end,
                        const jplQuaternion &q_begin,
                        const double &dt,
                        jplQuaternion *q_end,
                        jplQuaternion *q_mid = nullptr,
                        jplQuaternion *q_fourth = nullptr,
                        jplQuaternion *q_eighth = nullptr);

  /** Getters **/
  double x() const;
  double y() const;
  double z() const;
  double w() const;
};

}  // namespace util
}  // namespace PI

#endif  // UTIL_INCLUDE_UTIL_JPL_QUATERNION_H_
