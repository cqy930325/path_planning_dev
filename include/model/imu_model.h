/** PerceptIn license **/

#ifndef MODEL_INCLUDE_IMU_MODEL_H_
#define MODEL_INCLUDE_IMU_MODEL_H_

#include <array>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <model/model.h>

namespace PI {
namespace model {

/**
 * IMU model:
 *  For both gyro and accel signals,
 *      signal_corrected = TK * (AxesTransform(signal_raw) - bias),
 *  where TK is a 3-by-3 matrix, and AxesTransform is a function to transform
 *  signal from the original coordinate to the desired "right-handed" coordinate.
 *
 * @note The underlying structure of the ImuModel is defined using Eigen. Thus,
 *       all OpenCv interfaces would be less efficient.
 */
class ImuModel : public Model {
 public:
  /**
   * A rotation from original coordinate (left-handed or right-handed) to a
   * desired "right-handed" coordinate. This rotation consists of 0, 1, and -1
   * (i.e. this rotation only swap and/or flip the three axes.). Thus, we use
   * two arrays to represent the relation between the original and the desired
   * axes. The i-th axes (where i = 0, 1, 2) of the desired coordinate is the
   * j-th axes of the original coordinate, where j = index_[i]. The direction
   * of the axes is determined by sign_[i].
   *
   * For example, to represent V = (v[0], v[1], v[2]) = (u[1], u[2], -u[0]),
   * where V is the desired coordinate and U is the original coordinate, set
   *    index = {1, 2, 0}
   *    sign  = {true, true, false}.
   */
  class AxesTransform {
   public:
    /**
     * @brief Default constructor creates an Identity transform.
     */
    AxesTransform();
    AxesTransform(const std::array<size_t, 3> &index,
                  const std::array<bool, 3> &sign);
    AxesTransform(const AxesTransform &other);
    ~AxesTransform();

    /**
     * @return True if this transform switches the handedness of the coordinate.
     *         False, otherwise.
     */
    bool SwitchingHandedness() const;

    /**
     * @return The signal in the desired right-handed coordinate.
     */
    Eigen::Vector3d Transform(const Eigen::Vector3d &raw) const;

    /**
     * @brief Obtain the inverse of this axes transform.
     * @param[out] inverse Inverse axes transform.
     */
    void Inverse(AxesTransform *inverse) const;

    /** Getters */
    const std::array<size_t, 3> &index() const;
    const std::array<bool, 3> &sign() const;

    /** Setters */
    void index(const std::array<size_t, 3> &index);
    void sign(const std::array<bool, 3> &sign);
    // NOTE: Not allowing the caller to set each element in index() because the
    // class needs to make sure all axes from the original coordinate are
    // assigned to a axis in the new coordinate.
    std::array<bool, 3> &sign();

   private:
    std::array<size_t, 3> index_;
    std::array<bool, 3> sign_;
  };

  /**
   * @brief Default constructor creates an ideal IMU with no bias, no skew and
   *        the axes between accelerometer and gyroscope are perfectly aligned.
   */
  ImuModel(const AxesTransform &axes);
  ImuModel(const AxesTransform &axes,
           const Eigen::Vector3d &accel_bias,
           const Eigen::Matrix3d &accel_TK,
           const Eigen::Vector3d &gyro_bias,
           const Eigen::Matrix3d &gyro_TK);
  ImuModel(const AxesTransform &axes,
           const cv::Vec3d &accel_bias,
           const cv::Matx33d &accel_TK,
           const cv::Vec3d &gyro_bias,
           const cv::Matx33d &gyro_TK);
  ImuModel(const ImuModel &other);

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

  /** Getters */
  const Eigen::Vector3d &accel_bias() const;
  const Eigen::Matrix3d &accel_TK() const;
  const Eigen::Vector3d &gyro_bias() const;
  const Eigen::Matrix3d &gyro_TK() const;
  const AxesTransform &axes() const;  // Return axes transform for accelerometer.

  /** Setters */
  void accel_bias(const Eigen::Vector3d &accel_bias);
  void accel_TK(const Eigen::Matrix3d &accel_TK);
  void gyro_bias(const Eigen::Vector3d &gyro_bias);
  void gyro_TK(const Eigen::Matrix3d &gyro_TK);

  /**
   * @brief Correct a raw accelerometer signal.
   * @return Corrected accelerometer signal.
   */
  cv::Vec3d CorrectAccel(const cv::Vec3d &raw) const;

  /**
   * @brief Correct a raw accelerometer signal.
   * @return Corrected accelerometer signal.
   */
  Eigen::Vector3d CorrectAccel(const Eigen::Vector3d &raw) const;

  /**
   * @brief Correct a raw gyroscope signal.
   * @return Corrected gyroscope signal.
   */
  cv::Vec3d CorrectGyro(const cv::Vec3d &raw) const;

  /**
   * @brief Correct a raw gyroscope signal.
   * @return Corrected gyroscope signal.
   */
  Eigen::Vector3d CorrectGyro(const Eigen::Vector3d &raw) const;

 private:
  /**
   * Define the parameters to correct a raw signal.
   * signal_corrected = TK * (signal_raw - bias).
   */
  struct Correction {
    Eigen::Vector3d bias;
    Eigen::Matrix3d TK;
    Correction(const Eigen::Vector3d &bias,
               const Eigen::Matrix3d &TK);
    Correction(const cv::Vec3d &bias,
               const cv::Matx33d &TK);
    Correction(const Correction &other);
  };

  /**
   * @brief Correct a raw signal.
   * @param signal_raw Raw signal to correct.
   * @param axes Axes transform to apply to the raw signal.
   * @param correction Correction to apply.
   * @return Corrected signal.
   */
  static cv::Vec3d Correct(const cv::Vec3d &signal_raw,
                           const AxesTransform &axes,
                           const Correction &correction);

  /**
   * @brief Correct a raw signal.
   * @param signal_raw Raw signal to correct.
   * @param axes Axes transform to apply to the raw signal.
   * @param correction Correction to apply.
   * @return Corrected signal.
   */
  static Eigen::Vector3d Correct(const Eigen::Vector3d &signal_raw,
                                 const AxesTransform &axes,
                                 const Correction &correction);

  /// Axes transform for the accelerometer.
  const AxesTransform accel_axes_;
  /// Axes transform for the gyroscope. The only difference between this
  /// transform and #accel_axes_ is their #sign are opposite if the #accel_axes_
  /// is a left-handed transform (i.e. the transform changes the handedness.)
  /// because gyroscope measures "angular" velocity.
  /// NOTE: The SwitchingHandedness() shouldn't be used on gyro axes.
  const AxesTransform gyro_axes_;

  /// Correction for accelerometer.
  Correction accel_;
  /// Correction for gyroscope.
  Correction gyro_;
};

}  // namespace model
}  // namespace PI

#endif  // MODEL_INCLUDE_IMU_MODEL_H_
