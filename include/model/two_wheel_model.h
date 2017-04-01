/** PerceptIn license **/

#ifndef CALIB_INCLUDE_CALIB_MODEL_TWO_WHEEL_MODEL_H_
#define CALIB_INCLUDE_CALIB_MODEL_TWO_WHEEL_MODEL_H_

#include <model/wheel_model.h>

namespace PI {
namespace model {

/**
 * TODO(gtsai): Describe the two wheel model.
 */
class TwoWheelModel : public WheelModel {
 public:
  /**
   * @brief Constructor.
   * @note The radius of the wheels and the distance between the wheels are set
   *       to 0.
   */
  TwoWheelModel();

  /**
   * @brief Constructor.
   * @param radius Radius of the wheels.
   * @param distance Distance between two wheels.
   */
  TwoWheelModel(const double radius,
                const double distance);

  /**
   * @brief Constructor.
   * @param radius_left Radius of the left wheel.
   * @param radius_right Radius of the right wheel.
   * @param distance Distance between two wheels.
   */
  TwoWheelModel(const double radius_left,
                const double radius_right,
                const double distance);

  /**
   * @brief Copy constructor.
   */
  TwoWheelModel(const TwoWheelModel &other);

  ~TwoWheelModel();

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
  double radius_left() const;
  double radius_right() const;
  double circumference_left() const;
  double circumference_right() const;
  double distance() const;

  /** Setters **/
  void radius_left(const double value);
  void radius_right(const double value);
  void circumference_left(const double value);
  void circumference_right(const double value);
  void distance(const double value);

 private:
  /// Radius of the left wheel. (Unit: meter)
  double radius_left_;
  /// Circumference of the left wheel. (Unit: meter)
  /// Note, keep a copy of both the radius and the circumference for efficiency.
  double circumference_left_;
  /// Radius of the left wheel. (Unit: meter)
  double radius_right_;
  /// Circumference of the right wheel. (Unit: meter)
  /// Note, keep a copy of both the radius and the circumference for efficiency.
  double circumference_right_;
  /// Distance between two wheels.
  double distance_;
};

}  // namespace model
}  // namespace PI


#endif  // CALIB_INCLUDE_CALIB_MODEL_TWO_WHEEL_MODEL_H_
