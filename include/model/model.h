/** PerceptIn license **/

#ifndef MODEL_INCLUDE_MODEL_H_
#define MODEL_INCLUDE_MODEL_H_

#include <memory>

namespace PI {
namespace model {

/**
 * Define an API to store sensor calibration.
 */
class Model {
 public:
  Model();
  virtual ~Model();

  /**
   * @brief Clone the model.
   */
  virtual std::shared_ptr<Model> Clone() const = 0;

  /**
   * @brief A string defining the type of the model.
   * @note Must also implement a static function std::string class_type().
   */
  virtual std::string model_type() const = 0;
};

}  // namespace model
}  // namespace PI

#endif  // MODEL_INCLUDE_MODEL_H_
