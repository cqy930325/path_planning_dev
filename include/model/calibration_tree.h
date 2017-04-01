/** PerceptIn license **/

#ifndef MODEL_INCLUDE_CALIBRATION_TREE_H_
#define MODEL_INCLUDE_CALIBRATION_TREE_H_

#include <list>
#include <map>
#include <memory>
#include <set>

#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

#include <sensor/sensors.h>
#include <util/debug.h>

namespace PI {
namespace model {

// Forward declaration.
class Model;

/**
 * This data structure stores the intrinsics (CalibrationModels) + extrinsics
 * (3d coordinate transforms) of a set of sensors.
 */
class CalibrationTree {
 public:
  explicit CalibrationTree(const sensor::Id id,
                           const std::shared_ptr<const Model> model = nullptr);
  /**
   * @brief Copy constructor with a deep copy the tree.
   */
  CalibrationTree(const CalibrationTree &other);

  /**
   * @brief Reset the model by deep copying the source model.
   * @param model Source model.
   */
  void SetModel(std::shared_ptr<const Model> model);

  /**
   * @brief Add a child (a subtree) to this tree. This function creates a deep
   *        copy of the input subtree. A subtree can only be added if all the
   *        nodes in the subtree do not exist in this tree.
   * @param child The subtree to add.
   * @param this_T_child Transform from the child's coordinate to this coordinate.
   * @return True if the subtree is successfully added.
   */
  bool AddChild(const CalibrationTree &child,
                const cv::Affine3d &this_T_child);

  /**
   * @brief Replace a child (a subtree) to this tree. This function creates a
   *        deep copy of the input subtree. A subtree can only be added if all
   *        the nodes in the subtree only exists in the original subtree to be
   *        replaced.
   * @param child The subtree to replace.
   * @param this_T_child Transform from the child's coordinate to this coordinate.
   * @return True if the subtree is successfully replaced.
   */
  bool ReplaceChild(const CalibrationTree &child,
                    const cv::Affine3d &this_T_child);

  /**
   * @brief Remove a child (a subtree) to this tree.
   * @return True if the subtree is successfully removed.
   */
  bool RemoveChild(const sensor::Id id);

  /**
   * @brief Check if a subtree (specified by its ID) is a direct child of this
   *        tree.
   * @param id ID of the subtree to check.
   * @return True if #id is a direct subtree of this tree.
   */
  bool IsChild(const sensor::Id id) const;

  /**
   * @brief Check if a subtree (specified by its ID) exist in this tree.
   * @param id ID of the subtree to check.
   * @return True if #id is a subtree of this tree.
   */
  bool IsDescendant(const sensor::Id id) const;

  /**
   * @brief Get the IDs of all the descendants of this tree.
   * @return IDs of all the descendants.
   */
  std::set<sensor::Id> GetDescendantsId() const;

  /**
   * @brief Get a sub-CalibrationTree in this CalibrationTree.
   * @param id Id of the root of the sub-CalibrationTree.
   * @return The sub-CalibrationTree with the specified root.
   * @warning Fail if the subtree does not exist. Use IsDescendant() to check
   *          before calling this function.
   */
  const CalibrationTree &GetTree(const sensor::Id id) const;

  /**
   * @brief Get a mutable sub-CalibrationTree in this CalibrationTree.
   * @param id Id of the root of the sub-CalibrationTree.
   * @return The sub-CalibrationTree with the specified root.
   * @warning Fail if the subtree does not exist. Use IsDescendant() to check
   *          before calling this function.
   */
   CalibrationTree &GetMutableTree(const sensor::Id id);

  /**
   * @brief Get the model of a node in a tree.
   * @param tree The tree to retrieve the model from.
   * @param id Id of the model's data source.
   * @return The calibration model of data source with the specified ID. Nullptr
   *         if the ID is not part of the tree.
   */
  static std::shared_ptr<const Model> GetModel(const CalibrationTree &tree,
                                               const sensor::Id id);

  /**
   * @brief Get the specific type of model of a node in a tree.
   * @param tree The tree to retrieve the model from.
   * @param id Id of the model's data source.
   * @return The calibration model of data source with the specified ID. Nullptr
   *         if the ID is not part of the tree or the model is not the specified
   *         type.
   */
  template <typename T>
  static std::shared_ptr<const T> GetModel(const CalibrationTree &tree,
                                           const sensor::Id id) {
    if (!tree.IsDescendant(id)) {
      INTERNAL(
      LOG(ERROR) << "[" << sensor::ConvertSensorIdToName(id)
                 << "] does not exist in the tree.";
      )
      return nullptr;
    }

    std::shared_ptr<const T> model_casted =
        std::dynamic_pointer_cast<const T>(tree.GetTree(id).model_);
    if (!model_casted) {
      INTERNAL(
      LOG(ERROR) << "[" << sensor::ConvertSensorIdToName(id)
                 << "] exists but is not the specified type.";
      )
      return nullptr;
    }
    return model_casted;
  }

  /**
   * @brief Get the transform between two nodes in a tree.
   * @param tree The tree to retrieve the transform from.
   * @param dst The ID of the destination coordinate of the transform.
   * @param src The ID of the source coordinate of the transform.
   * @param[out] Transform from the #src coordinate to the #dst coordinate.
   * @return True if the transform is retrieved (i.e. both #dst and #src exist
   *         in the #tree).
   */
  static bool GetTransform(const CalibrationTree &tree,
                           const sensor::Id dst,
                           const sensor::Id src,
                           cv::Affine3d *dst_T_src);

  /**
   * @brief Collect intrinsic and extrinsic calibration of a set of sensors of a
   *        device.
   * @param calib Calibration of the device.
   * @param id_device_root Id of the device's origin.
   * @param sensors Sensors to retrieve calibration for.
   * @param[out] models Sensor intrinsics.
   * @param[out] sensors_Ts_device Sensor extrinsics.
   * @warning Fail if calibrations of the cameras are missing.
   */
  static bool GetCalibrationForSensors(
      const CalibrationTree &tree,
      const sensor::Id id_device_root,
      const std::set<sensor::Id> &sensors,
      std::map<sensor::Id, std::shared_ptr<const Model>> *models,
      std::map<sensor::Id, cv::Affine3d> *sensors_Ts_device);

  /** Getters **/
  std::shared_ptr<const Model> model() const;
  sensor::Id id() const;
  const std::list<std::pair<CalibrationTree, cv::Affine3d>> &children() const;

 private:
  /**
   * @brief Get the transform from a specified descendant to this coordinate.
   * @param id Id of the descendant.
   * @return Transform from the specified descendant to this coordinate.
   * @warning Fail if the specified #id is not a descendant of this tree.
   * @note This function is a helper for GetTransform().
   */
  cv::Affine3d GetThisTDescendant(const sensor::Id id) const;

  /// Id of the data source.
  const sensor::Id id_;
  /// Model that describes the intrinsics of the data source.
  std::shared_ptr<const Model> model_;
  /// List of children. For each child, store the child's tree and this_T_child
  /// (the transform from child's coordinate to this coordinate).
  std::list<std::pair<CalibrationTree, cv::Affine3d>> children_;
};

}  // namespace model
}  // namespace PI

#endif  // MODEL_INCLUDE_CALIBRATION_TREE_H_
