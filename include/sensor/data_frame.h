/** PerceptIn license **/

#ifndef SENSOR_INCLUDE_SENSOR_DATA_FRAME_H_
#define SENSOR_INCLUDE_SENSOR_DATA_FRAME_H_

#include <map>
#include <memory>

#include <glog/logging.h>

#include <sensor/data/data.h>
#include <sensor/sensors.h>
#include <util/debug.h>

namespace PI {
namespace sensor {

/**
 * DataFrame contains all Data from a single snapshot.
 */
class DataFrame {
 public:
  explicit DataFrame(const size_t timestamp_ms = 0);
  ~DataFrame();

  // Timestamp in ms.
  size_t timestamp_ms() const;
  // Timestamp in second.
  double timestamp_sec() const;

  /**
   * @brief Check if this DataFrame contains Data with a specific ID.
   * @param id ID to check.
   * @return True if a Data with this ID exists.
   */
  bool Exists(const Id id) const;

  /**
   * @brief Check if this DataFrame contains Data of a specific type with a
   *        specific ID.
   * @param id ID to check.
   * @return True if a Data of the specified type with this ID exists.
   */
  template <Data::Type TYPE>
  bool Exists(const Id id) const {
    const auto iter = id_data_map_.find(id);
    if (iter == id_data_map_.end()) {
      return false;
    }
    return iter->second->type() == TYPE;
  }

  /**
   * @brief Check if this DataFrame contains Data of a specific type with a
   *        specific ID.
   * @param id ID to check.
   * @return True if a Data of the specified type with this Id exists.
   */
  template <typename T>
  bool Exists(const Id id) const {
    const auto iter = id_data_map_.find(id);
    if (iter == id_data_map_.end()) {
      return false;
    }
    const std::shared_ptr<const T> data_casted =
        std::dynamic_pointer_cast<const T>(iter->second);
    if (!data_casted) {
      return false;
    }
    return true;
  }

  /**
   * @brief Get the Data of a specified ID.
   * @param Id ID to access.
   * @return Pointer to the const Data. Nullptr if Data of the specified ID is
   *         missing.
   */
  std::shared_ptr<const Data> At(const Id id) const;

  /**
   * @brief Get the Data of a specified ID. Data is mutable.
   * @param Id ID to access.
   * @return Pointer to the Data. Nullptr if Data of the specified ID is missing.
   */
  std::shared_ptr<Data> At(const Id id);

  /**
   * @brief Get the Data of a specific type of a specified ID.
   * @param Id ID to access.
   * @return Pointer to the const Data of a specific type. Nullptr if Data of
   *         the specified ID is missing, or if the Data is not the specified
   *         type.
   */
  template <typename T>
  std::shared_ptr<const T> At(const Id id) const {
    const auto iter = id_data_map_.find(id);
    if (iter == id_data_map_.end()) {
      INTERNAL(
      VLOG(1) << "[" << sensor::ConvertSensorIdToName(id) << "] does not exist.";
      )
      return nullptr;
    }
    std::shared_ptr<const T> data_casted =
        std::dynamic_pointer_cast<const T>(iter->second);
    if (!data_casted) {
      INTERNAL(
      VLOG(1) << "[" << sensor::ConvertSensorIdToName(id)
              << "] exists but not the specified type.";
      )
      return nullptr;
    }
    return data_casted;
  }

  /**
   * @brief Get the Data of a specific type of a specified ID. Data is mutable.
   * @param Id ID to access.
   * @return Pointer to the Data of a specific type. Nullptr if Data of the
   *         specified ID is missing, or if the Data is not the specified type.
   */
  template <typename T>
  std::shared_ptr<T> At(const Id id) {
    const auto iter = id_data_map_.find(id);
    if (iter == id_data_map_.end()) {
      INTERNAL(
      VLOG(1) << "[" << sensor::ConvertSensorIdToName(id) << "] does not exist.";
      )
      return nullptr;
    }
    std::shared_ptr<T> data_casted = std::dynamic_pointer_cast<T>(iter->second);
    if (!data_casted) {
      INTERNAL(
      VLOG(1) << "[" << sensor::ConvertSensorIdToName(id)
              << "] exists but not the specified type.";
      )
      return nullptr;
    }
    return data_casted;
  }

  /**
   * @brief Insert a data to the frame.
   * @param id ID of the data.
   * @param data A pointer to the Data to insert.
   * @param clone If true, maintain a deep copy of the data. If false, use the
   *              existing object, which allows the data to be modified outside
   *              of this class.
   * @return The data is being added. Return false if the data with the same ID
   *         exists.
   */
  bool Insert(const Id id,
              const std::shared_ptr<Data> data,
              const bool clone = true);

  /** Getters **/
  std::map<Id, std::shared_ptr<const Data>> map() const;

  /** Setters **/
  const std::map<Id, std::shared_ptr<Data>> &map();

 private:
  /// Timestamp in ms.
  const size_t timestamp_ms_;

  /// A map from the data source ID to the (pointer to the) actual data.
  std::map<Id, std::shared_ptr<Data>> id_data_map_;
};

}  // namespace sensor
}  // namespace PI

#endif  // SENSOR_INCLUDE_SENSOR_DATA_FRAME_H_
