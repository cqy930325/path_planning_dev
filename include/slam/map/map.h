/** PerceptIn license **/

#ifndef SLAM_INCLUDE_SLAM_MAP_MAP_H_
#define SLAM_INCLUDE_SLAM_MAP_MAP_H_

#include <string>

namespace PI {
namespace slam {

/**
 * A Map is a spatial representation of a physical environment. A derived class
 * has complete freedom to design the representation.
 */
class Map {
 public:
  Map();
  virtual ~Map();

  /**
   * @brief A string defining the type of the Map.
   * @note Must also implement a static function std::string class_type().
   */
  virtual std::string type() const = 0;

  /**
   * @brief Check whether the map is completely empty (e.g. no information in
   *        the map).
   * @return True if the map is empty.
   */
  virtual bool IsEmpty() const = 0;
};

}  // namespace slam
}  // namespace PI

#endif  // SLAM_INCLUDE_SLAM_MAP_MAP_H_
