/** PerceptIn license **/

#ifndef CONFIG_INCLUDE_CONFIG_CONFIG_H_
#define CONFIG_INCLUDE_CONFIG_CONFIG_H_

#include <memory>
#include <unordered_map>

#include <boost/lexical_cast.hpp>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <glog/logging.h>

#include <util/debug.h>

// Overload boost::lexical_cast to our convention.
namespace boost {
template<>
bool lexical_cast<bool, std::string>(const std::string &str);
/**
 * Format:
 *    width,height
 *    (No space between the comma.)
 */
template<>
std::string lexical_cast(const cv::Size &size);
template<>
cv::Size lexical_cast(const std::string &str);
/**
 * Format:
 *    str1,str2,str3,...
 *    (No space between the commas.)
 */
template<>
std::string lexical_cast(const std::vector<std::string> &strs);
template<>
std::vector<std::string> lexical_cast(const std::string &str);
/**
 * Format:
 *    val1,val2,val3,...
 *    (No space between the commas.)
 */
template<>
std::string lexical_cast(const std::vector<double> &vals);
template<>
std::vector<double> lexical_cast(const std::string &str);
/**
 * Format:
 *    a,b,c
 *    (No space between the commas.)
 */
template<>
std::string lexical_cast(const Eigen::Vector3d &vec);
template<>
Eigen::Vector3d lexical_cast(const std::string &str);
/**
 * Format:
 *    a,b,c,d,e,f,g,h,i
 *    (No space between the commas.)
 *
 *     where the matrix is
 *        [a b c;
 *         d e f;
 *         g h i]
 */
template<>
std::string lexical_cast(const Eigen::Matrix3d &mat);
template<>
Eigen::Matrix3d lexical_cast(const std::string &str);
/**
 * Format:
 *    a,b,c
 *    (No space between the commas.)
 */
template<>
std::string lexical_cast(const cv::Vec3d &vec);
template<>
cv::Vec3d lexical_cast(const std::string &str);
/**
 * Format:
 *    a,b,c,d,e,f,g,h,i
 *    (No space between the commas.)
 *
 *     where the matrix is
 *        [a b c;
 *         d e f;
 *         g h i]
 */
template<>
std::string lexical_cast(const cv::Matx33d &mat);
template<>
cv::Matx33d lexical_cast(const std::string &str);
}  // namespace boost

namespace PI {
namespace config {

class Config {
 public:
  Config();
  ~Config();

  /**
   * @brief Check if this Config contains value with a specific key.
   * @param key Key to retrieve value from.
   * @return True if the Key exists.
   */
  bool Exists(const std::string &key) const;

  /**
   * @brief Add a value to this Config.
   * @param key Key of the value.
   * @param value Value (represented as string) to be added.
   * @param overwrite True to allow overwriting existing value with the same
   *                  key. False, no overwriting is allowed.
   * @return True if the value has been added. False if value with the same key
   *         exists while overwrite is not allowed.
   */
  bool Add(const std::string &key,
           const std::string &value,
           const bool overwrite = false);

  /**
   * @brief Add a value to this Config.
   * @param key Key of the value.
   * @param value Value to be added.
   * @param overwrite True to allow overwriting existing value with the same
   *                  key. False, no overwriting is allowed.
   * @return True if the value has been added. False if value with the same key
   *         exists while overwrite is not allowed.
   */
  template <typename T>
  bool Add(const std::string &key,
           const T &value,
           const bool overwrite = false) {
    const auto iter = key_value_map_.find(key);
    if (iter != key_value_map_.end() && !overwrite) {
      INTERNAL(LOG(INFO) << "Cannot overwrite existing key [" << key << "]";)
      return false;
    }
    key_value_map_[key] = boost::lexical_cast<std::string>(value);
    return true;
  }

  /**
   * @brief Get the value of a specified key.
   * @param key Key to retrieve value from.
   * @param default_value Returned value if the key does not exist.
   * @return Value.
   */
  template <typename T>
  T Get(const std::string &key, T default_value) const {
    const auto iter = key_value_map_.find(key);
    if (iter == key_value_map_.end()) {
      INTERNAL(LOG(INFO) << "[" << key << "] does not exist.";)
      return default_value;
    }
    return boost::lexical_cast<T>(iter->second);
  }

  /**
   * @brief Get the value of a specified key.
   * @param key Key to retrieve value from.
   * @return Value.
   * @warning Fail if the key does not exist.
   */
  template <typename T>
  T Get(const std::string &key) const {
    const auto iter = key_value_map_.find(key);
    if (iter == key_value_map_.end()) {
      INTERNAL(printf("[%s] does not exist in the config.\n", key.c_str());)
      INTERNAL(LOG(FATAL) << "[" << key << "] does not exist.";)
    }
    return boost::lexical_cast<T>(iter->second);
  }

  /** Getters */
  const std::unordered_map<std::string, std::string> & map() const;

 private:
  std::unordered_map<std::string, std::string> key_value_map_;
};

}  // namespace config
}  // namespace PI

#endif  // UTIL_INCLUDE_UTIL_CONFIG_H_
