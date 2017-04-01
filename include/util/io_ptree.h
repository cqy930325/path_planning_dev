/** PerceptIn license **/

#ifndef UTIL_INCLUDE_UTIL_IO_PTREE_H_
#define UTIL_INCLUDE_UTIL_IO_PTREE_H_

#include <map>
#include <vector>

#define BOOST_SPIRIT_THREADSAFE
#include <boost/property_tree/ptree.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

namespace PI {
namespace util {

/**
 * @brief Serialize an OpenCV's ROWxCOL matrix into a boost property tree.
 * @param matrix The matrix to serialize.
 * @return Serialized ROWxCOL matrix in boost property tree format.
 */
template<typename T, int ROW, int COL>
boost::property_tree::ptree SerializeCvMatx(
    const cv::Matx<T, ROW, COL> &matrix);

/**
 * @brief Deserialize an OpenCV's ROWxCOL matrix from a boost property tree.
 * @param ptree The boost's property tree to deserialize.
 * @return The ROWxCOL matrix deserialized from the boost property tree.
 */
template<typename T, int ROW, int COL>
cv::Matx<T, ROW, COL> DeserializeCvMatx(
    const boost::property_tree::ptree &ptree);

/**
 * @brief Serialize an OpenCV's Vec into a boost property tree.
 * @param matrix The matrix to serialize.
 * @return Serialized Vec in boost property tree format.
 */
template<typename T, int NUM>
boost::property_tree::ptree SerializeCvVec(const cv::Vec<T, NUM> &vec);

/**
 * @brief Deserialize an OpenCV's Vec from a boost property tree.
 * @param ptree The boost's property tree to deserialize.
 * @return The Vec deserialized from the boost property tree.
 */
template<typename T, int NUM>
cv::Vec<T, NUM> DeserializeCvVec(const boost::property_tree::ptree &ptree);

/**
 * @brief Serialize a OpenCV's affine transform into a boost property tree.
 * @param transform The transform to serialize.
 * @return Serialized transform in boost property tree format.
 */
boost::property_tree::ptree SerializeTransform(const cv::Affine3d &transform);

/**
 * @brief Deserialize a OpenCV's affine transform from a boost property tree.
 * @param ptree The boost's property tree to deserialize.
 * @return The affine transform deserialized from the boost property tree.
 */
cv::Affine3d DeserializeTransform(const boost::property_tree::ptree &ptree);

/**
 * @brief Serialize a std::vector into a boost property tree.
 * @param vector The vector to serialize.
 * @return Serialized vector in boost property tree format.
 */
template<typename T>
boost::property_tree::ptree SerializeStdVector(const std::vector<T> &vector);

/**
 * @brief Deserialize a std::vector from a boost property tree.
 * @param ptree The boost's property tree to deserialize.
 * @return The std::vector deserialized from the boost property tree.
 */
template<typename T>
std::vector<T> DeserializeStdVector(const boost::property_tree::ptree &ptree);

/**
 * @brief Serialize a std::map into a boost property tree.
 * @param vector The map to serialize.
 * @return Serialized map in boost property tree format.
 */
template<typename Key, typename Value>
boost::property_tree::ptree SerializeStdMap(const std::map<Key, Value> &map);

/**
 * @brief Deserialize a std::map from a boost property tree.
 * @param ptree The boost's property tree to deserialize.
 * @return The std::map deserialized from the boost property tree.
 */
template<typename Key, typename Value>
std::map<Key, Value> DeserializeStdMap(const boost::property_tree::ptree &ptree);

}  // namespace util
}  // namespace PI

#include <util/io_ptree.hpp>

#endif  // UTIL_INCLUDE_UTIL_IO_PTREE_H_
