/** PerceptIn license **/

#ifndef UTIL_INCLUDE_UTIL_IO_PTREE_HPP_
#define UTIL_INCLUDE_UTIL_IO_PTREE_HPP_

#include <glog/logging.h>

#include <util/io_ptree.h>

namespace PI {
namespace util {

template<typename T, int ROW, int COL>
boost::property_tree::ptree SerializeCvMatx(
    const cv::Matx<T, ROW, COL> &matrix) {
  boost::property_tree::ptree pt_matrix;
  for (int r = 0; r < ROW; r++) {
    boost::property_tree::ptree pt_row;
    for (int c = 0; c < COL; c++) {
      // Create an unnamed value.
      boost::property_tree::ptree pt_cell;
      pt_cell.put_value(matrix.val[r * COL + c]);
      // Add the value to this row.
      pt_row.push_back(std::make_pair("", pt_cell));
    }
    // Add the row to our matrix.
    pt_matrix.push_back(std::make_pair("", pt_row));
  }
  return pt_matrix;
}

template<typename T, int ROW, int COL>
cv::Matx<T, ROW, COL> DeserializeCvMatx(
    const boost::property_tree::ptree &ptree) {
  cv::Matx<T, ROW, COL> matrix;
  int r = 0;
  for (const boost::property_tree::ptree::value_type &row : ptree) {
    int c = 0;
    CHECK_LT(r, ROW);
    for (const boost::property_tree::ptree::value_type &cell : row.second) {
      CHECK_LT(c, COL);
      matrix.val[r * COL + (c++)] = cell.second.get_value<T>();
    }
    CHECK_EQ(c, COL);
    r++;
  }
  CHECK_EQ(r, ROW);
  return matrix;
}

template<typename T, int NUM>
boost::property_tree::ptree SerializeCvVec(const cv::Vec<T, NUM> &vec) {
  boost::property_tree::ptree pt_vec;
  for (int i = 0; i < NUM; i++) {
    boost::property_tree::ptree pt_cell;
    pt_cell.put_value(vec.val[i]);
    pt_vec.push_back({"", pt_cell});
  }
  return pt_vec;
}

template<typename T, int NUM>
cv::Vec<T, NUM> DeserializeCvVec(const boost::property_tree::ptree &ptree) {
  cv::Vec<T, NUM> vec;
  int i = 0;
  for (const boost::property_tree::ptree::value_type &val : ptree) {
    CHECK_LT(i, NUM);
    vec.val[i++] = val.second.get_value<T>();
  }
  CHECK_EQ(i, NUM);
  return vec;
}

template<typename T>
boost::property_tree::ptree SerializeStdVector(const std::vector<T> &vector) {
  boost::property_tree::ptree pt_vec;
  for (auto const &val : vector) {
    boost::property_tree::ptree pt_cell;
    pt_cell.put_value(val);
    pt_vec.push_back({"", pt_cell});
  }
  return pt_vec;
}

template<typename T>
std::vector<T> DeserializeStdVector(const boost::property_tree::ptree &ptree) {
  std::vector<T> vec;
  for (const boost::property_tree::ptree::value_type &val : ptree) {
    vec.push_back(val.second.get_value<T>());
  }
  return vec;
}

template<typename Key, typename Value>
boost::property_tree::ptree SerializeStdMap(const std::map<Key, Value> &map) {
  boost::property_tree::ptree pt_map;
  for (auto const &key_value : map) {
    boost::property_tree::ptree pt_cell;
    pt_cell.put("key", key_value.first);
    pt_cell.put("value", key_value.second);
    pt_map.push_back({"" , pt_cell});
  }
  return pt_map;
}

template<typename Key, typename Value>
std::map<Key, Value> DeserializeStdMap(
    const boost::property_tree::ptree &ptree) {
  std::map<Key, Value> map;
  for (const boost::property_tree::ptree::value_type &key_val : ptree) {
    map.insert({key_val.second.get<Key>("key"),
                key_val.second.get<Value>("value")});
  }
  return map;
}

}  // namespace util
}  // namespace PI

#endif  // UTIL_INCLUDE_UTIL_IO_PTREE_HPP_
