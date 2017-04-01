/** PerceptIn license **/

#ifndef UTIL_INCLUDE_UTIL_COMPARE_H_
#define UTIL_INCLUDE_UTIL_COMPARE_H_

#include <map>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

namespace PI {
namespace util {

/**
 * @brief Check if two (single-channel) cv::Mat matrices are identical.
 * @tparam T Type of the values in the cv::Matx.
 * @param mat_1 One cv::Matx to compare to.
 * @param mat_2 The other cv::Matx to compare to.
 * @param eps A small number to compare floating points values.
 * @return True if all the elements in both cv::Mat matrices are identical.
 */
template<typename T>
bool EqualCvMats(const cv::Mat &mat_1,
                 const cv::Mat &mat_2,
                 const T eps) {
  CHECK_EQ(mat_1.channels(), 1);
  CHECK_EQ(mat_2.channels(), 1);

  if (mat_1.rows != mat_2.rows) {
    return false;
  }
  if (mat_1.cols != mat_2.cols) {
    return false;
  }

  for (int i = 0; i < mat_1.total(); i++) {
    if (std::fabs(mat_1.at<T>(i) - mat_2.at<T>(i)) > eps) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Check if two cv::Matx matrices are identical.
 * @tparam T Type of the values in the cv::Matx.
 * @tparam ROW Number of rows in the cv::Matx.
 * @tparam COL Number of columns in the cv::Matx.
 * @param mat_1 One cv::Matx to compare to.
 * @param mat_2 The other cv::Matx to compare to.
 * @param eps A small number to compare floating points values.
 * @return True if all the elements in both cv::Matx matrices are identical.
 */
template<typename T, int ROW, int COL>
bool EqualCvMatxs(const cv::Matx<T, ROW, COL> &mat_1,
                  const cv::Matx<T, ROW, COL> &mat_2,
                  const T eps) {
  for (int r = 0; r < ROW; r++) {
    for (int c = 0; c < COL; c++) {
      if (std::fabs(mat_1(r, c) - mat_2(r, c)) > eps) {
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief Check if two cv::Vecs matrices are identical.
 * @tparam T Type of the values in the cv::Vec.
 * @tparam NUM Number of elements in the cv::Vec.
 * @param vec_1 One cv::Vecs to compare to.
 * @param vec_2 The other cv::Vecs to compare to.
 * @param eps A small number to compare floating points values.
 * @return True if all the elements in both cv::Vecs matrices are identical.
 */
template<typename T, int NUM>
bool EqualCvVecs(const cv::Vec<T, NUM> &vec_1,
                 const cv::Vec<T, NUM> &vec_2,
                 const T eps) {
  return EqualCvMatxs(vec_1, vec_2, eps);
}

/**
 * @brief Check if two Eigen::Matrix matrices are identical.
 * @tparam T Type of the values in the Eigen::Matrix.
 * @tparam ROW Number of rows in the Eigen::Matrix.
 * @tparam COL Number of columns in the Eigen::Matrix.
 * @param mat_1 One Eigen::Matrix to compare to.
 * @param mat_2 The other Eigen::Matrix to compare to.
 * @param eps A small number to compare floating points values.
 * @return True if all the elements in both cv::Matx matrices are identical.
 */
template<typename T, int ROW, int COL>
bool EqualEigenMatrixs(const Eigen::Matrix<T, ROW, COL> &matrix_1,
                       const Eigen::Matrix<T, ROW, COL> &matrix_2,
                       const T eps) {
  for (int r = 0; r < ROW; r++) {
    for (int c = 0; c < COL; c++) {
      if (std::fabs(matrix_1(r, c) - matrix_2(r, c)) > eps) {
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief Check if two std::vector with integer values are identical.
 * @tparam T Type of the values in the vector.
 * @param vector_1 One vector to compare to.
 * @param vector_2 The other vector to compare to.
 * @return True if #vector_1 and #vector_2 have the same size and all the
 *         elements are identical.
 */
template<typename T>
bool EqualIntStdVectors(const std::vector<T> &vector_1,
                        const std::vector<T> &vector_2) {
  if (vector_1.size() != vector_2.size()) {
    return false;
  }
  for (size_t i = 0; i < vector_1.size(); i++) {
    if (vector_1[i] != vector_2[i]) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Check if two std::map with integer keys and floating point values
 *        are identical.
 * @tparam Key Type of the keys in the map.
 * @tparam Value Type of the values in the map.
 * @param map_1 One map to compare to.
 * @param map_2 The other map to compare to.
 * @return True if #map_1 and #map_2 have the same set of keys and the
 *         corresponding values are equal.
 */
template<typename Key, typename Value>
bool EqualIntKeyFloatValueStdMaps(const std::map<Key, Value> &map_1,
                                  const std::map<Key, Value> &map_2,
                                   const Value eps) {
  if (map_1.size() != map_2.size()) {
    return false;
  }
  for (auto const &key_val : map_1) {
    auto iter = map_2.find(key_val.first);
    if (iter == map_2.end()) {
      return false;
    }
    if (std::fabs(iter->second - key_val.second) > eps) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Check if two 3d transforms are identical (or equivalent).
 * @param transform_1 One transform to compare to.
 * @param transform_2 The other transform to compare to.
 * @param eps A small number to compare floating points values.
 * @return True both transforms are identical (or equivalent).
 */
bool EqualTransforms(const cv::Affine3d &transform_1,
                     const cv::Affine3d &transform_2,
                     const double eps);

}  // namespace util
}  // namespace PI

#endif  // UTIL_INCLUDE_UTIL_COMPARE_H_
