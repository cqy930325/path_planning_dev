/** PerceptIn license **/

#ifndef UTILS_INCLUDE_SYN_COMMON_TYPES_H_
#define UTILS_INCLUDE_SYN_COMMON_TYPES_H_

#include <map>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

namespace PI {
namespace util {

/**
 * @brief Fill in the values of a cv::Mat with random integer values.
 * @tparam T Type of the values in the cv::Mat.
 * @param[in, out] mat The cv::Mat to fill.
 * @param randomizer Random number generator.
 */
template<typename T>
void FillCvMat(cv::Mat_<T> *mat,
               std::default_random_engine &randomizer,
               std::true_type) {
  CHECK_NOTNULL(mat);
  std::uniform_int_distribution<T> random_value(
      std::numeric_limits<T>::lowest(), std::numeric_limits<T>::max());
  for (int i = 0; i < mat->total(); i++) {
    mat->template at<T>(i) = random_value(randomizer);
  }
}

/**
 * @brief Fill in the values of a cv::Mat with random floating-point values.
 * @tparam T Type of the values in the cv::Mat.
 * @param[in, out] mat The cv::Mat to fill.
 * @param randomizer Random number generator.
 */
template<typename T>
void FillCvMat(cv::Mat_<T> *mat,
               std::default_random_engine &randomizer,
               std::false_type) {
  CHECK_NOTNULL(mat);
  std::uniform_real_distribution<T> random_value(-1000.0, 1000.0);
  for (int i = 0; i < mat->total(); i++) {
    mat->template at<T>(i) = random_value(randomizer);
  }
}

/**
 * @brief Fill in the values of a cv::Mat with random values.
 * @tparam T Type of the values in the cv::Mat.
 * @param[in, out] mat The cv::Mat to fill.
 * @param randomizer Random number generator.
 */
template<typename T>
void FillCvMat(cv::Mat_<T> *mat, std::default_random_engine &randomizer) {
  CHECK_NOTNULL(mat);
  FillCvMat(mat, randomizer, std::is_integral<T>());
}

/**
 * @brief Create a random single-channel cv::Mat.
 * @tparam T Type of the values in the cv::Mat.
 * @tparam ROW Number of rows in the cv::Mat.
 * @tparam COL Number of columns in the cv::Mat.
 * @param randomizer Random number generator.
 * @return A random cv::Mat with values of type T.
 */
template<typename T>
cv::Mat_<T> CreateCvMat(std::default_random_engine &randomizer) {
  // Uniformly sample size of the matrix.
  std::uniform_int_distribution<int> random_size(1, 100);
  // Create the matrix.
  cv::Mat_<T> mat(random_size(randomizer), random_size(randomizer));
  FillCvMat(&mat, randomizer);
  return mat;
}

/**
 * @brief Create a random cv::Matx.
 * @tparam T Type of the floating-point values in the cv::Matx.
 * @tparam ROW Number of rows in the cv::Matx.
 * @tparam COL Number of columns in the cv::Matx.
 * @param randomizer Random number generator.
 * @return A random #ROW x #COL cv::Matx with values of type T.
 */
template<typename T, int ROW, int COL>
cv::Matx<T, ROW, COL> CreateCvMatx(std::default_random_engine &randomizer) {
  // Uniformly sample values to create the matrix.
  std::uniform_real_distribution<T> random_value(-1000.0, 1000.0);
  cv::Matx<T, ROW, COL> mat;
  for (int r = 0; r < ROW; r++) {
    for (int c = 0; c < COL; c++) {
      mat(r, c) = random_value(randomizer);
    }
  }
  return mat;
}

/**
 * @brief Create a random cv::Vec.
 * @tparam T Type of the floating-point values in the cv::Vec.
 * @tparam NUM Number of elements in the cv::Vec.
 * @param randomizer Random number generator.
 * @return A random cv::Vec with #NUM of values of type T.
 */
template<typename T, int NUM>
cv::Vec<T, NUM> CreateCvVec(std::default_random_engine &randomizer) {
  return cv::Vec<T, NUM>(CreateCvMatx<T, NUM, 1>(randomizer).val);
}

/**
 * @brief Create a random normalized cv::Vec.
 * @tparam T Type of the floating-point values in the cv::Vec.
 * @tparam NUM Number of elements in the cv::Vec.
 * @param randomizer Random number generator.
 * @return A random cv::Vec with #NUM of values of type T.
 */
template<typename T, int NUM>
cv::Vec<T, NUM> CreateUnitCvVec(std::default_random_engine &randomizer) {
  return cv::normalize(CreateCvVec<T, NUM>(randomizer));
}

/**
 * @brief Create a random Eigen::Matrix.
 * @tparam T Type of the floating-point values in the Eigen::Matrix.
 * @tparam ROW Number of rows in the Eigen::Matrix.
 * @tparam COL Number of columns in the Eigen::Matrix.
 * @param randomizer Random number generator.
 * @return A random #ROW x #COL Eigen::Matrix with values of type T.
 */
template<typename T, int ROW, int COL>
Eigen::Matrix<T, ROW, COL> CreateEigenMatrix(
    std::default_random_engine &randomizer,
    T min_val = -1000.0,
    T max_val = 1000.0) {
  // Uniformly sample values to create the matrix.
  std::uniform_real_distribution<T> random_value(min_val, max_val);
  Eigen::Matrix<T, ROW, COL> mat;
  for (int r = 0; r < ROW; r++) {
    for (int c = 0; c < COL; c++) {
      mat(r, c) = random_value(randomizer);
    }
  }
  return mat;
}

/**
 * @brief Create a random std::vector with integer values.
 * @tparam T Type of the values in the vector.
 * @param randomizer Random number generator.
 * @param num_elements (optional) Number of elements in the vector. If negative,
 *                     random.
 * @return A random std::vector of type T with #num_elements (if specified).
 */
template<typename T>
std::vector<T> CreateIntStdVector(std::default_random_engine &randomizer,
                                  const int num_elements = -1) {
  std::vector<T> vec;
  if (num_elements < 0) {
    std::uniform_int_distribution<size_t> random_number(0, 100);
    vec.resize(random_number(randomizer));
  } else {
    vec.resize(num_elements);
  }

  // Uniformly sample values to fill in the vector.
  std::uniform_int_distribution<T> random_value(
      std::numeric_limits<T>::lowest(), std::numeric_limits<T>::max());
  for (size_t i = 0; i < vec.size(); i++) {
    vec[i] = random_value(randomizer);
  }
  return vec;
}

/**
 * @brief Create a random std::map with integer keys and floating point values.
 * @tparam Key Type of the keys in the map.
 * @tparam Value Type of the values in the map.
 * @param randomizer Random number generator.
 * @param num_elements (optional) Number of elements in the map. If negative,
 *                     random.
 * @return A random std::map.
 */
template<typename Key, typename Value>
std::map<Key, Value> CreateIntKeyFloatValueStdMap(
    std::default_random_engine &randomizer, const int num_elements = -1) {
  // Uniformly sample number of elements in the map.
  std::uniform_int_distribution<size_t> random_number(0,
      std::min(Key(100), std::numeric_limits<Key>::max()));
  const size_t num_elements_to_add =
      num_elements >= 0 ? num_elements : random_number(randomizer);

  // Uniformly sample keys to fill in the map.
  std::uniform_int_distribution<Key> random_key(
      std::numeric_limits<Key>::lowest(), std::numeric_limits<Key>::max());
  // Uniformly sample values to fill in the map.
  std::uniform_real_distribution<Value> random_value(-1000.0, 1000.0);

  std::map<Key, Value> map;
  while (map.size() < num_elements_to_add) {
    map[random_key(randomizer)] = random_value(randomizer);
  }
  return map;
}

/**
 * @brief Create a random 3d rotation represented as a rotation vector.
 * @param randomizer Random number generator.
 * @return A random rotation vector.
 */
cv::Vec3d CreateRotationVector(std::default_random_engine &randomizer);

/**
 * @brief Create a random 3d rotation matrix.
 * @param randomizer Random number generator.
 * @return A random 3-by-3 rotation matrix.
 */
cv::Matx33d CreateRotationMatrix(std::default_random_engine &randomizer);

/**
 * @brief Create a random 3d transform (Unit: meter).
 * @param randomizer Random number generator.
 * @return A random transform.
 */
cv::Affine3d CreateTransform(std::default_random_engine &randomizer);

/**
 * @brief Create a 2-by-2 covariance matrix in a 2d space (Unit: meter).
 * @param randomizer Random number generator.
 * @param correlated Flag to specify whether there is a correlation between the
 *                   two dimensions. If false, the covariance matrix is a
 *                   diagonal matrix.
 * @return A random 2-by-2 covariance matrix.
 */
cv::Matx22d Create2by2Covariance(std::default_random_engine &randomizer,
                                 const bool correlated);

/**
 * @brief Create a 3-by-3 covariance matrix in a 3d space (Unit: meter).
 * @param randomizer Random number generator.
 * @param correlated Flag to specify whether there is a correlation among the
 *                   three dimensions. If false, the covariance matrix is a
 *                   diagonal matrix.
 * @return A random 3-by-3 covariance matrix.
 */
cv::Matx33d Create3by3Covariance(std::default_random_engine &randomizer,
                                 const bool correlated);

/**
 * @brief Create a 6-by-6 covariance matrix in the (om, t) space.
 * @param randomizer Random number generator.
 * @param correlated Flag to specify whether there is a correlation among the
 *                   three dimensions. If false, the covariance matrix is a
 *                   diagonal matrix.
 * @note Maximum variance is set to 10 meter.
 * @return A random 6-by-6 covariance matrix for (om, t) pose.
 */
cv::Matx66d CreatePoseCovariance(std::default_random_engine &randomizer,
                                 const bool correlated);

}  // namespace util
}  // namespace PI

#endif  // UTILS_INCLUDE_SYN_COMMON_TYPES_H_
