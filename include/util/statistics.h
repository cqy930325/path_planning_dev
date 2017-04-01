/** PerceptIn license **/

#include <glog/logging.h>
#include <math.h>

#ifndef UTIL_INCLUDE_UTIL_STATISTICS_H_
#define UTIL_INCLUDE_UTIL_STATISTICS_H_

namespace PI {
namespace util {

/**
 * @brief Get the median of each dimension in a multidimensional vector. The
 *        list of vectors are specified by the stating and ending iterators. The
 *        underlying data structure for the multidimensional vector must define
 *        "operator (int i)" to access the i-th dimension of that vector.
 *
 * @tparam ITERATOR The type of iterator.
 * @tparam DIM Number of dimensions in a vector.
 * @param iter_begin Iterator of the first element.
 * @param iter_end Iterator past the last element.
 * @return The multidimensional vector where the element in each dimension is
 *         the median of that dimension.
 */
template <typename ITERATOR, int DIM>
typename ITERATOR::value_type GetMedian(const ITERATOR iter_begin,
                                        const ITERATOR iter_end) {
  CHECK(iter_begin != iter_end);

  // Get the return type.
  typedef typename ITERATOR::value_type T;
  T median;

  // Calculate median of each dimension.
  const size_t num_elements = std::distance(iter_begin, iter_end);
  std::vector<double> values;
  for (int i = 0; i < DIM; i++) {
    values.clear();
    values.reserve(num_elements);

    for (auto iter = iter_begin; iter != iter_end; iter++) {
      values.push_back((*iter)(i));
    }
    auto iter_middle = values.begin() + num_elements / 2;
    std::nth_element(values.begin(), iter_middle, values.end());
    median(i) = *(iter_middle);
  }
  return median;
}

/**
 * @brief Get the variance of each dimension in a multidimensional vector. The
 *        list of vectors are specified by the stating and ending iterators. The
 *        underlying data structure for the multidimensional vector must define
 *        "operator (int i)" to access the i-th dimension of that vector.
 * @tparam ITERATOR The type of iterator.
 * @tparam DIM Number of dimensions in a vector.
 * @param iter_begin Iterator of the first element.
 * @param iter_end Iterator past the last element.
 * @return The multidimensional vector where the element in each dimension is
 *         the variance of that dimension.
 */
template <typename ITERATOR, int DIM>
typename ITERATOR::value_type GetVariance(const ITERATOR iter_begin,
                                          const ITERATOR iter_end) {
  CHECK(iter_begin != iter_end);

  // Get the return type.
  typedef typename ITERATOR::value_type T;
  T variance;

  double num_elements = static_cast<double>(std::distance(iter_begin, iter_end));
  for (int i = 0; i < DIM; i++) {
    // Calculate mean.
    double mean = 0.0;
    for (auto iter = iter_begin; iter != iter_end; iter++) {
      mean += (*iter)(i);
    }
    mean = mean / num_elements;

    // Calculate variance.
    double &var = variance(i);
    var = 0.0;
    for (auto iter = iter_begin; iter != iter_end; iter++) {
      const double diff = (*iter)(i) - mean;
      var += diff * diff;
    }
    var = var / num_elements;
  }
  return variance;
}

/**
 * @brief Get the minimum value of each dimension in a multidimensional vector.
 *        The list of vectors are specified by the stating and ending iterators.
 *        The underlying data structure for the multidimensional vector must
 *        define "operator (int i)" to access the i-th dimension of that vector.
 * @tparam ITERATOR The type of iterator.
 * @tparam DIM Number of dimensions in a vector.
 * @param iter_begin Iterator of the first element.
 * @param iter_end Iterator past the last element.
 * @return The multidimensional vector where the element in each dimension is
 *         the minimum value of that dimension.
 */
template <typename ITERATOR, int DIM>
typename ITERATOR::value_type GetMin(const ITERATOR iter_begin,
                                     const ITERATOR iter_end) {
  CHECK(iter_begin != iter_end);

  // Get the return type.
  typedef typename ITERATOR::value_type T;
  T min_values;

  for (int i = 0; i < DIM; i++) {
    double &min_val = min_values[i];
    min_val = std::numeric_limits<double>::max();
    for (auto iter = iter_begin; iter != iter_end; iter++) {
      if ((*iter)(i) < min_val) {
        min_val = (*iter)(i);
      }
    }
  }
  return min_values;
}

/**
 * @brief Get the maximum value of each dimension in a multidimensional vector.
 *        The list of vectors are specified by the stating and ending iterators.
 *        The underlying data structure for the multidimensional vector must
 *        define "operator (int i)" to access the i-th dimension of that vector.
 * @tparam ITERATOR The type of iterator.
 * @tparam DIM Number of dimensions in a vector.
 * @param iter_begin Iterator of the first element.
 * @param iter_end Iterator past the last element.
 * @return The multidimensional vector where the element in each dimension is
 *         the maximum value of that dimension.
 */
template <typename ITERATOR, int DIM>
typename ITERATOR::value_type GetMax(const ITERATOR iter_begin,
                                     const ITERATOR iter_end) {
  CHECK(iter_begin != iter_end);

  // Get the return type.
  typedef typename ITERATOR::value_type T;
  T max_values;

  for (int i = 0; i < DIM; i++) {
    double &max_val = max_values[i];
    max_val = std::numeric_limits<double>::lowest();
    for (auto iter = iter_begin; iter != iter_end; iter++) {
      if ((*iter)(i) > max_val) {
        max_val = (*iter)(i);
      }
    }
  }
  return max_values;
}

/**
 * A watcher is a light-weight actor that keeps track of the statistics of a
 * sequence of datums.
 */
template <typename T>
class StatisticWatcher {
 public:
  StatisticWatcher() : num_(0),
                       sum_(0),
                       sum_sq_(0),
                       min_(nan("")),
                       max_(nan("")) {}
  ~StatisticWatcher() {}

  /**
   * @brief Add a datum to the watcher.
   * @param datum New datum to watch.
   */
  void Watch(T datum) {
    if (num_ == 0) {
      min_ = datum;
      max_ = datum;
    }
    num_++;
    sum_ = sum_ + datum;
    sum_sq_ = sum_sq_ + datum * datum;
    if (datum < min_) {
      min_ = datum;
    }
    if (datum > max_) {
      max_ = datum;
    }
  }

  /**
   * @brief Reset the watcher by clearing all datums.
   */
  void Reset() {
    num_ = 0;
    sum_ = 0;
    sum_sq_ = 0;
    min_ = nan("");
    max_ = nan("");
  }

  /** Getters **/
  size_t num() const { return num_; }
  T sum() const { return sum_; }
  T sum_squared() const { return sum_sq_; }
  double mean() const {
    if (num_ == 0) {
      return nan("");
    }
    return static_cast<double>(sum) / static_cast<double>(num);
  }
  double var() const {
    if (num_ == 0) {
      return nan("");
    }
    const double num_d = static_cast<double>(num_);
    return
        (static_cast<double>(sum_sq_) - static_cast<double>(sum_ * sum_) / num_d)
            / num_d;
  }
  double rms() const {
    if (num_ == 0) {
      return nan("");
    }
    return std::sqrt(static_cast<double>(sum_sq_) / static_cast<double>(num_));
  }
  T min() const { return min_; }
  T max() const { return max_; }

 private:
  /// Number of datums.
  size_t num_;
  /// Sum of all datums.
  T sum_;
  /// Sum of squared of all datums.
  T sum_sq_;
  /// Minimum among all datums.
  T min_;
  /// Maximum among all datums.
  T max_;
};

}  // namespace util
}  // namespace PI

#endif  // UTIL_INCLUDE_UTIL_STATISTICS_H_
