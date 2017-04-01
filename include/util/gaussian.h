/** PerceptIn license **/

#ifndef UTIL_INCLUDE_UTIL_GAUSSIAN_H_
#define UTIL_INCLUDE_UTIL_GAUSSIAN_H_

#include <opencv2/core.hpp>

namespace PI {
namespace util {
/**
 * @brief Get value of a zero-mean Gaussian distribution.
 * @tparam DIM Dimension of the Gaussian distribution.
 * @param x The random vector of dimension DIM.
 * @param cov Covariance of the Gaussian distribution.
 * @return Value of the zero-mean Gaussian distribution at #x.
 */
template<int DIM>
double Gaussian(const cv::Vec<double, DIM> &x,
                const cv::Matx<double, DIM, DIM> &cov) {
  CHECK(IsCovarianceMatrix(cov, 1e-6));
  const double kNormalizer = std::sqrt(std::pow(2 * M_PI, DIM));
  return std::exp(-0.5 * (x.t() * cov.inv() * x)(0)) /
      kNormalizer / std::sqrt(cv::determinant(cov));
}

/**
 * @brief Get value of a zero-mean Gaussian distribution.
 * @tparam DIM Dimension of the Gaussian distribution.
 * @param x The random vector of dimension DIM.
 * @param mean The mean of the Gaussian distribution.
 * @param cov Covariance of the Gaussian distribution.
 * @return Value of the zero-mean Gaussian distribution at #x.
 */
template<int DIM>
double Gaussian(const cv::Vec<double, DIM> &x,
                const cv::Vec<double, DIM> &mean,
                const cv::Matx<double, DIM, DIM> &cov) {
  return Gaussian(x - mean, cov);
}

}  // namespace util
}  // namespace PI

#endif  // UTIL_INCLUDE_UTIL_GAUSSIAN_H_