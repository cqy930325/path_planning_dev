/** PerceptIn license **/

#ifndef SENSOR_INCLUDE_SENSOR_DATA_DEPTH_DATA_H_
#define SENSOR_INCLUDE_SENSOR_DATA_DEPTH_DATA_H_

#include <opencv2/opencv.hpp>

#include <sensor/data/data.h>

namespace PI {
namespace sensor {

/**
 * DepthData stores data captured from a camera as an image.
 */
class DepthData : public Data {
 public:
  DepthData();
  explicit DepthData(const cv::Mat_<uint16_t> &depth,
                     const cv::Mat &intensity = cv::Mat());
  DepthData(const DepthData &other);
  DepthData &operator=(const DepthData &other);
  Type type() const;
  std::shared_ptr<Data> clone() const;

  /** Getters **/
  const cv::Mat_<uint16_t>& depth() const;
  const cv::Mat &intensity() const;

  /** Setters **/
  void depth(const cv::Mat_<uint16_t> &depth);
  void intensity(const cv::Mat &intensity);

 private:
  // Depth image.
  cv::Mat_<uint16_t> depth_;
  // Optional intensity image. Leave it empty if not available.
  cv::Mat intensity_;
};

}  // namespace sensor
}  // namespace PI

#endif  // SENSOR_INCLUDE_SENSOR_DATA_IMAGE_DATA_H_
