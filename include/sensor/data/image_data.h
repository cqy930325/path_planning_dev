/** PerceptIn license **/

#ifndef SENSOR_INCLUDE_SENSOR_DATA_IMAGE_DATA_H_
#define SENSOR_INCLUDE_SENSOR_DATA_IMAGE_DATA_H_

#include <opencv2/opencv.hpp>

#include <sensor/data/data.h>

namespace PI {
namespace sensor {

/**
 * ImageData stores data captured from a camera as an image.
 */
class ImageData : public Data {
 public:
  ImageData();
  explicit ImageData(const cv::Mat &image);
  ImageData(const ImageData &other);
  ImageData &operator=(const ImageData &other);
  Type type() const;
  std::shared_ptr<Data> clone() const;

  /** Getters **/
  const cv::Mat& image() const;
  cv::Mat image_mutable();

 private:
  cv::Mat image_;
};

}  // namespace sensor
}  // namespace PI

#endif  // SENSOR_INCLUDE_SENSOR_DATA_IMAGE_DATA_H_
