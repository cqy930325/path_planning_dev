/** PerceptIn license **/

#ifndef PERCEPTIN_V1_INTERFACE_H_
#define PERCEPTIN_V1_INTERFACE_H_

#include <memory>
#include <mutex>
#include <queue>
#include <thread>  // TODO(gtsai): Use boost thread.

#include <opencv2/core/core.hpp>
//#include <libuvc/libuvc.h>

#include <interface/ifdata.h>

namespace PI {
namespace interface {

/**
 * Interface to stream stereo data and IMU data from a PerceptInV1 device.
 * This interface only allows the device to be turned on and then off once.
 */
class PerceptInV1Interface {
 public:
  /**
   * @brief Constructor.
   * @param get_imu Flag to specify whether to get IMU data from the interface
   *                or not. True to get both IMU and stereo data. False to get
   *                only stereo data and drop all the IMU data. In both cases,
   *                IMU data is streamed from the device.
   * @param quite Flag to specify whether to print out messages about the state
   *              of the UVC driver.
   */
  PerceptInV1Interface(const bool get_imu = true,
                       const bool quite = false);
  virtual ~PerceptInV1Interface();

  /**
   * @brief Initialize UVC, find device, and then start streaming stereo images
   *        and IMU.
   * @param[out] err_msg Error message to diagnose why the device failed to
   *                     start streaming. (Optional)
   * @return True if successful. False otherwise.
   */
  bool StartStreaming(std::string *err_msg = nullptr);

  /**
   * @brief Stop streaming data, and exit UVC device.
   * @param[out] err_msg Error message to diagnose why the device failed to
   *                     start streaming. (Optional)
   * @return True if successful. False otherwise.
   */
  bool StopStreaming(std::string *err_msg = nullptr);

  /**
   * @brief Get exposure of the stereo camera.
   * @param[out] exposure Value of the exposure read from the driver.
   * @return True if successful. False otherwise.
   */
  bool GetExposure(uint32_t *exposure) const;

  /**
   * @brief Set exposure of the stereo camera.
   * @param exposure Value to set to.
   * @return True if successful. False otherwise.
   */
  bool SetExposure(const uint32_t exposure);

  /**
   * @brief Get the next data in the data queue.
   * @return Pointer to the next data. Nullptr if no data is available.
   */
  std::shared_ptr<Data> GetData();

  /**
   * @brief Get the newest data and drop all the older data from the queue.
   * @note This API will drop frame if the caller spends too much time consuming
   *       the data.
   * @return Pointer to the newest data. Nullptr if no data is available.
   */
  std::shared_ptr<Data> GetNewestData();

  // Pointer to implementation.
  // 2017.03.10 modify to public, bcz need use its function in robot/src/capture_dataframe.cc
  class Impl;
 private:
  std::shared_ptr<Impl> impl_;
};

}  // namespace interface
}  // namespace PI

#endif  // PERCEPTIN_V1_INTERFACE_H_
