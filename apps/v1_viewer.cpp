/** PerceptIn license **/

#include <opencv2/highgui/highgui.hpp>
#include <interface/perceptin_v1_interface.h>

const std::string kWindowName1("PerceptIn V1 Stereo Left");
const std::string kWindowName2("PerceptIn V1 Stereo Right");
void ExposureTrackerBarCallback(int value, void *interface_ptr) {
  static_cast<PI::interface::PerceptInV1Interface*>(interface_ptr)->SetExposure(
      value);
  printf("Exposure: %d\n", value);
}
// A variable to keep track of the current exposure value on the slider bar.
int exposure_value;

int main(int argc, char *argv[]) {
  PI::interface::PerceptInV1Interface interface(true, false);

  // Start streaming.
  if (!interface.StartStreaming()) {
    return -1;
  }
  interface.SetExposure(250);

  // Create two windows with tracker bar to control exposure.
  cv::namedWindow(kWindowName1);
  cv::namedWindow(kWindowName2);
  {
    uint32_t exposure_value_32;
    if (interface.GetExposure(&exposure_value_32)) {
      exposure_value = static_cast<int>(exposure_value_32);
      cv::createTrackbar("Exposure", kWindowName1, &exposure_value, 800,
          ExposureTrackerBarCallback, &interface);
    }
  }

  // Let's print the total number of IMU and stereo streamed.
  size_t count_imu = 0;
  size_t count_stereo = 0;

  while (1) {
    std::shared_ptr<PI::interface::Data> data = interface.GetNewestData();
    if (!data) {
      continue;
    }

    // See if this an IMU data.
    std::shared_ptr<PI::interface::ImuData> imu_data =
        std::dynamic_pointer_cast<PI::interface::ImuData>(data);
    if (imu_data) {
      if (++count_imu % 200 == 0) {
        printf("Streamed %zd IMU data.\n", count_imu);
      }
      continue;
    }

    // This must be an stereo data.
    std::shared_ptr<PI::interface::StereoData> stereo_data =
        std::dynamic_pointer_cast<PI::interface::StereoData>(data);
    if (stereo_data) {
      if (++count_stereo % 30 == 0) {
        printf("Streamed %zd stereo data.\n", count_stereo);
      }
      cv::imshow(kWindowName1, stereo_data->image_left);
      cv::imshow(kWindowName2, stereo_data->image_right);

      char key = cv::waitKey(5);
      if (key == 27) {
        break;
      }
      continue;
    }

    // This is not an IMU data nor a stereo data. Who are you?
    printf("Something weird has happened.\n");
    break;
  }

  if (!interface.StopStreaming()) {
    return -1;
  }

  return 0;
}
