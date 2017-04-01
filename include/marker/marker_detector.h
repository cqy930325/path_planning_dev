/** PerceptIn license **/

#ifndef MARKER_INCLUDE_MARKER_MARKER_DETECTOR_H_
#define MARKER_INCLUDE_MARKER_MARKER_DETECTOR_H_

#include <map>
#include <memory>

#include <opencv2/core.hpp>

#include <marker/correspondence3d2d.h>

namespace PI {

namespace marker {

/**
 * A detector that finds a specific marker on images.
 */
class MarkerDetector {
 public:
  /**
   * Parameters for the marker or detection.
   */
  struct Parameters {
    Parameters();
    virtual ~Parameters();
  };

  explicit MarkerDetector(const Parameters &params);
  virtual ~MarkerDetector();

  /**
   * @brief A string defining the type of the detector.
   * @note Must also implement a static function std::string class_type().
   */
  virtual std::string type() const = 0;

  /**
   * @brief Detect the marker from an image to establish 2d-3d correspondences.
   * @param image Image to detect the marker from.
   * @param[out] correspondences A vector of correspondences (a pair of 3D
   *                             reference location and 2D observed location).
   * @return True if points are detected.
   */
  bool Detect(const cv::Mat &image,
              std::vector<Correspondence3d2d> *correspondences) const;

  /**
   * @brief Draw the 2d-3d correspondences on an image.
   * @param correspondences A vector of correspondences (a pair of 3D reference
   *                        location and 2D observed location).
   * @param[in,out] Image to draw on.
   */
  void Draw(const std::vector<Correspondence3d2d> &correspondences,
            cv::Mat *image) const;

 private:
  /**
   * @brief Implementation of Detect() to be implemented by the derived class.
   * @param image Image to detect the marker from.
   * @param[out] correspondences A vector of correspondences (a pair of 3D
   *                             reference location and 2D observed location).
   * @return True if points are detected.
   * @note Derived class can safely assume that \p image is not empty and that
   *       \p correspondences is not nullptr.
   */
  virtual bool DetectImpl(
      const cv::Mat &image,
      std::vector<Correspondence3d2d> *correspondences) const = 0;

  /**
   * @brief Implementation of Draw() to be implemented by the derived class.
   * @param correspondences A vector of correspondences (a pair of 3D reference
   *                        location and 2D observed location).
   * @param[in,out] Image to draw on.
   * @note Derived class can safely assume that \p image is not nullptr and is
   *       not empty.
   */
  virtual void DrawImpl(
      const std::vector<Correspondence3d2d> &correspondences,
      cv::Mat *image) const = 0;
};

}  // namespace marker
}  // namespace PI

#endif  // MARKER_INCLUDE_MARKER_MARKER_DETECTOR_H_
