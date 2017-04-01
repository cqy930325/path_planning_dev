/** PerceptIn license **/

#ifndef MARKER_INCLUDE_MARKER_CHESSBOARD_DETECTOR_H_
#define MARKER_INCLUDE_MARKER_CHESSBOARD_DETECTOR_H_

#include <map>
#include <memory>

#include <opencv2/core.hpp>

#include <marker/marker_detector.h>

namespace PI {
namespace marker {

/**
 * Detect a chessboard pattern.
 *
 * The chessboard pattern's coordinate is defined as x-axis right, y-axis back,
 * and z-axis down.
 */
class ChessboardDetector : public MarkerDetector {
 public:
  /**
   * Additional parameters for chessboard marker.
   */
  struct Parameters : public MarkerDetector::Parameters {
    /// Number of inner corners in the chessboard.
    cv::Size pattern_size;
    /// Physical size of a block on the chessboard. (Unit: meter).
    cv::Size2d block_size;
    /// Size of the sub-pixel refinement window size. Set to (0, 0) to turn off
    /// sub-pixel refinement.
    cv::Size subpixel_window_size;
    /// Sometimes chessboard detection can be really slow if the image dosn't
    /// have any chessboard marker. If this flag is true, the detector will
    /// speed up the search process but may result in more false negatives. If
    /// this flag is false, the detector will have almost no false negatives,
    /// but may be really slow when there's only partial or no chessboard in
    /// the image.
    bool speed;

    Parameters();
    ~Parameters();
  };

  explicit ChessboardDetector(const Parameters &params);
  virtual ~ChessboardDetector();

  /**
   * @brief A string specifier for the class.
   */
  static std::string class_type();
  /**
   * @copydoc ChessboardDetector::type()
   */
  std::string type() const override;

  /**
   * @copydoc MarkerDetector::DetectImpl().
   */
  bool DetectImpl(
      const cv::Mat &image,
      std::vector<Correspondence3d2d> *correspondences) const override;

  /**
   * @copydoc MarkerDetector::DrawImpl().
   */
  void DrawImpl(const std::vector<Correspondence3d2d> &correspondences,
                cv::Mat *image) const override;

 private:
  /// PIMPL
  class Impl;
  std::shared_ptr<const Impl> impl_;
};

}  // namespace marker
}  // namespace PI

#endif  // MARKER_INCLUDE_MARKER_CHESSBOARD_DETECTOR_H_
