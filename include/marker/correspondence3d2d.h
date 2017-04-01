/** PerceptIn license **/

#ifndef MARKER_INCLUDE_MARKER_CORRESPONDENCE_H_
#define MARKER_INCLUDE_MARKER_CORRESPONDENCE_H_

#include <opencv2/core.hpp>

namespace PI {
namespace marker {

/**
 * Correspondence between 2d detected point and 3d point on the marker.
 */
struct Correspondence3d2d {
  /// 3D point on the marker, represented in the marker's coordinate.
  cv::Point3d reference;
  /// Corresponding 2D point detected on the image.
  cv::Point2d observed;

  Correspondence3d2d();
  Correspondence3d2d(const cv::Point3d &reference, const cv::Point2d &observed);
  ~Correspondence3d2d();
};

}  // namespace marker
}  // namespace PI

#endif  // MARKER_INCLUDE_MARKER_CORRESPONDENCE_H_
