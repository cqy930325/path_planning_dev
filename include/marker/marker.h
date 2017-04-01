/** PerceptIn license **/

#ifndef MARKER_INCLUDE_MARKER_H_
#define MARKER_INCLUDE_MARKER_H_

#include <memory>

namespace PI {

// Forward declaration.
namespace config {
class Config;
}  // namespace config

namespace marker {

// Forward declaration.
class MarkerDetector;

/**
 * @brief Create a marker detector according to a config.
 */
std::shared_ptr<MarkerDetector> CreateMarkerDetector(
    std::shared_ptr<const config::Config> config);

}  // namespace marker
}  // namespace PI

#endif  // MARKER_INCLUDE_MARKER_H_
