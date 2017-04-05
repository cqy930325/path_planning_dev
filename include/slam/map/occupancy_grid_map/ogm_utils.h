/** PerceptIn license **/

#ifndef SLAM_INCLUDE_SLAM_MAP_OCCUPANCY_GRID_MAP_OGM_UTILS_H_
#define SLAM_INCLUDE_SLAM_MAP_OCCUPANCY_GRID_MAP_OGM_UTILS_H_

#include <memory>

namespace PI {
namespace slam {
namespace ogm {

// Forward declaration.
class OccupancyGridMap;

/**
 * @brief Dilate the occupancy grid according to the size of the observer.
 * @note This is useful for path planning or other algorithms that treated the
 *       observer as a dot in the metric coordinate or a cell in the grid.
 * @param iterations Number of pixels to dilate.
 * @param[out] ogm_dilate_ptr Dilated occupancy grid.
 * @param value_occupied (optional) Value of a cell to be considered as
 *                       occupied.
 * @return True if the occupancy grid was dilated.
 */
bool DilateOgm(std::shared_ptr<OccupancyGridMap> ogm,
               const size_t iterations,
               const double value_occupied = 0.7);

} // namespace ogm
} // namespace slam
} // namespace PI

#endif  // SLAM_INCLUDE_SLAM_MAP_OCCUPANCY_GRID_MAP_OGM_UTILS_H_
