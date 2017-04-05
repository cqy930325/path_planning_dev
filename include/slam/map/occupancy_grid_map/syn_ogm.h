/** PerceptIn license **/

#ifndef SLAM_INCLUDE_SLAM_MAP_OCCUPANCY_GRID_MAP_SYN_OGM_H_
#define SLAM_INCLUDE_SLAM_MAP_OCCUPANCY_GRID_MAP_SYN_OGM_H_

#include <math.h>
#include <memory>
#include <random>
#include <vector>

#include <opencv2/core.hpp>

namespace PI {
namespace slam {
namespace ogm {

// Forward declaration.
class OccupancyGridMap;

/**
 * @brief Create a random occupancy grid of random resolution and size.
 * @param randomizer Random number generator.
 * @return The generated occupancy grid. Nullptr if fail to create a random
 *         grid.
 */
std::shared_ptr<OccupancyGridMap> CreateRandomOgm(
    std::default_random_engine &randomizer);

/**
 * @brief Randomly fill in values for the occupancy grid with empty, occupied
 *        and unknown cells.
 * @param[in, out] ogm The occupancy grid to fill in.
 * @param randomizer Random number generator.
 * @return True if the occupancy grid is filled.
 */
bool CreateRandomOgm(std::shared_ptr<OccupancyGridMap> ogm,
                     std::default_random_engine &randomizer);

/**
 * @brief Randomly add an obstacle in the empty space of an occupancy grid map.
 * @param[in, out] ogm The occupancy grid to add obstacle to.
 * @param randomizer Random number generator.
 * @return True if an obstacle is added.
 */
bool AddRandomObstacle(std::shared_ptr<OccupancyGridMap> ogm,
                       std::default_random_engine &randomizer);

/**
 * @brief Fill in the occupancy grid with a room layout. The walls of the room
 *        are marked as occupied, and the room is marked as empty. The rest of
 *        the occupancy grid are unknown cells.
 *
 *        A basic room is a rectangle:
 *          -------------
 *         |             |
 *  height |             |
 *         |             |
 *          -------------
 *              width
 *
 *        There is an option to add an w-by-h indent wall in the room.
 *        (A room with indent wall sticking into the room.)
 *         -------------
 *        |      w      |
 *        |     ---     |
 *        |  h |   |    |
 *         ----|   | ----
 *        (A room with indent wall sticking out of the room.)
 *        -------------
 *        |             |
 *        |             |
 *        |             |
 *         ----     ----
 *            h|    |
 *             |----|
 *                w
 *
 * @param[in, out] ogm The occupancy grid to layout the room.
 * @param room_size The size of the room. (Unit: meter)
 * @param wall_thickness The thickness of the wall. (Unit: cell)
 * @param indent_wall_size The size of the indent wall. Set both dimensions to
 *                         negative values to get a basic room (no indent wall).
 *                         Both must be positive to specify the indent wall.
 *                         The size of the indent wall must be smaller than
 *                         the #room_size in order to fit.
 * @param indent_wall_in A flat to specify whether to stick the indent wall
 *                       in (true) or out (false) the room. This flag has no
 *                       effect when #indent_wall_size is negative.
 * @param room_center_offset An offset from the center of the occupancy grid.
 * @param orientation_degree Rotate the room with respect to the center of the
 *                           occupancy grid. (Hint: Rotating a multiply of 90
 *                           will place the indent wall in a different
 *                           direction.)
 * @return True if the occupancy grid is set to represent the room with the
 *         specified specs. False if the room cannot fit into the grid.
 */
bool InitializeToRoomOgm(
    std::shared_ptr<OccupancyGridMap> ogm,
    const cv::Size2d &room_size,
    const size_t wall_thickness = 2,
    const cv::Size2d &indent_wall_size = cv::Size2d(-1.0, -1.0),
    const bool indent_wall_in = false,
    const cv::Point2d &room_center_offset = cv::Point2d(0.0, 0.0),
    const double orientation_degree = 0.0);

/**
 * @brief Set all the cells in an occupancy grid to unknown.
 * @param[in, out] ogm The occupancy grid to set to unknown.
 */
void SetOgmToUnknown(std::shared_ptr<OccupancyGridMap> ogm);

/**
 * @brief Add an obstacle (a polygon) to an occupancy grid. The obstacle can
 *        only be added to empty spaces in the grid.
 * @param[in, out] ogm The occupancy grid to add the obstacle to.
 * @param obstacle_vertices The vertices (in original x, y coordinate in meter)
 *                          of the polygon.
 * @param border_thickness Thickness of the border of the obstacle. (Unit: cell)
 * @return True if the obstacle is added. False otherwise.
 */
bool AddObstacle(std::shared_ptr<OccupancyGridMap> ogm,
                 const std::vector<cv::Point2d> &obstacle_vertices,
                 const size_t border_thickness = 2);

} // namespace ogm
} // namespace slam
} // namespace PI

#endif  // SLAM_INCLUDE_SLAM_MAP_OCCUPANCY_GRID_MAP_SYN_OGM_H_
