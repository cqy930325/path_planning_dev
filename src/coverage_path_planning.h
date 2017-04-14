// copyright[2017] Perceptin

#include <iostream>
#include <queue>
#include <map>
#include <set>
#include <memory>
#include <vector>
#include <algorithm>
#include <glog/logging.h>
#include <slam/map/occupancy_grid_map/occupancy_grid_map.h>
#include <slam/map/occupancy_grid_map/syn_ogm.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace PI {
namespace slam {
namespace ogm {

/**
 * @brief Upscale the cell size of occupancy grid map but keep map size
 * @param ogm Pointer of input occupancy grid map
 * @param Factor scale factor
 * @return Pointer of up-scaled occupancy grid map
 **/
std::shared_ptr<OccupancyGridMap>
UpScale(std::shared_ptr<OccupancyGridMap> ogm, const int factor);

/**
 * @brief Binarize the occupancy grid map to 0(free) and 1(occupied)
 * @param ogm Pointer of input occupancy grid map
 * @param threshold Threshold for binarize, value less than threshold
 * will be count as 0
 * @return True if the threshold is in range [0,1]
 */
bool Binarize(std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm,
              double threshold);

/**
 * @brief Generate a pointer of occupancy grid map with cv::Mat
 * @param m Input cv:Mat style Map
 * @param cell_size Physical size of a single cell in the occupancy grid
 * @return A pointer of occupancy grid map
 */
std::shared_ptr<PI::slam::ogm::OccupancyGridMap> GenerateMap(cv::Mat m);

/**
 * @brief Data structure to implement Dijkstra's Algorithm, which is an
 * extension of each cell with their distance and shortest path from this
 * point to goal
 * @param idx Cell index
 * @param distance Distance from goal to current cell
 * @param val Value from OGM: 0(free) 1(occupied)
 * @param prev Previous node from goal to current cell
 */
typedef struct node {
  PI::slam::ogm::CellIndex idx;
  u_int distance;
  double val;
  node *prev;
} Node;

/**
 * @brief Comparator for min priority queue
 */
struct cmp {
  bool operator()(Node *n1, Node *n2) {
    return n1->distance > n2->distance;
  }
};

/**
 * @brief Find the neighboring cell for certain cell
 * @param nodes Cells available in the map
 * @param node  Target cell to search for
 * @return Vector of neighboring nodes
 */
std::vector<Node *> GetNeighbors(std::map<CellIndex, Node *> nodes,
                                 Node *node);

/**
 * @brief Find the neighboring cell for certain cell
 * @param nodes Cells available in the map
 * @param grid_size Boundary of the map
 * @param node Target cell to search for
 * @return Vector of neighboring nodes
 */
std::vector<Node *> GetNeighbors(std::vector<Node *> nodes,
                                 cv::Size grid_size, Node *node);
std::map<CellIndex, Node *>

/**
 * @brief Construct a distance tranfrom for the OGM map, all the reachable
 * node in the OGM will be store in the hashmap according to their cellidx,
 * @param ogm Target map to be transformed
 * @param goal Target physical goal point on the OGM
 * @return Hashmap contain all the accessible point on the OGM and store their
 * shortest distance
 */
DistanceTransform(std::shared_ptr<OccupancyGridMap> ogm, cv::Point2d goal);

/**
 * Generate a path which can iterate all the accessible point on the map using
 * potential field method
 * @param ogm Pointer of input occupancy grid map
 * @param start Start point on the map
 * @param goal Target point on the map
 * @param path List of point need to visit if return true
 * @return True find a path which can cover all the accessible point on the map
 */
bool GeneratePath(std::shared_ptr<OccupancyGridMap> ogm,
                  cv::Point2d start, cv::Point2d goal,
                  std::vector<CellIndex> *path);

/**
 * Visualize the coverage path planning cell by cell
 * @param ogm Pointer of input occupancy grid map
 * @param start Start point on the map
 * @param goal Goal point on the map
 * @param scale Scale to expand or shrind the map show on the window
 * @param delay Interval time for each step in ms
 * @return False if can not do the path planning due to invalid start
 * or goal point
 */
bool StepVisualize(std::shared_ptr<OccupancyGridMap> ogm,
                   cv::Point2d start, cv::Point2d goal, int scale, int delay);
};  // namespace ogm
};  // namespace slam
};  // namespace PI

