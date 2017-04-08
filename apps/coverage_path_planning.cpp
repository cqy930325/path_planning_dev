// copyright[2017] Perceptin
#include "coverage_path_planning.h"

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
UpScale(std::shared_ptr<OccupancyGridMap> ogm, int factor) {
  CHECK_EQ(ogm->grid_size().width % factor, 0)
    << "width should be divided by factor exactly!";
  CHECK_EQ(ogm->grid_size().height % factor, 0)
    << "height should be divided by factor exactly!";
  // TODO(qchen) :support regular division besides exact division.
  cv::Size grid_size(ogm->grid_size().width / factor
    , ogm->grid_size().height / factor);
  std::shared_ptr<PI::slam::ogm::OccupancyGridMap> scale_ogm(
    new OccupancyGridMap(ogm->cell_size() * factor, grid_size));
  double *tmp_val = new double;
  for (size_t i = 0; i < scale_ogm->grid_size().width; ++i) {
    for (size_t j = 0; j < scale_ogm->grid_size().height; ++j) {
      double val = 1;
      for (size_t k = i * factor; k < i * factor + factor; k++) {
        for (size_t l = j * factor; l < j * factor + factor; l++) {
          ogm->GetValue(k, l, tmp_val);
          val *= *tmp_val;
        }
      }
      val = val == 0 ? 0 : 1;
      scale_ogm->SetValue(i, j, val);
    }
  }
  return scale_ogm;
}

/**
 * @brief Binarize the occupancy grid map to 0(free) and 1(occupied)
 * @param ogm Pointer of input occupancy grid map
 * @param threshold Threshold for binarize, value less than threshold
 * will be count as 0
 * @return True if the threshold is in range [0,1]
 */
bool Binarize(std::shared_ptr<PI::slam::ogm::OccupancyGridMap> ogm,
              double threshold) {
  if (threshold >= 0 && threshold <= 1) return false;
  for (size_t i = 0; i < ogm->grid_size().width; ++i) {
    for (size_t j = 0; j < ogm->grid_size().height; ++j) {
      double val;
      ogm->GetValue(i, j, &val);
      val = val < threshold ? 0 : 1;
      ogm->SetValue(i, j, val);
    }
  }
  return true;
}

/**
 * @brief Generate a pointer of occupancy grid map with cv::Mat
 * @param m Input cv:Mat style Map
 * @param cell_size Physical size of a single cell in the occupancy grid
 * @return A pointer of occupancy grid map
 */
std::shared_ptr<PI::slam::ogm::OccupancyGridMap> GenerateMap(cv::Mat m, int cell_size) {
  std::shared_ptr<PI::slam::ogm::OccupancyGridMap> map(
    new PI::slam::ogm::OccupancyGridMap(cell_size, cv::Size(m.rows, m.cols)));
  for (int i = 0; i < m.cols; ++i) {
    for (int j = 0; j < m.rows; ++j) {
      map->SetValue(static_cast<size_t>(i),
                    static_cast<size_t>(j), m.at<int>(i, j));
    }
  }
  return map;
}

/**
 * @brief Find the neighboring cell for certain cell
 * @param nodes Cells available in the map
 * @param node  Target cell to search for
 * @return Vector of neighboring nodes
 */
std::vector<Node *> GetNeighbors(std::map<CellIndex, Node *> nodes,
                                 Node *node) {
  std::vector<Node *> neighbors = {};
  PI::slam::ogm::CellIndex idx = node->idx;
  idx.x_idx += 1;  // Right neighbor cell
  // Check neighbor cell is accessible
  if (nodes.find(idx) != nodes.end()) {
      neighbors.push_back(nodes[idx]);
  }
  idx = node->idx;
  idx.y_idx += 1;  // Top neighbor cell
  if (nodes.find(idx) != nodes.end()) {
      neighbors.push_back(nodes[idx]);
  }
  idx = node->idx;  // Bottom neighbor cell
  idx.y_idx -= 1;
  if (nodes.find(idx) != nodes.end()) {
      neighbors.push_back(nodes[idx]);
  }
  idx = node->idx;
  idx.x_idx -= 1;  // Left neighbor cell
  if (nodes.find(idx) != nodes.end()) {
      neighbors.push_back(nodes[idx]);
  }
  return neighbors;
}

/**
 * @brief Find the neighboring cell for certain cell
 * @param nodes Cells available in the map
 * @param grid_size Boundary of the map
 * @param node Target cell to search for
 * @return Vector of neighboring nodes
 */
std::vector<Node *> GetNeighbors(std::vector<Node *> nodes,
                                 cv::Size grid_size, Node *node) {
  std::vector<Node *> neighbors = {};
  int x = static_cast<int>(node->idx.x_idx);
  int y = static_cast<int>(node->idx.y_idx);
  if (x + 1 < grid_size.width) {   // Check Left Neighbor in boundary
    Node *n = nodes[y * grid_size.width + x + 1];
    if (n->val != 1)  // Check cell is free
      neighbors.push_back(n);
  }
  if (y + 1 < grid_size.height) {  // Check Top Neighbor in boundary
    Node *n = nodes[(y + 1) * grid_size.width + x];
    if (n->val != 1)
      neighbors.push_back(n);
  }
  if (x - 1 >= 0) {                // Check Right Neighbor in boundary
    Node *n = nodes[y * grid_size.width + x - 1];
    if (n->val != 1)
      neighbors.push_back(n);
  }
  if (y - 1 >= 0) {                // Check Bottom Neighbor in boundary
    Node *n = nodes[(y - 1) * grid_size.width + x];
    if (n->val != 1)
      neighbors.push_back(n);
  }
  return neighbors;
}

/**
 * @brief Construct a distance tranform for the OGM map, all the reachable
 * node in the OGM will be store in the hashmap according to their cellidx,
 * @param ogm Target map to be transformed
 * @param goal Target physical goal point on the OGM
 * @return Hashmap contain all the accessible point on the OGM and store their
 * shortest distance
 */
std::map<CellIndex, Node *>
DistanceTransform(std::shared_ptr<OccupancyGridMap> ogm, cv::Point2d goal) {
  cv::Size size = ogm->grid_size();
  std::priority_queue<Node *, std::vector<Node *>, cmp> unvisited;
  // Min heap to store all the unvisited nodes
  std::vector<Node *> nodes;
  PI::slam::ogm::CellIndex *goal_idx = new PI::slam::ogm::CellIndex;
  std::set<Node *> visited;  // set to record all the visited nodes
  std::set<Node *> unvisited_set;  //set to record all the unvisited nodes
  // ToDO(qchen) : Search in PQ or maintain a new set?
  // c++ STL does not support find in PQ
  std::map<PI::slam::ogm::CellIndex, Node *> map;
  if (!ogm->FindCell(goal, goal_idx)) {
    std::cerr << "invalid location" << std::endl;  // Goal is not accessible
  }
  for (size_t j = 0; j < size.height; ++j) {
    for (size_t i = 0; i < size.width; ++i) {
      //iterate OGM find all the accessible node, add into unvisited
      PI::slam::ogm::CellIndex tmp(i, j);
      double tmp_val = 0;
      bool got = ogm->GetValue(tmp, &tmp_val);
      CHECK(got) << "invalid index access!" << std::endl;
      Node *node = new Node;
      /*
       * for each unvisited node:
       *  set distance to inf
       *  set prev to NULL
       */
      if (tmp.x_idx == goal_idx->x_idx && tmp.y_idx == goal_idx->y_idx) {
        node->distance = 0;  // set the goal distance to 0
        unvisited.push(node);
        unvisited_set.insert(node);
      } else {
        node->distance = UINT_MAX;
      }
      node->idx = tmp;
      node->val = tmp_val;
      node->prev = NULL;
      nodes.push_back(node);
    }
  }
  // Dijkstra's algorithm
  // see https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
  while (!unvisited.empty()) {
    Node *closest = unvisited.top();
    unvisited.pop();
    visited.insert(closest);
    std::vector<Node *> neighbors = GetNeighbors(nodes,
                                                 ogm->grid_size(), closest);
    for (auto neighbor : neighbors) {
      if (visited.find(neighbor) != visited.end())
        continue;
      u_int new_distance = closest->distance + 1;
      if (unvisited_set.find(neighbor) == unvisited_set.end()) {
        unvisited_set.insert(neighbor);
        unvisited.push(neighbor);
      } else {
        if (new_distance > neighbor->distance) continue;
      }
      neighbor->distance = new_distance;
      neighbor->prev = closest;
    }
  }
  // Add all the accessible node in to hash map
  for (auto node : nodes) {
    if (visited.find(node) != visited.end())
      map[node->idx] = node;

  }
  return map;
}
/**
 * @brief Generate a path which can iterate all the accessible point on the map using
 * waterfront method see: "Planning Paths of Complete Coverage of an Unstructured
 * Environment by a Mobile Robot"
 * @param ogm Pointer of input occupancy grid map
 * @param start Start point on the map
 * @param goal Target point on the map
 * @param path List of point need to visit if return true
 * @return True find a path which can cover all the accessible point on the map
 */
bool GeneratePath(std::shared_ptr<OccupancyGridMap> ogm,
                  cv::Point2d start, cv::Point2d goal,
                  std::vector<CellIndex> *path) {
  PI::slam::ogm::CellIndex goal_idx;
  if (!ogm->FindCell(start, &goal_idx))
    return false;
  PI::slam::ogm::CellIndex start_idx;
  if (!ogm->FindCell(start, &start_idx))
    return false;
  double value;
  // Check start_idx and goal_idx is in the map and free
  if (ogm->GetValue(start_idx, &value)) {  // in map
    if (value == 1) {  // free
      return false;
    }
  } else {
    return false;
  }
  if (ogm->GetValue(goal_idx, &value)) {
    if (value == 1) {
      return false;
    }
  } else {
    return false;
  }
  std::map<CellIndex, Node *> transform_map = DistanceTransform(ogm, goal);
  std::set<Node *> unvisited;
  for (auto it = transform_map.begin(); it != transform_map.end(); ++it)
    unvisited.insert(it->second);
  if (unvisited.size() <= 1) {
    std::cerr << "invalid start point node# = 0" << std::endl;
    return false;
  }
  Node *node = transform_map[start_idx];  // Set Start Cell to Current Cell
  bool first = true;
  while (!unvisited.empty()) {
    unvisited.erase(node);
    std::vector<Node *> neighbors = GetNeighbors(transform_map, node);
    std::sort(neighbors.begin(), neighbors.end(),
              [](Node *n1, Node *n2) { return n1->distance > n2->distance; });
    Node *next = NULL;
    // Find unvisited Neighbouring cell with highest distance
    for (auto neighbor : neighbors) {
      if (unvisited.find(neighbor) != unvisited.end()) {
        next = neighbor;
        break;
      }
    }
    if (next == NULL) {  //if not found, go back to node has neighbors
      if (first)
        path->push_back(node->idx);
      first = false;
      node = node->prev;
    } else {  // if find, set this cell to Current Cell
      first = true;
      next->prev = node;
      path->push_back(node->idx);
      node = next;
    }
  }
  return true;
}

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
                   cv::Point2d start, cv::Point2d goal, int scale, int delay) {
  std::vector<CellIndex> *path = new std::vector<CellIndex>;
  if (!GeneratePath(ogm, start, goal, path)) return false;
  std::cout << path->size() << std::endl;

  for (auto idx : *path) {
    ogm->SetValue(idx, 0.2);  // Mark current visited cell on map
    cv::Mat tmp;
    cv::resize(ogm->Visualize(), tmp, cv::Size(), scale, scale);
    // Resize image for better visualization
    cv::imshow("VIZ", tmp);
    cv::waitKey(delay);  // Delay certain time for view
    ogm->SetValue(idx, 0.5);  // Set explored cell to gray
  }

  return true;
}
};  // namespace ogm
};  // namespace slam
};  // namespace PI


