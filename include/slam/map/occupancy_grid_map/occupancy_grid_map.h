/** PerceptIn license **/

#ifndef SLAM_INCLUDE_SLAM_MAP_OCCUPANCY_GRID_MAP_OCCUPANCY_GRID_MAP_H_
#define SLAM_INCLUDE_SLAM_MAP_OCCUPANCY_GRID_MAP_OCCUPANCY_GRID_MAP_H_

#include <vector>

#include <opencv2/core.hpp>

#include <slam/map/map.h>

namespace PI {
namespace slam {
namespace ogm {

/**
 * Index of a cell in the occupancy grid.
 */
struct CellIndex {
  /// Horizontal index.
  size_t x_idx;
  /// Vertical index.
  size_t y_idx;

  CellIndex();
  CellIndex(const size_t x_idx, const size_t y_idx);
};

/**
 * @brief Add an offset to a cell index.
 * @note idx_out = idx_1 + indx_2.
 */
CellIndex operator+(const CellIndex &a, const CellIndex &b);

/**
 * @brief Define operator for comparing two cell index.
 */
bool operator==(const CellIndex& a, const CellIndex& b);

/**
 * @brief Define operator for comparing two cell index.
 */
bool operator!=(const CellIndex& a, const CellIndex& b);

/**
 * @brief Define operator to support std::map or std::set.
 */
bool operator<(const CellIndex &a, const CellIndex &b);

/**
 * Data structure for an occupancy grid.
 *
 * The occupancy grid covers a square of a 2d metric space, where the center is
 * at (0, 0) of the metric space. Each cell in the grid is also a square. The
 * grid has the same number of cells in both horizontal and vertical directions.
 * The x-axis of the metric space is pointing to the right, and the y-axis of
 * the metric space is pointing down.
 */
// TODO(gtsai): Figure out an elegant way to resize the map.
class OccupancyGridMap : public Map {
 public:
  /**
   * @brief Constructor.
   * @param cell_size Physical size of a single cell in the occupancy grid..
   * @param grid_size Number of cells per side in the occupancy grid.
   */
  OccupancyGridMap(const double cell_size,
                   const cv::Size &grid_size);
  /**
   * @brief Copy constructor.
   */
  OccupancyGridMap(const OccupancyGridMap &ogm);
  virtual ~OccupancyGridMap();

  /**
   * An identifier specifying the type of the map.
   */
  static std::string class_type();

  /**
   * @copydoc Map::type()
   */
  std::string type() const override;

  /**
   * @copydoc Map::IsEmpty()
   * @note Map is empty when there's no cells in the grid.
   */
  // TODO(gtsai, yren, zzhang): Is there a better way to define empty ogm? Maybe
  // we check if all the cells are 0.5?
  bool IsEmpty() const override;

  /** Getters **/
  const std::vector<double>& cells() const;
  const double cell_size() const;
  const cv::Size &grid_size() const;
  const cv::Size2d map_size() const;
  const CellIndex idx_max() const;  /// Maximum index in both dimensions.

  /** Setters **/
  // Avoid resizing the cells in using this setter.
  std::vector<double> &cells();

  /**
   * @return The area that the occupancy grid covers.
   */
  double GetArea() const;

  /*
   * @return The ROI that the occupancy grid covered.
   */
  cv::Rect2d GetRoi() const;

  /**
   * @brief Retrieve the value of a cell in the grid from its 2-d index.
   * @param x_idx Index of the cell along the horizontal direction.
   * @param y_idx Index of the cell along the vertical direction.
   * @param[out] The value at cell (x_idx, y_idx).
   * @return True if the value is retrieved. Negative value if the index is
   *         outside of the grid.
   */
  bool GetValue(const size_t &x_idx,
                const size_t &y_idx,
                double *value) const;

  /**
   * @brief Retrieve the grid's value from its 2-d index.
   * @param idx Index of the cell.
   * @param[out] The value at cell (x_idx, y_idx).
   * @return True if the value is retrieved. Negative value if the index is
   *         outside of the grid.
   */
  bool GetValue(const CellIndex &idx,
                double *value) const;

  /**
   * @brief Set the value of a cell.
   * @param x_idx Horizontal index of the cell to edit.
   * @param y_idx Vertical index of the cell to edit.
   * @param val Value to set to.
   * @return True if (#x_idx, #y_idx) is a cell in the grid.
   */
  bool SetValue(const size_t x_idx,
                const size_t y_idx,
                const double val);
  /**
   * @brief Set the value of a cell.
   * @param idx Index of the cell to edit.
   * @param val Value to set to.
   * @return True if #idx is a cell in the grid.
   */
  bool SetValue(const CellIndex &idx,
                const double val);

  /**
   * @brief Get the metric value (x, y) of the center of the cell.
   */
  bool GetLocation(const size_t x_idx,
                   const size_t y_idx,
                   cv::Point2d *loc) const;

  /**
   * @brief Get the metric value (x, y) of the center of the cell.
   */
  bool GetLocation(const CellIndex &idx,
                   cv::Point2d *loc) const;

  /**
   * @brief Calculate which cell a point (x, y) in the metric coordinate belongs
   *        to in the occupancy grid.
   * @param loc_x X location of the point in the metric coordinate.
   * @param loc_y Y location of the point in the metric coordinate.
   * @param[out] x_idx Horizontal index of the cell.
   * @param[out] y_idx Vertical index of the cell.
   * @return True if the point (x, y) is within the grid's mapped area.
   */
  bool FindCell(const double loc_x,
                const double loc_y,
                size_t *x_idx,
                size_t *y_idx) const;
  /**
   * @brief Calculate which cell a point (x, y) in the metric coordinate belongs
   *        to in the occupancy grid.
   * @param loc_x X location of the point in the metric coordinate.
   * @param loc_y Y location of the point in the metric coordinate.
   * @param[out] idx Index of the cell.
   * @return True if the point (x, y) is within the grid's mapped area.
   */
  bool FindCell(const double loc_x,
                const double loc_y,
                CellIndex *idx) const;
  /**
   * @brief Calculate which cell a point (x, y) in the metric coordinate belongs
   *        to in the occupancy grid.
   * @param loc Location of the point in the metric coordinate.
   * @param[out] idx Index of the cell.
   * @return True if the point (x, y) is within the grid's mapped area.
   */
  bool FindCell(const cv::Point2d &loc,
                CellIndex *idx) const;

  /**
   * @brief Visualize the occupancy grid as a cv::Mat image. Each pixel in the
   *        image represents a cell. A cell with value 1.0 is marked as white,
   *        and 0.0 is marked as black.
   * @return The image showing the values in the grid.
   */
  cv::Mat Visualize() const;

 protected:
  /// Note, setting #res_, #sq_size_, and #idx_max_ constant until we figure out
  /// how to properly resize an occupancy grid.
  /// Physical size of each cell in the occupancy grid map.
  const double cell_size_;
  /// Maximum cell index along each dimension.
  const cv::Size grid_size_;
  /// The occupancy grid map. The value is in the range of [0.0, 1.0]. Negative
  /// value means the cell is invalid.
  std::vector<double> cells_;
};

} // namespace ogm
} // namespace slam
} // namespace PI

#endif  // SLAM_INCLUDE_SLAM_MAP_OCCUPANCY_GRID_MAP_OCCUPANCY_GRID_MAP_H_
