/*
  @file:      Grid.hpp
  @author:    psherman
  @date       June 2023
  
  @brief Implementation of 2D grid
*/

#pragma once

#include <amrl_libs/mapping/GridInfo.hpp>
#include <amrl_common/point/Point.hpp>
#include <amrl_libs/obstacle/Obstacle.hpp>

#include <array>
#include <set>
#include <vector>
#include <memory>

namespace amrl {

/// Class object to implement an bsaic 2D occupancy grid for use in
/// robotics application. The grid will maintain a grid of cells
/// with different probabilities of being "occupied" or "free".
class Grid
{
public:
  /// Constructor
  /// @param  origin Position of [0,0] cell (left-bottom) of grid
  /// @param  width Width of the grid along the x-direction
  /// @param  height Height of the grind along the y-direction
  /// @param  resolution 
  /// @param  init_value
  Grid(const Point<double> &origin,
    const double width,
    const double height,
    const double resolution,
    const double init_value);

  /// Constructor
  /// @param  info
  /// @param  init_value
  Grid(const GridInfo &info,
    const double init_value);

  // Destructor
  virtual ~Grid(void) = default;

  /// Get the number of cells in the grid
  /// @return Total number of cells
  uint32_t num_cells(void) const;

  /// Get the information that describes the grid configuration
  /// @return Grid info defining setup
  GridInfo info_get(void) const;

  /// Check if a world frame point is included in the grid
  /// @param  pt Point in the world frame
  /// @return True if pt is covered by the grid
  bool point_in_grid(const Point<double> &pt) const;

  /// Check if a grid cell is included in the grid
  /// @param  pt Point in the world frame
  /// @return True if pt is covered by the grid
  bool cell_in_grid(const Point<uint32_t> &pt) const;

  /// Get the size of the map
  /// @return Size of map. {distance x-direction, distance in y-direction}
  std::pair<double, double> size_get(void) const;

  /// Get the number of cells of the grid
  /// @return Size of map. {# cells in x-direction, # cells in y-direction}
  std::pair<uint32_t, uint32_t> grid_size_get(void) const;
 
  /// @return Get the resolution of the grid cells
  double resolution(void) const;

  /// @return Get the raw underlying grid data.
  const std::vector<double>& grid_get(void) const;

  /// @param grid Grid to replace raw grid data
  void grid_set(std::vector<double> grid);

  /// Convert a grid cell to corresponding point in the world frame
  /// @param pt Cell in grid to convert
  /// @return Point in the world frame
  Point<double> cell_to_point(const Point<uint32_t> &pt) const;

  /// Convert a point in the world frame to it's corresponding cell in the grid
  /// @param  pt Point in world frame
  /// @return Cell in grid corresponding to input
  Point<uint32_t> point_to_cell(const Point<double> &pt) const;

  /// Get all cells in the grid that are covered by polygon defined by obstacle
  /// @param  obs Obstacle in grid area
  /// @return All cells in grid that return as inside obstacle
  std::vector<Point<uint32_t>> cells_in_obstacle(const Obstacle &obs) const;

  // Limits of the map representation
  /// @return Limits [min_x, min_y, max_x, max_y]
  std::vector<double> map_limits(void) const;

  uint32_t cell_to_index(const Point<uint32_t> &cell) const;
  uint32_t point_to_index(const Point<double> &pt) const;

  Point<uint32_t> index_to_cell(const uint32_t idx) const;
  Point<double> index_to_point(const uint32_t idx) const;
  bool index_in_grid(const uint32_t idx) const;

  double value_from_index(const uint32_t idx) const;
  double value_from_cell(const Point<uint32_t> &cell) const;
  double value_from_point(const Point<double> &pt) const;

  virtual void index_set_value(const uint32_t idx, const double value);
  virtual void cell_set_value(const Point<uint32_t> &cell, const double value);
  virtual void point_set_value(const Point<double> &pt, const double value);

  void copy_grid_value(std::shared_ptr<Grid> rhs);

protected:

  // ---------------------------------- //
  // --       Member variables       -- //
  // ---------------------------------- //

  /// The main internal container representing the grid in row-major order
  std::vector<double> _grid;

  const double _resolution;          /// Size of each grid cell
  const double _half_resolution;     /// Half a cell size (used to offset world points to middle of cells)
  const Point<double> _origin;       /// Position (in world frame) of the [0,0] cell in grid
  const Point<double> _size;         /// Length & width of the grid
  Point<double> _max_pt;             /// Position (in world frame) of maximum x and y in grid

  const uint32_t _row_width;        /// Number of cells in x-row (convenient for use in row-major order grid)
  const uint32_t _col_height;       /// Number of cells in y-column
  const uint32_t _num_cells;        /// Total number of cells in grid;
  
  /// Size of the grid in number of cells
  const std::pair<uint32_t, uint32_t> _grid_size;
};

} // namespace amrl