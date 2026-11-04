/*
  @file:      OccupancyGridSimple.hpp
  @author:    psherman
  @date       June 2023
  
  @brief Implementation of 2D probabilistic occupancy grid
*/

#pragma once

#include <amrl_libs/mapping/GridInfo.hpp>
#include <amrl_libs/mapping/Grid.hpp>
#include <amrl_common/point/Point.hpp>

#include <boost/optional.hpp>

#include <array>
#include <set>
#include <vector>
#include <memory>

namespace amrl {

/// Class object to implement an bsaic 2D occupancy grid for use in
/// robotics application. The grid will maintain a grid of cells
/// with different probabilities of being "occupied" or "free".
class OccupancyGridSimple : public Grid
{
public:
  /// Constructor
  /// @param  origin
  /// @param  width
  /// @param  height
  /// @param  resolution
  OccupancyGridSimple(const Point<double> &origin,
    const double width,
    const double height,
    const double resolution);

  /// Constructor
  /// @param  info
  /// @param  init_probability
  OccupancyGridSimple(const GridInfo &info);

  // Destructor
  ~OccupancyGridSimple(void) = default;

  /// Reset the all cells back to the initial probability
  void reset(void);

  /// Get copy of set of all cells that are currently marked as occupied
  std::set<Point<uint32_t>> occupied_cells_get(void) const;

  /// Get copy of set of all grid indices that are currently marked as occupied
  std::set<uint32_t> occupied_indices_get(void) const;

  /// Manually mark cells as free
  void cells_set_free(const std::vector<Point<uint32_t>> &cells);

  /// Manually mark cells as occupied
  void cells_set_occupied(const std::vector<Point<uint32_t>> &cells);

  /// When updating a cell probability value, object will
  /// call any registered callback functions with the change
  /// @param func Function to register as callback
  void register_callback(GridCallback func);

  /// Check if a specific index is an occupied cell
  /// @param idx Index to check
  /// @return True if index probability is above occupied threshold
  bool index_is_occupied(const uint32_t idx) const;

  /// Check if a specific index is a free cell
  /// @param idx Index to check
  /// @return True if index probability is below free space threshold
  bool index_is_free(const uint32_t idx) const;

  /// Check if a specific cell is occupied
  /// @param cell Cell to check
  /// @return True if cell probability is above occupied threshold
  bool cell_is_occupied(const Point<uint32_t> &cell) const;

  /// Check if a specific cell is free
  /// @param cell Cell to check
  /// @return True if cell probability is below free space threshold
  bool cell_is_free(const Point<uint32_t> &cell) const;

  /// Check if a specific point is in an occupied cell
  /// @param pt Point to check
  /// @return True if point probability is above occupied threshold
  bool point_is_occupied(const Point<double> &pt) const;

  /// Check if a specific point is in a free cell
  /// @param pt Point to check
  /// @return True if point probability is below free space threshold
  bool point_is_free(const Point<double> &pt) const;

  /// Add solid obstacle to the the occupied cells.
  void add_solid_obstacle(const Obstacle &obs);

private:

  void index_set_value(const uint32_t idx, const double value) override;
  void index_value_updated(const uint32_t idx, const Point<uint32_t> &p);

  /// Adjust the log odds of specified cells
  /// @param  cells Grid cells to adjust
  /// @param  adj_amount Amount to adjust log odds of each cell
  void cells_set_values(const std::vector<Point<uint32_t>> &cells, const double value);

  // ---------------------------------- //
  // --       Member variables       -- //
  // ---------------------------------- //

  /// Set of all cells currently marked as "occupied".
  /// Dynamically updated as grid changes
  std::set<Point<uint32_t>> _occupied;
  std::set<uint32_t> _occupied_idx;

  static constexpr double kFree    = 0.0; /// Value for free cell
  static constexpr double kUnknown = 0.5; /// Value for unknown cell
  static constexpr double kOccup   = 1.0; /// Value for occupied cell

  static constexpr double kFreeThreshold = 0.1;
  static constexpr double kOccThreshold  = 0.9;

  /// Object has the feature to store functions to call anytime
  /// a cell's value is updated
  std::vector<GridCallback> _callbacks;
};

} // namespace amrl