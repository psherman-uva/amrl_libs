/*
  @file:      OccupancyGrid.hpp
  @author:    psherman
  @date       June 2023
  
  @brief Implementation of 2D probabilistic occupancy grid
*/

#pragma once

#include <amrl_libs/mapping/Grid.hpp>
#include <amrl_common/point/Point.hpp>

#include <boost/optional.hpp>

#include <array>
#include <set>
#include <vector>
#include <memory>

namespace amrl {

using GridCallback = std::function<void(uint32_t, uint32_t, double)>;

/// Class object to implement an bsaic 2D occupancy grid for use in
/// robotics application. The grid will maintain a grid of cells
/// with different probabilities of being "occupied" or "free".
class OccupancyGrid : public Grid
{
public:
  /// Constructor
  /// @param  origin
  /// @param  width
  /// @param  height
  /// @param  resolution
  /// @param  init_probability
  OccupancyGrid(const Point<double> &origin,
    const double width,
    const double height,
    const double resolution,
    const double init_probability = 0.5);

  /// Constructor
  /// @param  info
  /// @param  init_probability
  OccupancyGrid(const GridInfo &info,
    const double init_probability = 0.5);

  // Destructor
  ~OccupancyGrid(void) = default;

  double probability_index_occupied(const uint32_t idx) const;

  /// Get copy of set of all cells that are currently marked as occupied
  std::set<Point<uint32_t>> occupied_cells_get(void) const;

  /// Get copy of set of all grid indices that are currently marked as occupied
  std::set<uint32_t> occupied_indices_get(void) const;

  /// Reset which cells in the grid are set as occupied
  /// @param cells Set of cells to mark occupied
  void occupied_cells_set(const std::set<Point<uint32_t>> &cells);

  /// Manually update probability for cell with free and occupied cell measurements
  /// @param  free_cells Cells detected as free
  /// @param  occ_cells Cells detected as occupied
  /// @param  neighbor_cells
  void update_odds(
    const std::vector<Point<uint32_t>> &free_cells,
    const std::vector<Point<uint32_t>> &occ_cells, 
    const std::vector<Point<uint32_t>> &neighbor_cells = {});

  void index_set_value(const uint32_t idx, const double value) override;

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

private:

  void index_value_updated(const uint32_t idx, const Point<uint32_t> &p);


  /// Adjust the log odds of specified cells
  /// @param  cells Grid cells to adjust
  /// @param  adj_amount Amount to adjust log odds of each cell
  void adjust_cell_odds(const std::vector<Point<uint32_t>> &cells, const double adj_amount);

  // ---------------------------------- //
  // --       Member variables       -- //
  // ---------------------------------- //

  /// Set of all cells currently marked as "occupied".
  /// Dynamically updated as grid changes

  std::vector<double> _prob_occ;
  std::set<Point<uint32_t>> _occupied;
  std::set<uint32_t> _occupied_idx;

  static constexpr double kFreeProb     = 0.30; /// Probability sensor measures cell is free but is occuppied
  static constexpr double kOccupProb    = 0.70; /// Probability sensor measures cell is occuppied and is occuppied
  static constexpr double kNeighborProb = 0.55; /// Probability neighbor of cell measured occuppied is occuppied

  static constexpr double kOccupLogThreshold =  2.2; /// Log odds above which cell is considered "occupied"
  static constexpr double kFreeLogThreshold  = -1.5; /// Log odds below which cell is considered "free"

  const double kLogFree;     /// Log odds adjustment for measured free cell
  const double kLogOccup;    /// Log odds adjustment for mesaured occupied cell
  const double kLogNeighbor; /// Log odds adjustment for neighbor to a mesaured occupied cell

  double kLogOddsMin; /// Minimum allowed value for grid probability
  double kLogOddsMax; /// Maximum allowed value for grid probability

  /// Object has the feature to store functions to call anytime
  /// a cell's probability is updated
  std::vector<GridCallback> _callbacks;
};

} // namespace amrl