/*
  @file:      GridGraph.hpp
  @author:    psherman
  @date       June 2024
*/

#pragma once

#include <amrl_libs/mapping/OccupancyGrid.hpp>
#include <amrl_libs/path_planning/PathPlanNode.hpp>

#include <amrl_common/point/Point.hpp>

namespace amrl {

class GridGraph 
{
public:

  /// Constructor
  /// @param map Discrete map to reprsent as graph
  /// @param buffer Number of cells to buffer around obstacles
  GridGraph(
    std::shared_ptr<OccupancyGrid> map,
    uint32_t buffer = 0);

  /// Destructor
  ~GridGraph(void) = default;

  // ----------------------------------- //
  // --    Class Public Methods       -- //
  // ----------------------------------- //

  /// Get container all nodes in graph
  /// @return Vector of ALL nodes (even unconnected ones)
  std::vector<std::shared_ptr<PathPlanNode>> get_graph(void);

  /// Get an individual node
  /// @param idx Index of node
  /// @return Pointer to node of index
  std::shared_ptr<PathPlanNode> node(uint32_t idx);
  
  /// Gives graph and update from the occupancy map for the occupancy probability
  /// of a single cell. Allows graph to update nodes/edges to reflect the change.
  /// @param x Cell X position
  /// @param y Cell Y position
  /// @param probability Updated occupancy probability of cell
  void update_data(const uint32_t x, const uint32_t y, const double probability);
  
  /// Get index of node from the xy-cell
  /// @param cell XY cell of map
  /// @return Unique index id of cell
  uint32_t cell_to_index(const Point<uint32_t> &cell) const;

  /// Get xy-cell of a node from index
  /// @param idx Unique index id of cell
  /// @return XY cell of map
  Point<uint32_t> index_to_cell(uint32_t idx) const;

  /// Get the raw pointer to the underlying map
  std::shared_ptr<OccupancyGrid> get_map(void);

private:

  // ----------------------------------- //
  // --    Class Private Methods      -- //
  // ----------------------------------- //
  
  /// Construct all edges for node being re-added to graph
  /// and to corresponding neighbors
  /// @note Assumes edges is currently empty for node
  void construct_all_edges(const uint32_t idx);

  /// For specified node, remove all connected edges.
  /// @param idx Index of node to remove from graph
  void remove_edges(const uint32_t idx);

  /// When a node has been reconnected to graph.
  /// Need to reconstruct the edges to neighbors
  /// AND the edges from neighbors back to re-inserted node
  /// @param idx Index of node being reinserted
  void add_edges(const uint32_t idx);

  /// Procedure when a node has been marked as occupied
  /// Remove target node and neighbor cells within obstacle buffer zone
  /// @param idx Index of cell marked occupied
  void node_occupied_update(const uint32_t idx);

  /// Procedure when a node has been marked as free to add back into graph
  /// Add target node and neighbor cells no longer in obstacle buffer zone
  /// @param idx Index of cell marked free
  void node_free_update(const uint32_t idx);

  /// Get graph nodes that are certain number of steps away
  /// from a target node via currently connected edges
  /// @param neighborhood ID of all cells desired number of steps away from start
  /// @param idx Index of node to get neighbors of
  /// @param step Number of steps of neighbors to include
  void index_connected_neighborhood(std::set<uint32_t>& neighborhood, const uint32_t idx, const uint32_t step);

  /// Get graph nodes that are certain number of steps away
  /// via all POSSIBLE edges (even if not currently connected)
  /// @param neighborhood ID of all cells desired number of steps away from start
  /// @param idx Index of node to get neighbors of
  /// @param step Number of steps of neighbors to include
  void index_full_neighborhood(std::set<uint32_t>& neighborhood, const uint32_t idx, const uint32_t step);

  /// Get unique idx of a neighboring node
  /// @param idx Starting node
  /// @param j Neighbor label that specifies direction of neighbor
  /// @return Unique id of neighbor
  uint32_t get_neighbor(uint32_t idx, uint32_t j) const;

  /// Get a vector of all possible neighbor directions.
  /// Depends on if index is on a left, right, top, bottom or middle in the map
  /// @param idx Index of node to check
  std::vector<uint32_t> get_all_neighbor_directions(const uint32_t idx) const;

  /// Get the edge cost of a neighboring node depending on if
  /// direction is perpendicular or diagonal from node
  /// @param idx Id of starting node
  /// @param nbr Id of neighbor node
  /// @return Cost to travel from start node to neighbor
  double get_default_edge_cost(uint32_t idx, uint32_t nbr) const;

  /// Check if a node is on the bottom row of the map cells
  /// @param idx Index of node to check
  /// @return True if node along bottom
  bool bottom_row_check(const uint32_t idx) const;

  /// Check if a node is on the top row of the map cells
  /// @param idx Index of node to check
  /// @return True if node along top
  bool top_row_check(const uint32_t idx) const;

  /// Check if a node is on the left side of the map cells
  /// @param idx Index of node to check
  /// @return True if node along left side
  bool left_side_check(const uint32_t idx) const;

  /// Check if a node is on the right side of the map cells
  /// @param idx Index of node to check
  /// @return True if node along right side
  bool right_side_check(const uint32_t idx) const;

  /// Debugging method that will print each each node and it's
  /// current edges
  void print_graph(void);

  // ----------------------------------- //
  // --     Class Member Objects      -- //
  // ----------------------------------- //

  /// Occupancy Map represented by the graph
  std::shared_ptr<OccupancyGrid> _map;

  /// Size (width, height) in number of nodes of the map
  const std::pair<uint32_t, uint32_t> _grid_size;

  /// Number of nodes that make up one row of map
  const uint32_t _row_width;

  /// Total number of nodes that make up the graph
  const uint32_t _num_nodes;

  /// Number of free cells away from occupied space to
  /// remove from graph. Acts as a buffer for planning
  /// Buffer of zero will just ignore.
  const uint32_t _obs_buffer;

  /// All nodes that cover the map
  /// May not be fully connected (i.e. occupied cells are removed)
  std::vector<std::shared_ptr<PathPlanNode>> _graph;
  
  /// Indices of all cells currently marked as occupied
  std::set<uint32_t> _occupied_cells;

  // ----------------------------------- //
  // --       Class Constants         -- //
  // ----------------------------------- //
  //  Neighbor Nodes:
  //   -------------
  //   | 5 | 6 | 7 |
  //   -------------
  //   | 3 | - | 4 |
  //   -------------
  //   | 0 | 1 | 2 |
  //   -------------

  static const std::vector<uint32_t> kAllNeighbors;      // {0, 1, 2, 3, 4, 5, 6, 7}
  static const std::vector<uint32_t> kLeftNeighbors;     // {1, 2, 4, 6, 7}
  static const std::vector<uint32_t> kRightNeighbors;    // {0, 1, 3, 5, 6}
  static const std::vector<uint32_t> kTopNeighbors;      // {0, 1, 2, 3, 4}
  static const std::vector<uint32_t> kBottomNeighbors;   // {3, 4, 5, 6, 7}
  static const std::vector<uint32_t> kTopLeftNghbrs;     // {1, 2, 4}
  static const std::vector<uint32_t> kTopRightNghbrs;    // {0, 1, 3}
  static const std::vector<uint32_t> kBottomLeftNghbrs;  // {4, 6, 7}
  static const std::vector<uint32_t> kBottomRightNghbrs; // {3, 5, 6}
  
  static const std::vector<double> kNeighborCost; /// Cost of all edges
  static constexpr double kOccupProb = 0.70;      /// Probability cell is occuppied
};



} // namespace amrl