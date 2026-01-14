/*
  @file:      AStar.hpp
  @author:    psherman
  @date       May 2024
  
  @brief Implementation A-Star algorithm for path planning
*/

#pragma once

#include <amrl_common/point/Point.hpp>
#include <amrl_libs/mapping/GridGraph.hpp>

#include <map>
#include <queue>

namespace amrl {

class AStar 
{
protected:
  struct Node {
    uint32_t idx;
    float cost;

    Node(const uint32_t i, const float cst) : idx(i), cost(cst) {}
    Node(void) : idx(0), cost(0.0) {}

    bool operator<(const Node& n2) const { return this->cost < n2.cost; }
    bool operator>(const Node& n2) const { return this->cost > n2.cost; }
  };

public:
  /// Constructor
  /// @param map Map object for planning path through
  /// @param func Heuristic function used for cost estimation
  AStar(std::shared_ptr<GridGraph> grid_graph,
    std::function<float(const Point<uint32_t>&, const Point<uint32_t>&)> func);

  /// Default destructor
  virtual ~AStar(void) = default;

  /// Plan a path through map
  /// @param start Starting point for path
  /// @param goal Desired goal position for path
  /// @return Ordered set of connected points from start to goal
  virtual std::vector<Point<uint32_t>> path_plan(const Point<uint32_t>& start, const Point<uint32_t>& goal);

protected:

  void initialize(const Point<uint32_t>& start, const Point<uint32_t>& goal);

  std::vector<Point<uint32_t>> construct_path(
    const uint32_t goal_idx,
    const uint32_t start_idx) const;

  std::shared_ptr<GridGraph> _grid_graph;
  std::function<float(const Point<uint32_t>&, const Point<uint32_t>&)> _H_func;

  std::vector<bool> _closed_pts;
  std::vector<float> _g_scores;
  std::map<uint32_t, uint32_t> _parents;

  uint32_t _goal_idx;
  uint32_t _start_idx;

  std::shared_ptr<PathPlanNode> _start_node;

  std::priority_queue<std::shared_ptr<PathPlanNode>, std::vector<std::shared_ptr<PathPlanNode>>, CmpPathNodePtrs> _open_set;
  
  uint32_t _num_cells;

  static constexpr uint32_t kMaxCnt = 500000;
};

} // namespace amrl