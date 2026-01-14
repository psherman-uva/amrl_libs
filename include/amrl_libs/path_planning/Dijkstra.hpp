
#pragma once

#include <amrl_libs/mapping/GridGraph.hpp>

#include <vector>
#include <cstdint>
#include <map>

namespace amrl {

class Dijkstra
{
public:
  /// Constructor
  Dijkstra(std::shared_ptr<GridGraph> grid_graph);

  /// Default Desctructor
  ~Dijkstra(void) = default;

  /// Plan a path through map
  /// @param start Starting point for path
  /// @param goal Desired goal position for path
  /// @return Ordered set of connected points from start to goal
  std::vector<uint32_t> path_plan(const uint32_t start, const uint32_t goal);

private:
  std::vector<uint32_t> construct_path(
    const uint32_t goal_idx,
    const uint32_t start_idx) const;

  std::shared_ptr<GridGraph> _grid_graph;
  std::map<uint32_t, uint32_t> _parents;

  static constexpr uint32_t kMaxCnt = 1000000;
};

} // namespace amrl