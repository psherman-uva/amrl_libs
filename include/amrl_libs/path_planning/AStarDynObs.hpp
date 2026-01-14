/*
  @file:      AStar.hpp
  @author:    psherman
  @date       May 2024
  
  @brief Extending A-Star algorithm for path planning to inlcude dynamic obstacle
*/

#pragma once

#include <amrl_libs/path_planning/AStar.hpp>
#include <amrl_libs/obstacle/DynamicObstacle.hpp>

#include <map>
#include <limits>

namespace amrl {

class AStarDynObs : public AStar
{

public:
  /// Constructor
  /// @param map Map object for planning path through
  /// @param func Heuristic function used for cost estimation
  AStarDynObs(std::shared_ptr<GridGraph> grid_graph,
    std::function<float(const Point<uint32_t>&, const Point<uint32_t>&)> func);

  /// Default destructor
  ~AStarDynObs(void) = default;

  /// Plan a path through map
  /// @param start Starting point for path
  /// @param goal Desired goal position for path
  /// @return Ordered set of connected points from start to goal
  std::vector<Point<uint32_t>> path_plan(const Point<uint32_t>& start, const Point<uint32_t>& goal) override;

  void add_dyn_obs(std::shared_ptr<DynamicObstacle> dyn_obs);

private:

  std::shared_ptr<DynamicObstacle> _dyn_obs;

  static constexpr float kMaxScore = std::numeric_limits<float>::max();
};

} // namespace amrl