/*
  @file:      PathPlanningFunctions.hpp
  @author:    psherman
  @date       Feb. 2026
  
  @brief Helpful functions when dealing with path planning objects
*/

#pragma once

#include <amrl_libs/path_planning/PathPlanNode.hpp>
#include <amrl_libs/mapping/GridGraph.hpp>
#include <amrl_libs/mapping/OccupancyGrid.hpp>

namespace amrl {
namespace util {

std::vector<Point<uint32_t>> smooth_path(
  const std::vector<Point<uint32_t>> &path_cells, 
  std::shared_ptr<GridGraph> graph);


}
}
