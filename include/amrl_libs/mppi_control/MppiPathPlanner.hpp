/*
  @file:      MppiPathPlanner.hpp
  @author:    psherman
  @date       June 2024
*/

#pragma once

#include <amrl_common/point/Point.hpp>

#include <amrl_libs/mapping/GridGraph.hpp>
#include <amrl_libs/mapping/OccupancyGrid.hpp>
#include <amrl_libs/obstacle/DynamicObstacle.hpp>
#include <amrl_libs/path_planning/AStarDynObs.hpp>

namespace amrl {

class MppiPathPlanner
{
public:

  MppiPathPlanner(
    std::shared_ptr<amrl::OccupancyGrid> map,
    uint32_t obs_buffer);

  ~MppiPathPlanner(void) = default;

  std::vector<Point<double>> plan_path(const Point<double> &start_pt, const Point<double> &end_pt);

  std::vector<Point<double>> sub_path(
    const Point<double> &rbt_pos,
    const uint32_t num_sub_steps);

  void set_path_manually(const std::vector<Point<double>> &path);

  void set_dyn_obs(std::shared_ptr<DynamicObstacle> dyn_obs);

private:
  std::shared_ptr<OccupancyGrid> _map;
  std::shared_ptr<GridGraph> _path_graph;
  std::shared_ptr<AStarDynObs> _path_planner;

  std::shared_ptr<DynamicObstacle> _dyn_obs;

  std::vector<Point<double>> _path;
  std::vector<Point<uint32_t>> _path_cells;

  const double _half_res; // 1/2 resolution of map. For getting middle of cells
};

}
