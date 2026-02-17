
#include <amrl_libs/mppi_control/MppiPathPlanner.hpp>
#include <amrl_common/util/util.hpp>

namespace amrl {


MppiPathPlanner::MppiPathPlanner(
    std::shared_ptr<amrl::OccupancyGrid> map,
    uint32_t obs_buffer) :
  _map(map),
  _path_graph(nullptr),
  _path_planner(nullptr),
  _dyn_obs(nullptr),
  _half_res(_map->resolution()/2.0)
{
  _path_graph    = std::make_shared<GridGraph>(_map, obs_buffer);
  _path_planner  = std::make_shared<AStarDynObs>(_path_graph,
                    [] (const Point<uint32_t>& pt, const Point<uint32_t>& goal) -> float { return util::distance<float>(pt, goal); });
}

std::vector<Point<double>> MppiPathPlanner::plan_path(const Point<double> &start_pt, const Point<double> &end_pt)
{
  Point<uint32_t> start_cell = _map->point_to_cell(start_pt);
  Point<uint32_t> end_cell   = _map->point_to_cell(end_pt);

  _path_cells = _path_planner->path_plan(start_cell, end_cell);
  _path.resize(_path_cells.size());
  for(size_t i = 0; i < _path_cells.size(); ++i) {
    _path[i] = _map->cell_to_point(_path_cells[i]) + _half_res;
  }

  return _path;
}

std::vector<Point<double>> MppiPathPlanner::sub_path(
  const Point<double> &rbt_pos, const uint32_t num_sub_steps)
{
  std::vector<Point<double>> sub_pth(num_sub_steps);
  Point<double> close_pt;

  double dist  = std::numeric_limits<double>::max();
  uint32_t idx = 0;

  for(uint32_t i = 0; i < _path.size(); ++i) {
    double d = util::distance<double>(rbt_pos, _path[i]);
    if(d < dist) {
      dist = d;
      idx  = i;
      close_pt.x = _path[i].x;
      close_pt.y = _path[i].y;
    }
  }

  uint32_t end_idx = std::min<uint32_t>(idx + num_sub_steps, _path.size());
  std::copy(_path.begin()+idx, _path.begin()+end_idx, sub_pth.begin());

  uint32_t dSteps  = end_idx - idx;
  if(dSteps < num_sub_steps) {
    uint32_t steps_add = num_sub_steps - dSteps;
    std::fill_n(sub_pth.begin()+dSteps, steps_add, _path.back());
  }

  return sub_pth;
}

void MppiPathPlanner::set_path_manually(const std::vector<Point<double>> &path)
{
  _path = path;
}

void MppiPathPlanner::set_dyn_obs(std::shared_ptr<DynamicObstacle> dyn_obs)
{
  _dyn_obs = dyn_obs;
  _path_planner->add_dyn_obs(_dyn_obs);
}

}