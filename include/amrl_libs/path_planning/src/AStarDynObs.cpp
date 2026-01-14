/*
  @file:      AStar.cpp
  @author:    psherman
  @date       July 2023
*/

#include <amrl_libs/path_planning/AStarDynObs.hpp>
#include <amrl_common/util/util.hpp>

#include <limits>
#include <cmath>
#include <queue>

namespace amrl {

AStarDynObs::AStarDynObs(
    std::shared_ptr<GridGraph> grid_graph,
    std::function<float(const Point<uint32_t>&, const Point<uint32_t>&)> func) : 
  AStar(grid_graph, func),
  _dyn_obs(nullptr)
{
}

std::vector<Point<uint32_t>> AStarDynObs::path_plan(
  const Point<uint32_t> &start,
  const Point<uint32_t> &goal)
{
  // Initialization
  initialize(start, goal);
  
  // ------------------------- // 
  // --       A* Loop       -- //
  // ------------------------- //
  uint32_t cnt = 0;  // Counter to stop loop if algorithm can't finish for any reason
  std::shared_ptr<PathPlanNode> curr;
  
  while(!_open_set.empty() && (++cnt) < kMaxCnt) {

    // Pop next cell to check off Priority Queue
    curr = _open_set.top();
    _open_set.pop();
    
    // Have we reached our goal??
    if(curr->id == _goal_idx) { return construct_path(_goal_idx, _start_idx); }

    // Expand to all neighboring cells
    for (const auto &edge : curr->edges) {
      uint32_t nbr_idx = edge.n->id;

      float tentative_g_score = _g_scores[curr->id];
      tentative_g_score      += edge.cost;

      // Super hacky, if neighbor cell is in the obstacles.
      // Just give it a super high score.
      // Would be better to somehow dynamically remove nodes from graph.
      // But too complicated for today.
      if(_dyn_obs) {
        Point<double> pt = _grid_graph->get_map()->index_to_point(nbr_idx);
        if(_dyn_obs->is_inside(pt)) {
          tentative_g_score = kMaxScore;
        }
      }

      if(tentative_g_score < _g_scores[nbr_idx]) {
        Point<uint32_t> nbr_cell = _grid_graph->index_to_cell(nbr_idx);

        _parents[nbr_idx]  = curr->id;
        _g_scores[nbr_idx] = tentative_g_score;

        edge.n->dist_cost = tentative_g_score + _H_func(nbr_cell, goal);
        _open_set.push(edge.n);
      }
    }
  }

  return {};
}

void AStarDynObs::add_dyn_obs(std::shared_ptr<DynamicObstacle> dyn_obs)
{
  _dyn_obs = dyn_obs;
}


} // namespace amrl