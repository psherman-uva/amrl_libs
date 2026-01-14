/*
  @file:      AStar.cpp
  @author:    psherman
  @date       July 2023
*/

#include <amrl_libs/path_planning/AStar.hpp>
#include <amrl_common/util/util.hpp>

#include <limits>
#include <cmath>

namespace amrl {

AStar::AStar(
    std::shared_ptr<GridGraph> grid_graph,
    std::function<float(const Point<uint32_t>&, const Point<uint32_t>&)> func)
: _grid_graph(grid_graph), 
  _H_func(func),
  _num_cells(grid_graph->get_graph().size())
{
  _g_scores.resize(_num_cells, std::numeric_limits<float>::max());
}


void AStar::initialize(const Point<uint32_t>& start, const Point<uint32_t>& goal)
{
  // Reset
  std::fill(_g_scores.begin(), _g_scores.end(), std::numeric_limits<float>::max());
  _parents.clear();
  
  // Initialization
  _goal_idx  = _grid_graph->cell_to_index(goal);
  _start_idx = _grid_graph->cell_to_index(start);

  // Node start_node(start_idx, _H_func(start, goal));
  _start_node = _grid_graph->node(_start_idx);
  _start_node->dist_cost = _H_func(start, goal);
  _g_scores[_start_idx]  = 0.0;

  _open_set = std::priority_queue<std::shared_ptr<PathPlanNode>, std::vector<std::shared_ptr<PathPlanNode>>, CmpPathNodePtrs>();
  _open_set.push(_start_node);
}

std::vector<Point<uint32_t>> AStar::path_plan(
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

std::vector<Point<uint32_t>> AStar::construct_path(
  const uint32_t goal_idx,
  const uint32_t start_idx) const
{
  std::vector<uint32_t> parent_path;
  parent_path.reserve(_parents.size());

  uint32_t path_idx = goal_idx;
  while (path_idx != start_idx) {
    parent_path.push_back(path_idx);
    path_idx = _parents.at(path_idx);
  }
  parent_path.push_back(path_idx);

  std::vector<Point<uint32_t>> path;
  path.reserve(parent_path.size());
  for (auto rit = parent_path.rbegin(); rit != parent_path.rend(); ++rit) {
    path.push_back(_grid_graph->index_to_cell(*rit));
  }
  return path;
}

} // namespace amrl