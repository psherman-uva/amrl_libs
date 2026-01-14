
#include <amrl_libs/path_planning/Dijkstra.hpp>

#include <limits>
#include <queue>
#include <iostream>
#include <algorithm>

namespace amrl {

Dijkstra::Dijkstra(std::shared_ptr<GridGraph> grid_graph)
  : _grid_graph(grid_graph)
{
}

std::vector<uint32_t> Dijkstra::path_plan(const uint32_t start, const uint32_t goal)
{
  auto graph = _grid_graph->get_graph();

  // Initialization   
  std::for_each(graph.begin(), graph.end(), 
    [](std::shared_ptr<PathPlanNode> n) { n->dist_cost = std::numeric_limits<double>::max(); });
  _parents.clear();

  std::shared_ptr<PathPlanNode> start_node = graph[start];
  start_node->dist_cost = 0.0;

  std::priority_queue<std::shared_ptr<PathPlanNode>, std::vector<std::shared_ptr<PathPlanNode>>, CmpPathNodePtrs> open_set;
  open_set.push(start_node);
  _parents[start] = start;

  // ----------------------------------- // 
  // --  Dijkstra Path Planning Loop  -- //
  // ----------------------------------- //

  std::shared_ptr<PathPlanNode> curr;
  uint32_t cnt = 0;

  while(!open_set.empty() && (++cnt) < kMaxCnt) {
    // Get the node with the lowest path cost. If at goal, we're done
    curr = open_set.top();
    if(curr->id == goal) { return construct_path(start, goal); }
    open_set.pop();

    // Expand to all neighboring nodes via edges
    for (const auto &edge : curr->edges) {
      double temp_score = curr->dist_cost;
      temp_score += edge.cost;
      if (temp_score < edge.n->dist_cost) {
        _parents[edge.n->id] = curr->id;
        edge.n->dist_cost    = temp_score;
        open_set.push(edge.n);
      }
    }
  }

  return std::vector<uint32_t>();
}

std::vector<uint32_t> Dijkstra::construct_path(
    const uint32_t start_idx,
    const uint32_t goal_idx) const
{
  std::vector<uint32_t> path;

  // Create the path from "goal" back to "start"
  uint32_t path_idx = goal_idx;
  while(path_idx != start_idx) {
    path.push_back(path_idx);
    path_idx = _parents.at(path_idx);
  }
  path.push_back(start_idx);

  // Reverse path in place to get path in desired order (start->goal)
  std::reverse(path.begin(), path.end());

  return path;
}

} // namespace amrl