
#include <amrl_libs/mapping/GridGraph.hpp>

#include <amrl_common/util/util.hpp>
#include <iostream>


namespace amrl { 

const std::vector<uint32_t> GridGraph::kAllNeighbors({0, 1, 2, 3, 4, 5, 6, 7});
const std::vector<uint32_t> GridGraph::kLeftNeighbors({1, 2, 4, 6, 7});
const std::vector<uint32_t> GridGraph::kRightNeighbors({0, 1, 3, 5, 6});
const std::vector<uint32_t> GridGraph::kTopNeighbors({0, 1, 2, 3, 4});
const std::vector<uint32_t> GridGraph::kBottomNeighbors({3, 4, 5, 6, 7});
const std::vector<uint32_t> GridGraph::kTopLeftNghbrs({1, 2, 4});
const std::vector<uint32_t> GridGraph::kTopRightNghbrs({0, 1, 3});
const std::vector<uint32_t> GridGraph::kBottomLeftNghbrs({4, 6, 7});
const std::vector<uint32_t> GridGraph::kBottomRightNghbrs({3, 5, 6});

const std::vector<double> GridGraph::kNeighborCost (
  {sqrt(2.0), 1.0, sqrt(2.0), 1.0, 1.0, sqrt(2.0), 1.0, sqrt(2.0)});

GridGraph::GridGraph(std::shared_ptr<OccupancyGrid> map, uint32_t buffer) :
    _map(map),
    _grid_size(map->grid_size_get()), 
    _row_width(_grid_size.first),
    _num_nodes(_grid_size.first * _grid_size.second),
    _obs_buffer(buffer),
    _graph(_grid_size.first * _grid_size.second),
    _occupied_cells()
{
  map->register_callback(
    [this](uint32_t x, uint32_t y, double prob) { this->update_data(x, y, prob); } );

  // Create all nodes
  for (uint32_t i = 0; i < _num_nodes; ++i) {
    _graph[i] = std::make_shared<PathPlanNode>(PathPlanNode(i, {}));
  }

  // Add all the edges
  for (uint32_t idx = 0; idx < _num_nodes; ++idx) {
    construct_all_edges(idx);
  }

  // Remove edges for occupied space
  for (uint32_t idx = 0; idx < _num_nodes; ++idx) {
    if(_map->index_is_occupied(idx)) {
      // std::cout << "Occupied idx: " << idx << std::endl;
      node_occupied_update(idx);
    }
  }
}

std::shared_ptr<PathPlanNode> GridGraph::node(uint32_t idx)
{
  return _graph[idx];
}

void GridGraph::update_data(const uint32_t x, const uint32_t y, const double probability)
{
  uint32_t idx = _map->cell_to_index({x, y});
  bool already_occupied =  (_occupied_cells.find(idx) != _occupied_cells.end());
  if(already_occupied && (probability < kOccupProb)) {
    node_free_update(idx);
  } else if ((!already_occupied) && (probability >= kOccupProb) ) {
    node_occupied_update(idx);
  }
}

void GridGraph::remove_edges(const uint32_t idx)
{
  // Jump to all neighbor nodes and remove this node from
  // their list of edges
  for (const auto &edge : _graph[idx]->edges) {
    std::shared_ptr<PathPlanNode> neighbor = edge.n;
    for(auto it = neighbor->edges.begin(); it < neighbor->edges.end(); ++it) {
      if ((*it).n->id == idx) {
        neighbor->edges.erase(it);
        break;
      }
    }
  }

  // Clear this nodes edges
  _graph[idx]->edges.clear();
}

void GridGraph::add_edges(const uint32_t idx)
{
  // Construct all edges for node being re-added to graph
  construct_all_edges(idx);

  // For all connected nodes, add this node back in.
  for (const auto &edge : _graph[idx]->edges) {
    std::shared_ptr<PathPlanNode> neighbor = edge.n;
    double cost = get_default_edge_cost(idx, edge.n->id);
    edge.n->edges.push_back(PathPlanEdge(_graph[idx], cost));
  }
}

void GridGraph::construct_all_edges(const uint32_t idx)
{
  std::vector<uint32_t> neighbors = get_all_neighbor_directions(idx);

  for (const auto &n : neighbors) {
    uint32_t idx_n = get_neighbor(idx, n);
    double cost    = kNeighborCost[n];
    if(_occupied_cells.find(idx_n) == _occupied_cells.end()) {
      _graph[idx]->edges.push_back(PathPlanEdge(_graph[idx_n], cost));
    }
  }
}

void GridGraph::node_occupied_update(const uint32_t idx)
{
  std::set<uint32_t> neighborhood;
  // index_connected_neighborhood(neighborhood, idx, _obs_buffer);
  index_full_neighborhood(neighborhood, idx, _obs_buffer);

  for(const auto &i : neighborhood) {
    _occupied_cells.insert(i);
    remove_edges(i);
  }
}

void GridGraph::node_free_update(const uint32_t idx)
{
  std::set<uint32_t> neighborhood;
  index_full_neighborhood(neighborhood, idx, _obs_buffer);

  for(const auto ni : neighborhood) {
    bool marked_occ = _occupied_cells.find(ni) != _occupied_cells.end();
    if(marked_occ && !_map->index_is_occupied(ni)) {
      std::set<uint32_t> nghbr;
      index_full_neighborhood(nghbr, ni, _obs_buffer);

      bool ni_free = true;
      for(const auto n : nghbr) {
        if(_map->index_is_occupied(n)) { 
          ni_free = false;
        }
      }

      if(ni_free) {
        _occupied_cells.erase(ni);
        add_edges(ni);
      }
    }
  }


}

void GridGraph::index_connected_neighborhood(std::set<uint32_t>& neighborhood, const uint32_t idx, const uint32_t step)
{
  neighborhood.insert(idx);
  if(step > 0) {
    for(const auto &edge : _graph[idx]->edges) {
      index_connected_neighborhood(neighborhood, edge.n->id, step-1);
    }
  }
}

void GridGraph::index_full_neighborhood(std::set<uint32_t>& neighborhood, const uint32_t idx, const uint32_t step)
{
  neighborhood.insert(idx);
  if(step > 0) {
    std::vector<uint32_t> neighbors = get_all_neighbor_directions(idx);
    for(const auto n : neighbors) {
      uint32_t nidx = get_neighbor(idx, n);
      index_full_neighborhood(neighborhood, nidx, step-1);
    }
  }
}


uint32_t GridGraph::get_neighbor(uint32_t idx, uint32_t nbr) const
{
  switch(nbr) {
    case 0: return idx - _row_width - 1;
    case 1: return idx - _row_width;
    case 2: return idx - _row_width + 1;
    case 3: return idx - 1;
    case 4: return idx + 1;
    case 5: return idx + _row_width - 1;
    case 6: return idx + _row_width;
    case 7: return idx + _row_width + 1;
  }

  return idx;
}

std::vector<uint32_t> GridGraph::get_all_neighbor_directions(const uint32_t idx) const
{
  std::vector<uint32_t> neighbors = kAllNeighbors;
  if(bottom_row_check(idx)) {
    if(left_side_check(idx)) {
      neighbors = kBottomLeftNghbrs;
    } else if(right_side_check(idx)) {
      neighbors = kBottomRightNghbrs;
    } else {
      neighbors = kBottomNeighbors;
    }
  } else if(top_row_check(idx)) {
    if(left_side_check(idx)) {
      neighbors = kTopLeftNghbrs;
    } else if(right_side_check(idx)) {
      neighbors = kTopRightNghbrs;
    } else {
      neighbors = kTopNeighbors;
    }
  } else if(left_side_check(idx)) {
    neighbors = kLeftNeighbors;
  } else if(right_side_check(idx)) {
    neighbors = kRightNeighbors;
  }
  return neighbors;
}

double GridGraph::get_default_edge_cost(uint32_t idx, uint32_t nbr) const
{
  int diff         = abs(static_cast<int>(nbr) - static_cast<int>(idx));
  uint32_t diffcmp = static_cast<uint32_t>(diff);
  
  if (diffcmp == 1 || diffcmp == _row_width) {
    return 1.0;
  } 
  return sqrt(2.0);
}

bool GridGraph::bottom_row_check(const uint32_t idx) const
{
  return idx < _row_width;
}

bool GridGraph::top_row_check(const uint32_t idx) const
{
  return idx >= (_num_nodes - _row_width);
}

bool GridGraph::left_side_check(const uint32_t idx) const
{
  return (idx % _row_width) == 0;
}

bool GridGraph::right_side_check(const uint32_t idx) const
{
  return (idx % _row_width) == (_row_width - 1);
}

uint32_t GridGraph::cell_to_index(const Point<uint32_t> &cell) const
{
  return _map->cell_to_index(cell);
}

Point<uint32_t> GridGraph::index_to_cell(uint32_t idx) const
{
  return _map->index_to_cell(idx);
}

std::vector<std::shared_ptr<PathPlanNode>> GridGraph::get_graph(void)
{
  return _graph;
}

std::shared_ptr<OccupancyGrid> GridGraph::get_map(void)
{
  return _map;
}

void GridGraph::print_graph(void)
{
  for (const auto &nd : _graph) {
    std::cout << "Node: " << nd->id << std::endl;
    for (const auto &e : nd->edges) {
      std::cout << "\t-->" << e.n->id << ": " << e.cost << std::endl;
    }
  }
}


} // namespace amrl