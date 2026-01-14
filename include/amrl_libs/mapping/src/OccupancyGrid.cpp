/*
  @file:      OccupancyGrid.cpp
  @author:    psherman
  @date       June 2023
*/

#include <amrl_libs/mapping/OccupancyGrid.hpp>
#include <amrl_common/util/util.hpp>

#include <cmath>
#include <utility> // make_pair
#include <iostream>
#include <algorithm>

namespace amrl {


OccupancyGrid::OccupancyGrid(const Point<double> &origin,
    const double width,
    const double height,
    const double resolution,
    const double init_probability)
: Grid(origin, width, height, resolution, log(init_probability/(1.0-init_probability))),
  _prob_occ(_num_cells, init_probability),
  kLogFree(log(kFreeProb/(1-kFreeProb))), 
  kLogOccup(log(kOccupProb/(1-kOccupProb))),
  kLogNeighbor(log(kNeighborProb/(1-kNeighborProb)))
{
  // Initialize log-odds grid
  double init_odds = log(init_probability/(1.0-init_probability));

  // Set values for the mim and max log odds
  kLogOddsMin = init_odds;
  kLogOddsMax = init_odds;
  for (uint8_t i = 0; i < 25; ++i) {
    kLogOddsMin += kLogFree - init_odds;
    kLogOddsMax += kLogOccup - init_odds;
  }
}

OccupancyGrid::OccupancyGrid(const GridInfo &info,
    const double init_probability)
: OccupancyGrid(info.origin, info.width, info.height, info.resolution, init_probability)
{
}

double OccupancyGrid::probability_index_occupied(const uint32_t idx) const
{
  return _prob_occ.at(idx);
}

std::set<Point<uint32_t>> OccupancyGrid::occupied_cells_get(void) const
{
  return _occupied;
}

std::set<uint32_t> OccupancyGrid::occupied_indices_get(void) const
{
  return _occupied_idx;
}


void OccupancyGrid::occupied_cells_set(const std::set<Point<uint32_t>> &cells)
{
  // TODO: Need to set _grid to match
  _occupied = cells;
}

void OccupancyGrid::update_odds(
  const std::vector<Point<uint32_t>> &free_cells, 
  const std::vector<Point<uint32_t>> &occ_cells,
  const std::vector<Point<uint32_t>> &neighbor_cells)
{
  adjust_cell_odds(free_cells, kLogFree);
  adjust_cell_odds(occ_cells, kLogOccup);
  adjust_cell_odds(neighbor_cells, kNeighborProb);
}

void OccupancyGrid::adjust_cell_odds(const std::vector<Point<uint32_t>> &cells, const double adj_amount)
{
  for(auto it = cells.begin(); it != cells.end(); ++it) {
    Point<uint32_t> p = *it;
    if (cell_in_grid(p)) {
      size_t idx = p.x + _row_width*p.y;
      _grid[idx] += adj_amount;
      _grid[idx] = std::min(kLogOddsMax, std::max(kLogOddsMin, _grid[idx]));

      index_value_updated(idx, p);
    }
  }
}

void OccupancyGrid::index_set_value(const uint32_t idx, const double value)
{
  if (index_in_grid(idx)) {
    _grid[idx] = value;
    _grid[idx] = std::min(kLogOddsMax, std::max(kLogOddsMin, _grid[idx]));

    Point<uint32_t> p = index_to_cell(idx);
    index_value_updated(idx, p);
  }
}

void OccupancyGrid::index_value_updated(const uint32_t idx, const Point<uint32_t> &p)
{
  double prob = 1 - 1.0/(1+exp(_grid[idx]));
  _prob_occ[idx] = prob;

  if(_grid[idx] > kOccupLogThreshold){
    _occupied.insert(p);
    _occupied_idx.insert(idx);
  } else {
    _occupied.erase(p);
    _occupied_idx.erase(idx);
  }

  if(!_callbacks.empty()) {
    for_each(_callbacks.begin(), _callbacks.end(), [&p, prob](auto func) { func(p.x, p.y, prob); });
  }
}

bool OccupancyGrid::index_is_occupied(const uint32_t idx) const
{
  return idx < _num_cells ? _grid[idx] >= kOccupLogThreshold : false;
}

bool OccupancyGrid::index_is_free(const uint32_t idx) const
{
  return idx < _num_cells ? _grid[idx] <= kFreeLogThreshold : false;
}

bool OccupancyGrid::cell_is_occupied(const Point<uint32_t> &cell) const
{
  size_t idx = cell.x + _row_width*cell.y;
  return index_is_occupied(idx);
}

bool OccupancyGrid::cell_is_free(const Point<uint32_t> &cell) const
{
  size_t idx = cell.x + _row_width*cell.y;
  return index_is_free(idx);
}

bool OccupancyGrid::point_is_occupied(const Point<double> &pt) const
{
  Point<uint32_t> cell = point_to_cell(pt);
  return cell_is_occupied(cell);
}

bool OccupancyGrid::point_is_free(const Point<double> &pt) const
{
  Point<uint32_t> cell = point_to_cell(pt);
  return cell_is_free(cell);
}

void OccupancyGrid::register_callback(GridCallback func)
{
  _callbacks.push_back(std::move(func));
}

} // namespace amrl