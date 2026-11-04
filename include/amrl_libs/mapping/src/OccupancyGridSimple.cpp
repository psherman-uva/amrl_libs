/*
  @file:      OccupancyGridSimple.cpp
  @author:    psherman
  @date       June 2023
*/

#include <amrl_libs/mapping/OccupancyGridSimple.hpp>
#include <amrl_common/util/util.hpp>

#include <cmath>
#include <utility> // make_pair
#include <iostream>
#include <algorithm>

namespace amrl {

OccupancyGridSimple::OccupancyGridSimple(const Point<double> &origin,
    const double width,
    const double height,
    const double resolution) : 
  Grid(origin, width, height, resolution, kUnknown)
{
}

OccupancyGridSimple::OccupancyGridSimple(const GridInfo &info) : 
  OccupancyGridSimple(info.origin, info.width, info.height, info.resolution)
{
}

void OccupancyGridSimple::reset(void)
{
  for(size_t idx = 0; idx < _num_cells; ++idx ) {
    index_set_value(idx, kUnknown);
  } 
}

std::set<Point<uint32_t>> OccupancyGridSimple::occupied_cells_get(void) const
{
  return _occupied;
}

std::set<uint32_t> OccupancyGridSimple::occupied_indices_get(void) const
{
  return _occupied_idx;
}

void OccupancyGridSimple::cells_set_free(const std::vector<Point<uint32_t>> &cells)
{
  cells_set_values(cells, kFree);
}

void OccupancyGridSimple::cells_set_occupied(const std::vector<Point<uint32_t>> &cells)
{
  cells_set_values(cells, kOccup);
}

void OccupancyGridSimple::cells_set_values(const std::vector<Point<uint32_t>> &cells, const double value)
{
  for(auto it = cells.begin(); it != cells.end(); ++it) {
    Point<uint32_t> p = *it;
    if (cell_in_grid(p)) {
      size_t idx = p.x + _row_width*p.y;

      _grid[idx] = value;
      index_value_updated(idx, p);
    }
  }
}

void OccupancyGridSimple::index_set_value(const uint32_t idx, const double value)
{
  if (index_in_grid(idx)) {
    _grid[idx] = value;
    Point<uint32_t> p = index_to_cell(idx);
    index_value_updated(idx, p);
  }
}

void OccupancyGridSimple::index_value_updated(const uint32_t idx, const Point<uint32_t> &p)
{
  const double val = _grid[idx];
  if(val > kOccThreshold){
    _occupied.insert(p);
    _occupied_idx.insert(idx);
  } else {
    _occupied.erase(p);
    _occupied_idx.erase(idx);
  }

  std::for_each(_callbacks.begin(), _callbacks.end(), [&p, val](auto func) { func(p.x, p.y, val); });
}

bool OccupancyGridSimple::index_is_occupied(const uint32_t idx) const
{
  return idx < _num_cells ? _grid[idx] >= kOccThreshold : false;
}

bool OccupancyGridSimple::index_is_free(const uint32_t idx) const
{
  return idx < _num_cells ? _grid[idx] <= kFreeThreshold : false;
}

bool OccupancyGridSimple::cell_is_occupied(const Point<uint32_t> &cell) const
{
  size_t idx = cell.x + _row_width*cell.y;
  return index_is_occupied(idx);
}

bool OccupancyGridSimple::cell_is_free(const Point<uint32_t> &cell) const
{
  size_t idx = cell.x + _row_width*cell.y;
  return index_is_free(idx);
}

bool OccupancyGridSimple::point_is_occupied(const Point<double> &pt) const
{
  Point<uint32_t> cell = point_to_cell(pt);
  return cell_is_occupied(cell);
}

bool OccupancyGridSimple::point_is_free(const Point<double> &pt) const
{
  Point<uint32_t> cell = point_to_cell(pt);
  return cell_is_free(cell);
}

void OccupancyGridSimple::register_callback(GridCallback func)
{
  _callbacks.push_back(std::move(func));
}

void OccupancyGridSimple::add_solid_obstacle(const Obstacle &obs)
{
  std::vector<Point<uint32_t>> obs_cells = cells_in_obstacle(obs);
  if(obs_cells.size() > 0) {
    cells_set_occupied(obs_cells);
  }
}

} // namespace amrl