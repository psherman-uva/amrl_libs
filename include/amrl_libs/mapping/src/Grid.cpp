/*
  @file:      Grid.cpp
  @author:    psherman
  @date       Sept. 2023
*/

#include <amrl_libs/mapping/Grid.hpp>
#include <amrl_common/util/util.hpp>

#include <cmath>
#include <utility> // make_pair
#include <iostream>
#include <algorithm>

namespace amrl {


Grid::Grid(const Point<double> &origin,
    const double width,
    const double height,
    const double resolution,
    const double init_value) :  
  _grid(),
  _resolution(resolution),
  _half_resolution(_resolution/2.0),
  _origin(origin),
  _size(width, height),
  _max_pt(),
  _row_width(static_cast<uint32_t>(round(width/_resolution))),
  _col_height(static_cast<uint32_t>(round(height/_resolution))),
  _num_cells(_row_width*_col_height),
  _grid_size({_row_width, _col_height})
{
  _max_pt = _origin + _size;

  // Initialize grid
  _grid = std::vector<double>(_num_cells, init_value);
}

Grid::Grid(const GridInfo &info,
    const double init_value) 
: Grid(info.origin, info.width, info.height, info.resolution, init_value) 
{
}

uint32_t Grid::num_cells(void) const
{
  return _grid.size();
}

GridInfo Grid::info_get(void) const
{
  return GridInfo(_origin, _size.x, _size.y, _resolution);
}

bool Grid::point_in_grid(const Point<double> &pt) const
{
  return pt.x >= _origin.x && pt.x < _max_pt.x
      && pt.y >= _origin.y && pt.y < _max_pt.y;
}

bool Grid::cell_in_grid(const Point<uint32_t> &pt) const
{
  return pt.x < _grid_size.first && pt.y < _grid_size.second;
}

std::pair<double, double> Grid::size_get(void) const
{
  return std::make_pair(_size.x, _size.y);
}

std::pair<uint32_t, uint32_t> Grid::grid_size_get(void) const
{
  return std::make_pair(_grid_size.first, _grid_size.second);
}

double Grid::resolution(void) const
{
  return _resolution;
}

const std::vector<double>& Grid::grid_get(void) const
{
  return _grid;
}

void Grid::grid_set(std::vector<double> grid)
{
  if (grid.size() == _grid.size())
    _grid = grid;
}

Point<double> Grid::cell_to_point(const Point<uint32_t> &pt) const
{
  Point<double> pt_real(pt);
  return (pt_real*_resolution) + _origin;// + _half_resolution;
}

uint32_t Grid::cell_to_index(const Point<uint32_t> &cell) const
{
  return cell.x + _row_width*cell.y;
}

Point<uint32_t> Grid::point_to_cell(const Point<double> &pt) const
{
  Point<double> p_adj = (pt - _origin) / _resolution;
  int16_t c_x = std::min<int16_t>(_grid_size.first, std::max<int16_t>(0, static_cast<int16_t>(p_adj.x)));
  int16_t c_y = std::min<int16_t>(_grid_size.second, std::max<int16_t>(0, static_cast<int16_t>(p_adj.y)));

  return {static_cast<uint32_t>(c_x), static_cast<uint32_t>(c_y)};
}

uint32_t Grid::point_to_index(const Point<double> &pt) const
{
  return cell_to_index(point_to_cell(pt));
}

Point<uint32_t> Grid::index_to_cell(const uint32_t idx) const
{
  return Point<uint32_t>(idx % _row_width, idx / _row_width);
}

Point<double> Grid::index_to_point(const uint32_t idx) const
{
  Point<uint32_t> cell = index_to_cell(idx);
  return cell_to_point(cell);
}

bool Grid::index_in_grid(const uint32_t idx) const
{
  return idx < _num_cells;
}

double Grid::value_from_index(const uint32_t idx) const
{
  if (idx < _grid.size()) {
    return _grid[idx];
  }
  return -1.0;
}

double Grid::value_from_cell(const Point<uint32_t> &cell) const
{
  return value_from_index(cell_to_index(cell));
}

double Grid::value_from_point(const Point<double> &pt) const
{
  return value_from_cell(point_to_cell(pt));
}

void Grid::index_set_value(const uint32_t idx, const double value)
{
  if(idx < _grid.size()) {
    _grid[idx] = value;
  }
}

void Grid::cell_set_value(const Point<uint32_t> &cell, const double value)
{
  index_set_value(cell_to_index(cell), value);
}

void Grid::point_set_value(const Point<double> &pt, const double value)
{
  index_set_value(point_to_index(pt), value);
}

void Grid::copy_grid_value(std::shared_ptr<Grid> rhs)
{
  _grid = rhs->_grid;
}

std::vector<double> Grid::map_limits(void) const
{
  std::vector<double> limits(4);

  limits[0] = _origin.x;
  limits[1] = _origin.y;
  limits[2] = _max_pt.x;
  limits[3] = _max_pt.y;
  
  return limits;
}

std::vector<Point<uint32_t>> Grid::cells_in_obstacle(const Obstacle &obs) const
{
  // Get min/max x- and y- cells covered by obstacle
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  obs.min_max_coordinates(min_x, max_x, min_y, max_y);

  Point<uint32_t> min_cell = point_to_cell({min_x, min_y});
  Point<uint32_t> max_cell = point_to_cell({max_x, max_y});

  // Check all cells in min/min range
  std::vector<Point<uint32_t>> cells_in_obs;
  for(uint32_t i = min_cell.x; i <= max_cell.x; ++i) {
    for(uint32_t j = min_cell.y; j <= max_cell.y; ++j) {

      Point<double> pt = cell_to_point({i, j}) + _half_resolution;
      if(obs.is_inside(pt)) {
        cells_in_obs.push_back({i, j});
      }
    }
  }

  return cells_in_obs;
}

} // namespace amrl