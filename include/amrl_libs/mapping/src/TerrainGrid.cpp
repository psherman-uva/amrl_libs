/*
  @file:      TerrainGrid.cpp
  @author:    psherman
  @date       Sept. 2023
*/

#include <amrl_libs/mapping/TerrainGrid.hpp>
#include <amrl_common/util/util.hpp>

#include <cmath>
#include <utility> // make_pair
#include <iostream>
#include <algorithm>

namespace amrl {

TerrainGrid::TerrainGrid(const Point<double> &origin,
    const double width,
    const double height,
    const double resolution)
: Grid(origin, width, height, resolution, 1.0)
{   
}

double TerrainGrid::terrain_at_cell(const Point<uint32_t> &pt) const
{
  if (cell_in_grid(pt)) {
    size_t idx = pt.x + _row_width*pt.y;
    return _grid[idx];
  }
  return 1.0;
}

double TerrainGrid::terrain_at_point(const Point<double> &pt) const
{
  return terrain_at_cell(point_to_cell(pt));
}

bool TerrainGrid::hazard_at_cell(const Point<uint32_t> &pt) const
{
  if (cell_in_grid(pt)) {
    size_t idx = pt.x + _row_width*pt.y;
    return _grid[idx] == 0.0;
  }
  return false;
}

bool TerrainGrid::hazard_at_point(const Point<double> &pt) const
{
  return hazard_at_cell(point_to_cell(pt));
}

void TerrainGrid::add_hazard_cells(const std::vector<Point<uint32_t>> &cells)
{
  std::vector<Point<double>> pts;
  for(const auto &c : cells) {
    if (cell_in_grid(c)) {
      size_t idx = c.x + _row_width*c.y;
      _grid[idx] = 0.0;
      _hazard_cells.insert(c);
      // if(_display) {
      //   pts.push_back(cell_to_point(c));
      // }
    }
  }

  // if(_display) {
  //   _display->add_terrain(pts);
  // }
}

void TerrainGrid::add_terrain_cells(const std::vector<Point<uint32_t>> &cells, const double terrain_value)
{
  for(const auto &c : cells) {
    if (cell_in_grid(c)) {
      size_t idx = c.x + _row_width*c.y;
      _grid[idx] = terrain_value;
    }
  }
}

std::set<Point<uint32_t>> TerrainGrid::get_hazard_cells(void) const
{
  return _hazard_cells;
}

// void TerrainGrid::set_display(std::shared_ptr<TerrainGridDisplay> display)
// {
//   _display = display;
// }

}