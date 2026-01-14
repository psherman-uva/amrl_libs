/*
  @file:      TerrainGrid.hpp
  @author:    psherman
  @date       Sept. 2023

*/


#pragma once

#include <amrl_libs/mapping/Grid.hpp>
#include <amrl_common/point/Point.hpp>

#include <boost/optional.hpp>

#include <array>
#include <set>
#include <vector>
#include <memory>

namespace amrl {


class TerrainGrid : public Grid
{
public:
  TerrainGrid(const Point<double> &origin,
    const double width,
    const double height,
    const double resolution);

  ~TerrainGrid(void) = default;

  double terrain_at_cell(const Point<uint32_t> &pt) const;
  double terrain_at_point(const Point<double> &pt) const;

  bool hazard_at_cell(const Point<uint32_t> &pt) const;
  bool hazard_at_point(const Point<double> &pt) const;

  void add_hazard_cells(const std::vector<Point<uint32_t>> &cells);
  void add_terrain_cells(const std::vector<Point<uint32_t>> &cells, const double terrain_value);

  std::set<Point<uint32_t>> get_hazard_cells(void) const;
  

  // void set_display(std::shared_ptr<TerrainGridDisplay> display);

private:

  // std::shared_ptr<TerrainGridDisplay> _display;

  std::set<Point<uint32_t>> _hazard_cells;
};



}