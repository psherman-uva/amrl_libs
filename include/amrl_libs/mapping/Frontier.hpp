/*
  @file:      Frontier.hpp
  @author:    psherman
  @date       June 2023
  
  @brief Implementation of Frontier Points
*/

#pragma once

#include <amrl_common/point/Point.hpp>

#include <vector>
#include <set>
#include <cstdint>

namespace amrl {

class Frontier 
{
private:
  enum class State : uint8_t { 
    Free     = 0, 
    Occupied = 1, 
    Unknown  = 2};

public:
  /// Constructor
  /// @param width Number of cells along width of grid
  /// @param height Numner of cells along height of grid
  /// @param resolution
  /// @param origin
  Frontier(const double width, const double height, const double resolution, const Point<double> &origin);

  ~Frontier(void) = default;

  std::set<Point<uint32_t>> get_frontier(void) const;

  std::set<Point<double>> get_frontier_pts(void) const;

  bool point_in_frontier(const Point<uint32_t> &pt) const;

  void update(const uint32_t x, const uint32_t y, const double probability);

  int size(void) const;

  bool empty(void) const;

private:

  void remove_point(const Point<uint32_t> &pt);

  bool frontier_check(const Point<uint32_t> &pt) const;

  bool index_valid(const uint32_t x, const uint32_t y) const;


  std::set<Point<uint32_t>> _frontier;

  std::set<Point<double>> _frontier_pts;

  std::vector<std::vector<State>> _grid;

  const uint32_t _width;
  const uint32_t _height;

  static constexpr double kOccupiedLimit = 0.8;
  static constexpr double kFreeLimit     = 0.2;

  const double _resolution;
  const double _half_resolution;
  const Point<double> _origin;
};


} // namespace amrl