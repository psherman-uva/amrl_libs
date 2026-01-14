/*
  @file:      Frontier.cpp
  @author:    psherman
  @date       June 2023
*/

#include <amrl_libs/mapping/Frontier.hpp>

namespace amrl {

Frontier::Frontier(const double width, const double height, const double resolution, const Point<double> &origin) 
: _width(width), _height(height), _resolution(resolution), _half_resolution(_resolution/2.0), _origin(origin)
{
  // Initialize 2D grid of cells in the "Unknown" state
  std::vector<State> v(height, State::Unknown);
  for (int i = 0; i < width; ++i) {
    _grid.push_back(v);
  }
}

std::set<Point<uint32_t>> Frontier::get_frontier(void) const
{
  return _frontier;
}

std::set<Point<double>> Frontier::get_frontier_pts(void) const
{
  return _frontier_pts;
}

bool Frontier::point_in_frontier(const Point<uint32_t> &pt) const
{
  return _frontier.find(pt) != _frontier.end();
}

void Frontier::update(const uint32_t x, const uint32_t y, const double probability)
{
  if (index_valid(x, y)) {
    if(probability >= kOccupiedLimit) {
      _grid[x][y] = State::Occupied;
    } else if (probability <= kFreeLimit && _grid[x][y] != State::Occupied) {
      _grid[x][y] = State::Free;
    } else if (_grid[x][y] != State::Occupied){
      _grid[x][y] = State::Unknown;
    }

    // If point is a valid frontier point, include in set
    Point<uint32_t> p(x, y);
    if (frontier_check(p)) {
      _frontier.insert(p);

      Point<double> pt_real = Point<double>(p)*_resolution + _origin + _half_resolution;
      _frontier_pts.insert(pt_real);
    }

    // Need to check neighboring cells to see if update changes frontier
    std::vector<Point<uint32_t>> test_pts({{x-1, y}, {x+1, y}, {x, y-1}, {x, y+1}});
    for(const auto &p : test_pts) {
      if (!frontier_check(p)) {
        _frontier.erase(p);
        
        Point<double> pt_erase = Point<double>(p)*_resolution + _origin + _half_resolution;
        _frontier_pts.erase(pt_erase);
      }
    }

    // If point that was previously in frontier 
    // is now marked as occuppied remove from frontier
    if((probability >= kOccupiedLimit) && (_frontier.find(p) != _frontier.end())) {
      Point<double> pt_erase = Point<double>(p)*_resolution + _origin + _half_resolution;

      _frontier.erase(p);
      _frontier_pts.erase(pt_erase);
    }
  }
}

void Frontier::remove_point(const Point<uint32_t> &pt)
{
  _frontier.erase(pt);
}

bool Frontier::frontier_check(const Point<uint32_t> &pt) const
{
  uint32_t x = pt.x;
  uint32_t y = pt.y;

  if(index_valid(x, y) && (_grid[x][y] == State::Free)) {
    bool north   = index_valid(x+1,y) && (_grid[x+1][y] == State::Unknown);
    bool south   = index_valid(x-1,y) && (_grid[x-1][y] == State::Unknown);
    bool east    = index_valid(x,y+1) && (_grid[x][y+1] == State::Unknown);
    bool west    = index_valid(x,y-1) && (_grid[x][y-1] == State::Unknown);

    return north || south || east || west;
  }
  return false;
}

bool Frontier::index_valid(const uint32_t x, const uint32_t y) const
{
  return x < _width && y < _height;
}

int Frontier::size(void) const
{
  return _frontier.size();
}

bool Frontier::empty(void) const
{
  return _frontier.empty();
}



} // namespace