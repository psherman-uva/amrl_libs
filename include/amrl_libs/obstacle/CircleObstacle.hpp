
#pragma once

#include <amrl_libs/obstacle/ObstacleShape.hpp>


namespace amrl {

class CircleObstacle : public ObstacleShape 
{
public:

  /// Constructor
  /// @param center Initial center position of the circle shaped obstacle 
  /// @param radius Intial radius of the circle shaped obstacle
  CircleObstacle(const Point<double> &center, const double radius);
  
  /// Default destructor
  ~CircleObstacle(void) = default;

  /// Check if a point is inside the obstacle defining polygon
  /// @param p Point to check if in obstacle
  /// @return True if point is inside the obstacle
  bool is_inside(const Point<double> &p) const override;

  /// Check if a point is on the border of the of the obstacle
  /// @param pt Point to check if on border
  /// @return True if point is directly on the border of the obstacle
  bool on_border(const Point<double> &p) const override;

  /// Compute the shortest distance from a point in space to the obstacle
  /// @note If point is inside obstacle, function will return 0
  /// @param p Point in space to get distance from
  /// @return Distance between point and obstacle
  std::pair<double, std::vector<double>> distance_from_point(const Point<double> &pt) const override;

  /// Check if a line segment crosses any side of an obstacle and
  /// returns if so, returns distance to intersection point.
  /// @cite https://math.stackexchange.com/questions/275529/check-if-line-intersects-with-circles-perimeter
  /// @param p1 First point defining line segment
  /// @param p2 Second point defining line segment
  /// @return Tuple with first element boolean that is true if line segment intersects obstacle, second element is distance to obstacle 
  std::pair<bool, double> ray_crossing(const Point<double> &pt1, const Point<double> &pt2) const override;


  std::vector<double> data(void) const override;
  void update_position(const Eigen::Vector2d &pos_delta) override;

private:

  /// Center of the obstacle
  Point<double> _center;

  /// Radius of the obstacle
  double _radius;


};


}