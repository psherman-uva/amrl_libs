
#pragma once

#include <amrl_libs/obstacle/ObstacleShape.hpp>


namespace amrl {

class PolygonObstacle : public ObstacleShape 
{
public:

  /// Constructor
  /// @param coordinates Initial coordinates of polygon vertices
  PolygonObstacle(const std::vector<Point<double>> &coordinates);
  
  /// Default destructor
  ~PolygonObstacle(void) = default;

  std::vector<double> data(void) const override;

  /// Check if a point is inside the obstacle defining polygon
  /// @param p Point to check if in obstacle
  /// @return True if point is inside the obstacle
  bool is_inside(const Point<double> &pt) const override;

  /// Check if a point is on the border of the of the obstacle
  /// @param pt Point to check if on border
  /// @return True if point is directly on the border of the obstacle
  bool on_border(const Point<double> &pt) const override;

  /// Compute the shortest distance from a point in space to the obstacle
  /// @note If point is inside obstacle, function will return 0
  /// @param p Point in space to get distance from
  /// @return Distance between point and obstacle
  std::pair<double, std::vector<double>> distance_from_point(const Point<double> &pt) const override;

  /// Check if a line segment crosses any side of an obstacle and
  /// returns if so, returns distance to intersection point.
  /// @param p1 First point defining line segment
  /// @param p2 Second point defining line segment
  /// @return Tuple with first element boolean that is true if line segment intersects obstacle, second element is distance to obstacle 
  std::pair<bool, double> ray_crossing(const Point<double> &p1, const Point<double> &p2) const override;

  void update_position(const Eigen::Vector2d &pos_delta) override;


private:

  /// Determines if the path from p1->p2->p3 goes in a clockwise,
  /// counter-clockwise, or co-linear direction. Compares the slope
  /// from p1->p2 and p2->p3.
  int orientation(const Point<double> &p1, const Point<double> &p2, const Point<double> &p3) const;

  /// Check if the point p is on the line segment defined by points p1 and p2.
  /// @warning Assumes points are co-linear. Check that orientation() == 0 first
  /// @param p1 First point defining line segment
  /// @param p2 Second point defining line segment
  /// @param p Point to check if on segment
  /// @return True if point 'p' is on line segment between 'p1' and 'p2'
  bool on_segment(const Point<double> &p1, const Point<double> &p2, const Point<double> &p) const;

  /// Check if the line segments defined by points [p11, p12] and [p21, p22] intersect
  bool do_intersect(const Point<double> &p11,
    const Point<double> &p12,
    const Point<double> &p21,
    const Point<double> &p22) const;

  /// Distance from a point to the line defined by two coordinates
  /// @param p1
  /// @param p2 
  /// @param coord1
  /// @param coord2
  /// @return 
  double distance_to_intersection(const Point<double> &p1, const Point<double> &p2, const Point<double> &coord1, const Point<double> &coord2) const;

  void set_min_max_values(void);

  /// Center of the obstacle
  std::vector<Point<double>> _coordinates;

  // Number of coordinates
  size_t kNumCoords;
};


}