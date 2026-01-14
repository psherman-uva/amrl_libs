
#pragma once

#include <amrl_common/point/Point.hpp>

#include <Eigen/Dense>
#include <vector>

namespace amrl {

class ObstacleShape
{
public:

  enum class ShapeType : uint8_t {
    kPolygon = 0,
    kCircle  = 1,
  };

  /// Constructor: Polygon constructor
  /// @param coordinates Points that define the polygon of the obstacle
  ObstacleShape(const ShapeType &shape_type);

  /// Default destructor
  virtual ~ObstacleShape(void) = default;

  /// @return Obstacle shape type (polygon vs. circle)
  ShapeType shape_type(void) const;

  virtual std::vector<double> data(void) const = 0;

  /// Get max & min coordinates that describe a bounding box around the obstacle
  /// @note Values are returned by reference
  /// @param min_x Minimum x-coordinate of obstacle
  /// @param max_x Maximum x-coordinate of obstacle
  /// @param min_y Minimum y-coordinate of obstacle
  /// @param max_y Maximum y-coordinate of obstacle
  void min_max_coordinates(double &min_x, double &max_x, double &min_y, double &max_y) const;

  /// Check if a point is inside the obstacle defining polygon
  /// @param p Point to check if in obstacle
  /// @return True if point is inside the obstacle
  virtual bool is_inside(const Point<double> &pt) const = 0;

  /// Check if a point is on the border of the of the obstacle
  /// @param pt Point to check if on border
  /// @return True if point is directly on the border of the obstacle
  virtual bool on_border(const Point<double> &pt) const = 0;

  /// Compute the shortest distance from a point in space to the obstacle
  /// @note If point is inside obstacle, function will return 0
  /// @param p Point in space to get distance from
  /// @return Distance between point and obstacle
  virtual std::pair<double, std::vector<double>> distance_from_point(const Point<double> &pt) const = 0;

  /// Check if a line segment crosses any side of an obstacle and
  /// returns if so, returns distance to intersection point.
  /// @param p1 First point defining line segment
  /// @param p2 Second point defining line segment
  /// @return Tuple with first element boolean that is true if line segment intersects obstacle, second element is distance to obstacle 
  virtual std::pair<bool, double> ray_crossing(const Point<double> &p1, const Point<double> &p2) const = 0;

  virtual void update_position(const Eigen::Vector2d &pos_delta) = 0;

protected:

  // Maximum & minimum coordinates of vertices. Creates a bounding box around obstacle
  double _max_X;
  double _min_X;
  double _max_Y;
  double _min_Y;

  // Some X-position to the right of all other points in polygon
  // Useful for the 'is_inside()' algorithm
  double _infinityX;

  // The 'type' of obstacle.
  ShapeType _type;

  // For testing if a float point value equals 0
  static constexpr double kEps = 1e-5;
};

} // namespace amrl
