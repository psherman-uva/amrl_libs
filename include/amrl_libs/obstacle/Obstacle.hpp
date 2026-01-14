/*
    File: Obstacle.hpp
    Authors: pdsherman
    Date: Fall 2022

    Description: Implementation for an 2D obstacle object defined as a polygon
*/

#pragma once

#include <amrl_libs/obstacle/CircleObstacle.hpp>
#include <amrl_libs/obstacle/PolygonObstacle.hpp>
#include <amrl_common/point/Point.hpp>

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace amrl {

class Obstacle
{
public:

  enum class ObstacleType : uint8_t {
    kSolid     = 0,
    kDynamic   = 1,
    kVibration = 2,
    kDoor      = 3,
  };

  Obstacle(const ObstacleType type, const std::vector<Point<double>> &coordinates);

  Obstacle(const ObstacleType type, const Point<double> center, const double radius);

  /// Constructor: Polygon constructor
  /// @param coordinates Points that define the polygon of the obstacle
  Obstacle(const std::vector<Point<double>> &coordinates);

  /// Constructor: Circle constructor
  /// @param center
  /// @param radius
  Obstacle(const Point<double> center, const double radius);

  /// Default destructor
  ~Obstacle(void) = default;

  /// Get coordinates that define the obstacle polygon
  /// @return List of coordinates
  std::vector<Point<double>> coordinates(void) const;

  /// Get max & min coordinates that describe a bounding box around the obstacle
  /// @note Values are returned by reference
  /// @param min_x Minimum x-coordinate of obstacle
  /// @param max_x Maximum x-coordinate of obstacle
  /// @param min_y Minimum y-coordinate of obstacle
  /// @param max_y Maximum y-coordinate of obstacle
  void min_max_coordinates(double &min_x, double &max_x, double &min_y, double &max_y) const;

  /// Check if a point is inside the obstacle
  /// @param p Point to check if in obstacle
  /// @return True if point is inside the obstacle
  bool is_inside(const Point<double> &pt) const;

  /// Check if a point is on the border of the of the obstacle
  /// @param pt Point to check
  /// @return True if point is directly on the border of the obstacle
  bool on_border(const Point<double> &pt) const;

  /// Check if a line segment crosses any side of an obstacle and
  /// returns if so, returns distance to intersection point.
  /// @param p1 First point defining line segment
  /// @param p2 Second point defining line segment
  /// @return Tuple with first element boolean that is true if line segment intersects obstacle, second element is distance to obstacle 
  std::pair<bool, double> ray_crossing(const Point<double> &pt1, const Point<double> &pt2) const;

  /// Compute the shortest distance from a point in space to the obstacle
  /// @note If point is inside obstacle, function will return 0
  /// @param p Point in space to get distance from
  /// @return Distance between point and obstacle
  std::pair<double, std::vector<double>> distance_from_point(const Point<double> &pt) const;

  /// Get the type of the obstacle
  /// @return Obstacle type
  ObstacleType type_get(void) const;

  ObstacleShape::ShapeType shape_get(void) const;

  std::vector<double> data_get(void) const;

protected:

  void update_position(const Eigen::Vector2d &pos_delta);

  // Implementation to obstacle shape (polygon or circle)
  std::shared_ptr<ObstacleShape> _pimpl; 

  // The 'type' of obstacle.
  ObstacleType _type;
};

} // namespace amrl
