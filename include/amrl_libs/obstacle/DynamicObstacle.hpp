/*
  @file:    DynamicObstacle.hpp
  @author:  pdsherman
  @date:    July 2024

  @brief: Implementation for a dynamic circle obstacle
*/

#pragma once

#include <amrl_libs/obstacle/Obstacle.hpp>
#include <amrl_common/point/Point.hpp>

#include <Eigen/Dense>
#include <vector>

namespace amrl {
 
class DynamicObstacle : public Obstacle
{
public:

  /// Constructor
  DynamicObstacle(
    const std::vector<Point<double>> &vertices,
    uint32_t id);

  /// Constructor
  DynamicObstacle(
    const Point<double> &center,
    const double radius,
    uint32_t id);

  DynamicObstacle(
    const Eigen::Vector2d &vel,
    const Point<double> &center,
    const double radius,
    uint32_t id);

  void drive(const double dt);

  void vel_set(const Eigen::Vector2d &vel);

  Eigen::Vector2d vel_curr(void) const;

  Eigen::Vector2d position_curr(void) const;

  void position_set(const Point<double> &position);

  void id_set(uint32_t id);
  uint32_t id_get(void) const;
  
private:

  Eigen::Vector2d _anchor;
  Eigen::Vector2d _vel_cmd;
  uint32_t _id;
};


}
