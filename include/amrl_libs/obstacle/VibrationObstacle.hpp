/*
  @file:    VibrationObstacle.hpp
  @author:  pdsherman
  @date:    July 2024

  @brief: Implementation for a vibration terrain type obstacle
*/

#pragma once

#include <amrl_libs/obstacle/Obstacle.hpp>
#include <amrl_common/point/Point.hpp>

#include <Eigen/Dense>
#include <vector>

namespace amrl {

class VibrationObstacle : public Obstacle
{
public:

  /// Constructor
  VibrationObstacle(
    const std::vector<Point<double>> &vertices,
    const double roughness);

  /// Constructor
  VibrationObstacle(
    const Point<double> &center,
    const double radius,
    const double roughness);

  double roughness_get(void) const;
  void rougness_set(const double roughness);

private:

  double _roughness;

};


} // namespace amrl