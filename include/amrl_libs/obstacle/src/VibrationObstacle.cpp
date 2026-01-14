
#include <amrl_libs/obstacle/VibrationObstacle.hpp>

#include <amrl_common/util/util.hpp>

namespace amrl {

VibrationObstacle::VibrationObstacle(
  const std::vector<Point<double>> &vertices,
  const double roughness)
  : Obstacle(ObstacleType::kVibration, vertices),
  _roughness(roughness)
{
}

VibrationObstacle::VibrationObstacle(
  const Point<double> &center,
  const double radius,
  const double roughness)
  : Obstacle(ObstacleType::kVibration, center, radius),
  _roughness(roughness)
{
}

double VibrationObstacle::roughness_get(void) const
{
  return _roughness;
}

void VibrationObstacle::rougness_set(const double roughness)
{
  _roughness = roughness;
}

}
