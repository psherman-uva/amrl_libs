
#include <amrl_libs/lidar/LidarSimulatedObstacles.hpp>

namespace amrl {

LidarSimulatedObstacles::LidarSimulatedObstacles(
    const std::vector<Obstacle> &obstacles,
    const double range,
    const uint32_t num_beams,
    const double angle_sweep) 
: LidarSimulated(range, num_beams, angle_sweep),
  _obstacles(obstacles)
{
}

void LidarSimulatedObstacles::add_obstacle(const Obstacle &obs)
{
  _obstacles.push_back(obs);
}

const std::vector<double>& LidarSimulatedObstacles::measurement(const Point<double> &pos, const double heading)
{
  for (size_t j = 0; j < kNumBeams; ++j) {
    double theta = kAngles[j] + heading;
    Point<double> p2 = pos + _range*Point<double>(cos(theta), sin(theta));

    _scan.ranges[j]      = kDefaultMeasurementFlt;
    _dist_measurement[j] = kDefaultMeasurementDbl;
    for(const auto &obs : _obstacles) {
      if (!obs.is_inside(pos)) {
        auto crossing = obs.ray_crossing(pos, p2);

        if (crossing.first) {
          _dist_measurement[j] = std::min<double>(_dist_measurement[j], crossing.second);
          _scan.ranges[j]      = static_cast<float>(_dist_measurement[j]);
        }
      } else {
        _scan.ranges[j]      = 0.0;
        _dist_measurement[j] = 0.0;
      }
    }
  }

  return _dist_measurement;
}

bool LidarSimulatedObstacles::position_in_obstacle(const Point<double> &pos) const
{
  bool in_obs = false;

  for(const auto &obs : _obstacles) {
    if (obs.is_inside(pos)) {
      in_obs = true;
    }
  }

  return in_obs;
}

} // namespace amrl