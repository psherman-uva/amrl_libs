
#include <amrl_libs/lidar/LidarSimulatedMap.hpp>
#include <amrl_common/util/util.hpp>
#include <amrl_common/util/bresenham_line.hpp>

namespace amrl {

LidarSimulatedMap::LidarSimulatedMap(
  std::shared_ptr<OccupancyGrid> map,
  const double range,
  const uint32_t num_beams,
  const double angle_sweep)
: LidarSimulated(range, num_beams, angle_sweep),
  _map(std::move(map))
{
}

const std::vector<double>& LidarSimulatedMap::measurement(const Point<double> &pos, const double heading)
{
  Point<uint32_t> cntr = _map->point_to_cell(pos);
  
  for(size_t i = 0; i < kNumBeams; ++i) {
    double theta         = kAngles[i] + heading;
    Point<double> end_pt = pos + _range*Point<double>(cos(theta), sin(theta));;

    Point<uint32_t> end_cell = _map->point_to_cell(end_pt);
    auto line = util::bresenham_line(cntr, end_cell);

    _scan.ranges[i]      = kDefaultMeasurementFlt;
    _dist_measurement[i] = kDefaultMeasurementDbl;
    for (const auto &cell : line) {
      if(_map->cell_is_occupied(cell)) {
        Point<double> obs_pt = _map->cell_to_point(cell);
        double dist = util::distance(pos, obs_pt);
        _dist_measurement[i] = dist;
        _scan.ranges[i]      = static_cast<float>(dist);
        break;
      }
    }
  }

  return _dist_measurement;
}

bool LidarSimulatedMap::position_in_obstacle(const Point<double> &pos) const
{
  return _map->point_is_occupied(pos);
}

void LidarSimulatedMap::change_map(std::shared_ptr<OccupancyGrid> new_map)
{
  if(new_map) { _map = new_map; }
}

} // namespace amrl