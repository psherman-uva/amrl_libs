/*
  @file:      LidarSimulatedMap.hpp
  @author:    psherman
  @date       July 2023
  
  @brief Simulating Lidar readings using Occupancy Map
*/

#pragma once

#include <amrl_libs/lidar/LidarSimulated.hpp>
#include <amrl_libs/mapping/OccupancyGrid.hpp>

namespace amrl {

class LidarSimulatedMap : public LidarSimulated 
{
public:

  /// Constructor
  /// @param map Occupancy map to use as environment source of truth
  /// @param range Maxmimum sensor range of each lidar beam
  /// @param num_beams Number of beams that make up full lidar sweep
  /// @param angle_sweep The total angle of the lidar field of view (centered at 0)
  LidarSimulatedMap(
    std::shared_ptr<OccupancyGrid> map,
    const double range,
    const uint32_t num_beams = 720,
    const double angle_sweep  = 2*M_PI);

  /// Default destructor
  ~LidarSimulatedMap(void) = default;

  /// Get a full measurement sweep from the lidar given a position in the environment
  /// @param pos Position in the world sensor is placed
  /// @param heading Rotation of the sensor
  /// @return Vector containing distance measuremet from each lidar beam
  const std::vector<double>& measurement(const Point<double> &pos, const double heading) override final;

  /// Check if the sensor thinks the position is inside an obstacle
  /// @param pos Position in the world of the sensor
  /// @return True if simualtion implementation thinks the lidar is inside an obstacle
  bool position_in_obstacle(const Point<double> &pos) const override final;

  /// If desired, use a different map object as environment source of truth
  /// @param new_map New map object to use as environment source of truth
  void change_map(std::shared_ptr<OccupancyGrid> new_map);

private:

  /// Occupancy map describing enviroment
  std::shared_ptr<OccupancyGrid> _map;

};

} // namespace amrl
