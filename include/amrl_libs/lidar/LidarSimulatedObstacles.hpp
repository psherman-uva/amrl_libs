/*
  @file:      LidarSimulatedObstacles.hpp
  @author:    psherman
  @date       July 2023
  
  @brief Simulating Lidar readings using Obstacle objects
*/

#pragma once

#include <amrl_libs/lidar/LidarSimulated.hpp>
#include <amrl_libs/obstacle/Obstacle.hpp>

namespace amrl {

class LidarSimulatedObstacles : public LidarSimulated
{
public:

  /// Constructor
  /// @param obstacles Obstacles in the environment
  /// @param range Maxmimum sensor range of each lidar beam
  /// @param num_beams Number of beams that make up full lidar sweep
  /// @param angle_sweep The total angle of the lidar field of view (centered at 0)
  LidarSimulatedObstacles(
      const std::vector<Obstacle> &obstacles,
      const double range,
      const uint32_t num_beams = 720,
      const double angle_sweep  = 2*M_PI);

  /// Default destructor
  ~LidarSimulatedObstacles(void) = default;

  /// Get a full measurement sweep from the lidar given a position in the environment
  /// @param pos Position in the world sensor is placed
  /// @param heading Rotation of the sensor
  /// @return Vector containing distance measuremet from each lidar beam
  const std::vector<double>& measurement(const Point<double> &pos, const double heading) override final;

  /// Check if the sensor thinks the position is inside an obstacle
  /// @param pos Position in the world of the sensor
  /// @return True if simualtion implementation thinks the lidar is inside an obstacle
  bool position_in_obstacle(const Point<double> &pos) const override final;

  /// Adds a new obstacle to the environment
  void add_obstacle(const Obstacle &obs);
  
private:

  /// Container of obstacles in the environment
  std::vector<Obstacle> _obstacles;
};

}
