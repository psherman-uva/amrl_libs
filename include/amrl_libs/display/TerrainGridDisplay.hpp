/*
  @file:      TerrainGridDisplay.hpp
  @author:    psherman
  @date       Sept. 2023
  
  @brief Class to display terrain grid using ROS RViz software tool
*/

#pragma once

#include <amrl_common/point/Point.hpp>

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>

namespace amrl {

class TerrainGridDisplay
{
public:

  /// Constructor
  /// @param  nh ROS nodehandle object
  /// @param  topic_namespace Name of topic namespace object will publish OccupancyGrid message
  /// @param  coords
  TerrainGridDisplay(ros::NodeHandle &nh,
    const std::string &topic_namespace,
    const double cell_width,
    const double cell_height,
    const std::string &frame_id);

  // Destuctor
  ~TerrainGridDisplay(void) = default;

  void publish(void);

  void add_terrain(std::vector<Point<double>> &points);

private:
  /// ROS publisher
  ros::Publisher _pub;

  /// ROS message for displaying grid
  nav_msgs::GridCells _msg;

};

  
} // namespace amrl