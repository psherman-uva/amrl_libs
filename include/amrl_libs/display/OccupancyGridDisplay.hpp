/*
  @file:      OccupancyGridDisplay.hpp
  @author:    psherman
  @date       June 2023
  
  @brief Class to display occupancy grid using ROS RViz software tool
*/

#pragma once

#include <amrl_libs/mapping/GridInfo.hpp>
#include <amrl_common/point/Point.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>

namespace amrl {

class OccupancyGridDisplay 
{
public:

  /// Constructor
  /// @param  nh ROS nodehandle object
  /// @param  topic_namespace Name of topic object will publish OccupancyGrid message
  /// @param  frame_id Name of parent frame for grid
  /// @param  origin Position (in world frame) of the [0,0] grid cell
  /// @param  width Number of cells along width of grid
  /// @param  height Numner of cells along height of grid
  /// @param  resolution Length of each grid cell side
  /// @param  init_prob Initial probability [0.0, 1.0] of each cell
  OccupancyGridDisplay(ros::NodeHandle &nh,
    const std::string &topic_namespace,
    const std::string &frame_id,
    const Point<double> &origin,
    const double width,
    const double height,
    const double resolution,
    const double init_prob = 0.5);
  
  /// Constructor
  /// @param  nh ROS nodehandle object
  /// @param  topic_namespace Name of topic object will publish OccupancyGrid message
  /// @param  frame_id Name of parent frame for grid
  /// @param  info
  /// @param  init_prob Initial probability [0.0, 1.0] of each cell
  OccupancyGridDisplay(ros::NodeHandle &nh,
    const std::string &topic_namespace,
    const std::string &frame_id,
    const GridInfo &info,
    const double init_prob = 0.5);

  // Destuctor
  ~OccupancyGridDisplay(void) = default;

  /// Add an offset to the origin before publishing
  /// @param origin_offset 
  void add_origin_offset(const geometry_msgs::Pose &origin_offset);

  /// Publish current OccupancyGrid message
  void publish(void);

  /// Update probability value of cell in the occupancy grid
  /// @param  x X-index of cell in grid to update
  /// @param  y Y-index of cell in grid to update
  /// @param  probability Probability [0.0, 1.0] to update cell value to
  void update_data(const uint32_t x, const uint32_t y, const double probability);

private:
  /// ROS publisher
  ros::Publisher _pub;

  /// ROS message for displaying grid
  nav_msgs::OccupancyGrid _msg;
  
};

} // namespace amrl