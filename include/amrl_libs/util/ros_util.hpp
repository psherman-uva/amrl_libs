/*
File:   ros_util.hpp
Author: pdsherman-uva
Date:   June 2023

Description: Collection of simple useful utility functions useful
  for ROS-based projects.
*/
#pragma once

#include <amrl_libs/lidar/LidarSimulated.hpp>
#include <amrl_libs/mapping/OccupancyGrid.hpp>
#include <amrl_libs/obstacle/Obstacle.hpp>

#include <amrl_common/point/Point.hpp>
#include <amrl_logging/LoggingData.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

namespace amrl {
namespace util {


double vel_twist_norm(const geometry_msgs::Twist &vel);

/// Reset the robot odom pose to zeros
/// @param  nh ROS node handle object
/// @param robot_namespace Namespace to specify which robot
void reset_pose(ros::NodeHandle &nh, const std::string &robot_namespace);

geometry_msgs::Quaternion quaternion_from_yaw(const double theta);

bool obstacle_lidar_in_view( 
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<Obstacle> obs,
  const Point<double> &pos,
  const double &theta
);

std::vector<std::vector<uint32_t>> lidar_perceptual_range(
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &pos,
  const double theta);

std::vector<std::vector<uint32_t>> lidar_measurement_range(
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &pos,
  const double theta);

std::set<uint32_t> lidar_cells_blocked( 
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &pos,
  const double &theta
);

std::set<uint32_t> lidar_cells_full_range( 
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &pos,
  const double &theta
);

std::set<uint32_t> lidar_cells_seen( 
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &pos,
  const double &theta
);

void map_update_from_lidar_scan(
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &rbt_pos,
  const double rbt_heading);

void map_update_from_lidar_ray(
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &rbt_pos,
  const double theta,
  const double dist);

/// From json input file, get names of all robots.
/// @param full_rbt_file Complete filepath of json input file with robot data
std::vector<std::string> robot_names_from_file(const std::string &full_rbt_file);


bool topic_is_advertised(const std::string &topic);

bool param_namespace_exists(const std::string &ns);

} // namespace util
} // namespace amrl