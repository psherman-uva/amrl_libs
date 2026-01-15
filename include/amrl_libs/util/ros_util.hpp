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

/// Setup an occupancy map using obstacles 
/// @param nh 
/// @param obs_file 
/// @param frame_id 
/// @param map 
/// @return Container of obstacles
std::vector<Obstacle> map_setup(ros::NodeHandle &nh, 
  const std::string &obs_file, 
  const std::string &frame_id, 
   std::shared_ptr<OccupancyGrid> map);

/// Logging setup actions. Calls logging service to setup data subscribing
/// If desired, will delete data table before-hand. Initializes data
/// message object to correct size.
/// @param nh ROS node handle object
/// @param data_pub ROS publisher to setup
/// @param data Data message to initialize to correct size
/// @param table_name Name of data table to log to
/// @param topic_name Name of topic data will be published
/// @param label_header List of header names for labels in data
/// @param value_header List of header names for values to log
/// @param delete_table True if table should be cleared as a first action
/// @return True is logging setup actions were all successful
bool logging_full_setup(
  ros::NodeHandle &nh,
  ros::Publisher &data_pub,
  amrl_logging::LoggingData &data,
  const std::string &table_name,
  const std::string &topic_name,
  const std::vector<std::string> &labels_header,
  const std::vector<std::string> &ints_header,
  const std::vector<std::string> &reals_header,
  bool delete_table = false);


/// From json input file, get names of all robots.
/// @param full_rbt_file Complete filepath of json input file with robot data
std::vector<std::string> robot_names_from_file(const std::string &full_rbt_file);


bool param_namespace_exists(const std::string &ns);

} // namespace util
} // namespace amrl