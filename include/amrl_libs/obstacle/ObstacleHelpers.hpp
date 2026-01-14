/*
File:   ros_util.hpp
Author: pdsherman-uva
Date:   June 2023

Description: Collection of simple useful utility functions useful
  for ROS-based projects.
*/
#pragma once

#include <amrl_libs/obstacle/Obstacle.hpp>
#include <amrl_libs/obstacle/ObstacleShape.hpp>
#include <amrl_common/point/Point.hpp>

#include <ros/ros.h>

namespace amrl {
namespace util {

// Convenvience struct to hold all possible obstacle data from file
struct ObstacleSetupData 
{
  Obstacle::ObstacleType type;
  ObstacleShape::ShapeType shape;

  std::string label;

  bool pos_set    = false;
  bool detectable = false;
  
  double x     = 0.0;
  double y     = 0.0;
  double theta = 0.0;

  // For Polygon shape obstacle
  std::vector<Point<double>> vertices;

  // For Circle shape obstacle
  double radius = 0.0;

  // Color String for display puposes
  std::string color;

  // Alpha value for display purposes
  double alpha = 1.0;

  // Extra info
  std::map<std::string, double> extra_info;
};

std::shared_ptr<Obstacle> obs_from_setup_data(const ObstacleSetupData &obs_data);
std::vector<std::shared_ptr<Obstacle>> obstacles_from_file(const std::string &json_file);

Obstacle solid_obs_from_setup_data(const ObstacleSetupData &obs_data);

/// Get obstacles from JSON file with all coordinates in world frame
/// @param json_file File containing obstacles to check
/// @return Container of obstacle objects
std::vector<Obstacle> solid_obstacles_from_file(const std::string &json_file);

/// Get coordinates of the obstacles in the world frame using a nominal 
/// configuration and Vicon positions/angles to transform coordinates
/// @param nh ROS node handle object for subscribing to vicon
/// @param json_file File containing obstacles to check
/// @return Container of obstacle objects
std::vector<Obstacle> obstacles_with_vicon(
  ros::NodeHandle &nh, 
  const std::string &json_file);
std::map<std::string, std::vector<double>> obstacle_vicon_data(ros::NodeHandle &nh, const std::string &filename);

/// Parse a JSON file to get obstacle names and nominal coordinates
/// @param json_file Name of JSON file with obstacle information
/// @return Container of obstacles names and coordinates
std::vector<ObstacleSetupData> obstacle_data_from_file(const std::string &json_file);

/// @brief Parse a JSON file representing obstacles at point features in the environment
/// @param json_file Name of JSON file with obstacle information
/// @return Container of features point coordinates
std::vector<Point<double>> features_from_obs_file(const std::string &json_file);


bool topic_is_advertised(const std::string &topic);

} // namespace util
} // namespace amrl