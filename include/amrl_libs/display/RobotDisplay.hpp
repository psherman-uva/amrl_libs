/*
  @file:      RobotDisplay.hpp
  @author:    psherman
  @date       May 2024
  
  @brief Class for displaying robots
*/

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>

namespace amrl
{

class RobotDisplayManager
{
public:
  
  /// Contstructor
  /// @param nh ROS nodehandle object
  /// @param figure_label Name of the figure to display. Okay if empty
  RobotDisplayManager(ros::NodeHandle &nh, const std::string &figure_label = "");

  /// Destructor
  ~RobotDisplayManager(void) = default;

  // ----------------------------------- //
  // --    Public Class Methods       -- //
  // ----------------------------------- //

  void set_figure_label(const std::string &figure_label);

  void initialize_single(
    const std::string &robot_name,
    const std::string &topic,
    const geometry_msgs::Pose &robot_pose,
    const uint8_t robot_display_type,
    const std::string &color,
    const double alpha,
    const double zorder,
    const double scaling,
    const std::vector<double> &data);

  void initialize_from_json_file(const std::string &filename);

  void update(const std::string &rbt_name, 
    const geometry_msgs::Point &rbt_pt,
    const geometry_msgs::Quaternion &rbt_orientation);

  void remove_robot(const std::string &rbt_name);

private:


  // ----------------------------------- //
  // --     Class Member Variables    -- //
  // ----------------------------------- //

  /// ROS Node Handle object
  ros::NodeHandle _nh;

  /// Service client to add a robot to figure
  ros::ServiceClient _add_robot_client;

  /// Service client to delte an element from figure
  ros::ServiceClient _del_client;

  /// Label of figure. Needed for applications with mutiple figures.
  std::string _fig_label;

  /// Names of all the displayed tasks currently being manged by object
  std::set<std::string> _robot_names;

  /// Map from name of a robot to the ROS objects need to send updates
  std::map<std::string, std::pair<ros::Publisher, geometry_msgs::Pose>> _rbt_pubs;

  // ----------------------------------- //
  // --       Class Constants         -- //
  // ----------------------------------- //

  static const std::string kDefaultRbtColor;
  static constexpr double kDefaultScaling = 1.0;
  static constexpr double kDefaultTheta   = 0.0;
  static constexpr double kDefaultAlpha   = 1.0;
  static constexpr double kDefaultZOrder  = 5.0;
};

} // namespace amrl