/*
  @file:      TaskDisplay.hpp
  @author:    psherman
  @date       May 2024
  
  @brief Class for displaying tasks
*/

#pragma once

#include <amrl_common/point/Point.hpp>
#include <amrl_display/UpdatePolygon.h>

#include <ros/ros.h>

namespace amrl {

class TaskDisplayManager
{
public:

  /// Constructor
  /// @param nh ROS nodehandle object
  /// @param figure_label Name of the figure to display. Okay if empty
  TaskDisplayManager(ros::NodeHandle &nh, const std::string &figure_label = "");

  /// Destructor
  ~TaskDisplayManager(void) = default;

  // ----------------------------------- //
  // --    Public Class Methods       -- //
  // ----------------------------------- //

  void set_figure_label(const std::string &figure_label);

  void add_from_json_file(const std::string &json_task_file);

  void initialize_single(
    const std::string &task_name,
    const std::string &topic_name,
    const Point<double> &pos,
    const double radius,
    const std::string &color,
    const double alpha = 1.0);

  void update_task(
    const std::string &task_name,
    const Point<double> &pos,
    const std::string &color,
    const double alpha = 1.0);

private:

  // ----------------------------------- //
  // --    Private Class Methods      -- //
  // ----------------------------------- //

  std::vector<geometry_msgs::Point> polygon_vertices(
    const Point<double> &pos, double radius);

  // ----------------------------------- //
  // --     Class Member Variables    -- //
  // ----------------------------------- //

  /// ROS Node Handle object
  ros::NodeHandle _nh;

  /// Service client to add polygon to figure
  ros::ServiceClient _add_poly_client;

  /// Service client to delte an element from figure
  ros::ServiceClient _del_client;

  /// Label of figure. Needed for applications with mutiple figures.
  std::string _fig_label;

  /// Names of all the displayed tasks currently being manged by object
  std::set<std::string> _task_names;

  /// Map from name of a task to the ROS objects need to send updates
  std::map<std::string, std::pair<ros::Publisher, amrl_display::UpdatePolygon>> _task_pubs;

  // ----------------------------------- //
  // --       Class Constants         -- //
  // ----------------------------------- //

  static const std::string kDefaultColor;
  static constexpr uint8_t kNumSides     = 6;
  static constexpr double kZorder        = 2.0;
  static constexpr double kDefaultRadius = 1.0;
  static constexpr double kDefaultAlpha  = 1.0;

  std::vector<double> kCosAngles;
  std::vector<double> kSinAngles;
};


} // namespace amrl