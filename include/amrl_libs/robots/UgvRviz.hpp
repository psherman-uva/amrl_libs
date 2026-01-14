/*
  @file:      UgvDisplay.hpp
  @author:    psherman
  @date       July 2023
  
  @brief Class for displaying Ugv in RViz
*/

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>

namespace amrl
{
  
class UgvRviz
{
public:
  
  UgvRviz(ros::NodeHandle &nh, 
    const std::string &frame_id, 
    const std::string &marker_ns,
    const std::string &pub_namespace,
    const double scaling = 1.0,
    const std::array<float, 3> &bdy_clr = {1.0, 0.0, 0.0});

  ~UgvRviz(void) = default;

  void update(const double x, const double y, const double theta);

private:

  // RViz visualization of robot body
  visualization_msgs::Marker _body;
  std::array<visualization_msgs::Marker, 4> _wheels;
  ros::Publisher _rviz_pub;

  // Transform for rosbot
  tf2_ros::TransformBroadcaster _br;
  geometry_msgs::TransformStamped _transform_xy;
  geometry_msgs::TransformStamped _transform_theta;
};

} // namespace amrl
