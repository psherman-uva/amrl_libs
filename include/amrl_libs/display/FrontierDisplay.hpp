/*
  @file:      FrontierDisplay.hpp
  @author:    psherman
  @date       July 2023
  
  @brief Class to display frontier points using ROS RViz software tool
*/

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <amrl_common/point/Point.hpp>

namespace amrl {

class FrontierDisplay
{
public:

  FrontierDisplay(ros::NodeHandle &nh, 
    const std::string &topic_namespace,
    const std::string &frame_id, 
    const double resolution, 
    const double scale = 0.1);

  ~FrontierDisplay(void) = default;

  void set_points(const std::set<Point<double>> &pts);

  void publish(void);

private:

  /// ROS publisher
  ros::Publisher _pub;

  /// Message for frontier point
  visualization_msgs::Marker _points_msg;

  /// Offset
  const double kOffset;
};

}