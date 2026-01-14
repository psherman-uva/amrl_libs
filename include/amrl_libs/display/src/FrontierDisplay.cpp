/*
    @file:      FrontierDisplay.cpp
    @author:    psherman
    @date       July 2023
*/

#include <amrl_libs/display/FrontierDisplay.hpp>


namespace amrl {

FrontierDisplay::FrontierDisplay(ros::NodeHandle &nh, 
    const std::string &topic_namespace,
    const std::string &frame_id, 
    const double resolution, 
    const double scale)
: kOffset(resolution/2.0)
{
  _pub = nh.advertise<visualization_msgs::Marker>(topic_namespace + "/frontier_marker", 10);

  _points_msg.header.frame_id = frame_id;
  _points_msg.header.seq      = 0;
  _points_msg.id              = 0;
  _points_msg.ns              = "frontier";
  _points_msg.action          = visualization_msgs::Marker::ADD;
  _points_msg.type            = visualization_msgs::Marker::POINTS;
  
  _points_msg.pose.orientation.w = 1.0;

  _points_msg.scale.x = scale;
  _points_msg.scale.y = scale;

  _points_msg.color.r = 1.0f;
  _points_msg.color.g = 0.0f;
  _points_msg.color.b = 0.0f;
  _points_msg.color.a = 1.0;
}

void FrontierDisplay::set_points(const std::set<Point<double>> &pts)
{
  _points_msg.points.clear();

  for (const auto &pt : pts) {
    geometry_msgs::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.1;

    _points_msg.points.push_back(p);
  }
}

void FrontierDisplay::publish(void)
{
  _points_msg.header.seq  += 1;
  _points_msg.header.stamp = ros::Time::now();
  _pub.publish(_points_msg);
}


} // namespace amrl