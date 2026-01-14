/*
  @file:      OccupancyGridDisplay.hpp
  @author:    psherman
  @date       June 2023
*/

#include <amrl_libs/display/OccupancyGridDisplay.hpp>

#include <cmath>

namespace amrl {

OccupancyGridDisplay::OccupancyGridDisplay(ros::NodeHandle &nh,
  const std::string &topic_namespace,
  const std::string &frame_id,
  const Point<double> &origin,
  const double width,
  const double height,
  const double resolution,
  const double init_prob)
{
  _pub = nh.advertise<nav_msgs::OccupancyGrid>(topic_namespace + "/grid", 1, true);

  //
  // Define a ROS message to publish to visual in RViz
  //

  // Header
  _msg.header.seq      = 0;
  _msg.header.frame_id = frame_id;

  // Meta Data
  geometry_msgs::Pose msg_origin;
  msg_origin.position.x = origin.x;
  msg_origin.position.y = origin.y;
  msg_origin.position.z = 0.0;
  msg_origin.orientation.x = 0.0;
  msg_origin.orientation.y = 0.0;
  msg_origin.orientation.z = 0.0;
  msg_origin.orientation.w = 1.0;
  

  _msg.info.map_load_time = ros::Time::now();
  _msg.info.resolution    = resolution;
  _msg.info.width         = static_cast<uint32_t>(round(width/resolution));
  _msg.info.height        = static_cast<uint32_t>(round(height/resolution));
  _msg.info.origin        = msg_origin;

  // TODO (pdsherman): Initialize data using map instead. Could have situations
  // where the map had already been updated before this object is constructed.

  // Data
  size_t n        = _msg.info.width *_msg.info.height;
  signed char val = static_cast<signed char>(100.0 * init_prob);
  _msg.data = std::vector<signed char>(n, val);
}

OccupancyGridDisplay::OccupancyGridDisplay(ros::NodeHandle &nh,
    const std::string &topic_namespace,
    const std::string &frame_id,
    const GridInfo &info,
    const double init_prob)
  : OccupancyGridDisplay(nh, topic_namespace, frame_id, info.origin, info.width, info.height, info.resolution, init_prob)
{
}

void OccupancyGridDisplay::publish(void)
{
  ++_msg.header.seq;
  _msg.header.stamp = ros::Time::now();
  _pub.publish(_msg);
}

void OccupancyGridDisplay::update_data(const uint32_t x, const uint32_t y, const double probability)
{
  const size_t idx = x + _msg.info.width*y;
  if(idx < _msg.data.size() && probability >= 0.0 && probability <= 1.0 ) {
    _msg.data[idx] = 100.0 * probability;
  }
}

void OccupancyGridDisplay::add_origin_offset(const geometry_msgs::Pose &origin_offset)
{
  _msg.info.origin.position.x += origin_offset.position.x;
  _msg.info.origin.position.y += origin_offset.position.y;
}


} // namespace amrl