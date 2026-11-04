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
    const std::shared_ptr<Grid> grid)
{
  common_setup(nh, topic_namespace, frame_id, origin, width, height, resolution);

  // Initialize data using current data map
  size_t n  = _msg.info.width *_msg.info.height;
  _msg.data = std::vector<int8_t>(n);
  
  for(size_t i = 0; i < n; ++i) {
    double prob  = grid->value_from_index(i);
    prob = 100.0 - 100.0/(1+exp(prob));
    int8_t val   = static_cast<int8_t>(prob);
    _msg.data[i] = val;
  }
}

OccupancyGridDisplay::OccupancyGridDisplay(ros::NodeHandle &nh,
    const std::string &topic_namespace,
    const std::string &frame_id,
    const GridInfo &info,
    const std::shared_ptr<Grid> grid) :
  OccupancyGridDisplay(nh, topic_namespace, frame_id, info.origin, info.width, info.height, info.resolution, grid)
{
}

OccupancyGridDisplay::OccupancyGridDisplay(ros::NodeHandle &nh,
  const std::string &topic_namespace,
  const std::string &frame_id,
  const Point<double> &origin,
  const double width,
  const double height,
  const double resolution,
  const double init_prob)
{
  common_setup(nh, topic_namespace, frame_id, origin, width, height, resolution);

  // Data
  size_t n   = _msg.info.width *_msg.info.height;
  int8_t val = static_cast<int8_t>(100.0 * init_prob);
  _msg.data  = std::vector<int8_t>(n, val);
}

OccupancyGridDisplay::OccupancyGridDisplay(ros::NodeHandle &nh,
    const std::string &topic_namespace,
    const std::string &frame_id,
    const GridInfo &info,
    const double init_prob) :
  OccupancyGridDisplay(nh, topic_namespace, frame_id, info.origin, info.width, info.height, info.resolution, init_prob)
{
}

void OccupancyGridDisplay::common_setup(ros::NodeHandle &nh,
    const std::string &topic_namespace,
    const std::string &frame_id,
    const Point<double> &origin,
    const double width,
    const double height,
    const double resolution)
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