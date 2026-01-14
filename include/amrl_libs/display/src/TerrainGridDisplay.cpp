/*
  @file:      TerrainGridDisplay.hpp
  @author:    psherman
  @date       June 2023
*/

#include <amrl_libs/display/TerrainGridDisplay.hpp>

#include <geometry_msgs/Point32.h>

namespace amrl {

TerrainGridDisplay::TerrainGridDisplay(ros::NodeHandle &nh,
  const std::string &topic_namespace,
  const double cell_width,
  const double cell_height,
  const std::string &frame_id)
{
  _pub = nh.advertise<nav_msgs::GridCells>(topic_namespace + "/terrain", 1, true);

  _msg.header.seq      = 0;
  _msg.header.frame_id = frame_id;
  _msg.cell_height     = cell_height;
  _msg.cell_width      = cell_width;

  _msg.cells.clear();
}

void TerrainGridDisplay::publish(void)
{
  ++_msg.header.seq;
  _msg.header.stamp = ros::Time::now();
  _pub.publish(_msg);
}

void TerrainGridDisplay::add_terrain(std::vector<Point<double>> &points)
{
  for(const auto &p : points) {
    geometry_msgs::Point pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = 0.0;
    _msg.cells.push_back(pt);
  }
}

} // namespace amrl