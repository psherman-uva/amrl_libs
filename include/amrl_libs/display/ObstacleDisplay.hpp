
#pragma once

#include <amrl_libs/obstacle/Obstacle.hpp>
#include <amrl_libs/obstacle/ObstacleHelpers.hpp>
#include <amrl_common/point/Point.hpp>

#include <amrl_display/AddPolygon.h>
#include <amrl_display/AddCircle.h>
#include <amrl_display/UpdatePolygon.h>
#include <amrl_display/UpdateCircle.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

namespace amrl {

class ObstacleDisplayManager
{
public:

  ObstacleDisplayManager(ros::NodeHandle &nh, const std::string &figure_label = "");

  ~ObstacleDisplayManager(void) = default;

  void set_figure_label(const std::string &figure_label);

  void add_from_setup_data(const std::vector<util::ObstacleSetupData> &setup_data);

  void add_from_json_file(const std::string &json_obs_file);

  void add_obstacle(std::shared_ptr<Obstacle> obs);

  void add_obstacle(
    std::shared_ptr<Obstacle> obs,
    const std::string &obs_name,
    const std::string &color,
    const double alpha,
    const double zorder);

  void update_circle_obs(
    const std::string &obs_name,
    const Point<double> &pos,
    const std::string &color = "",
    const double alpha       = 0.0);

  void update_polygon_obs(
    const std::string &obs_name,
    const std::vector<Point<double>> &vertices,
    const std::string &color = "",
    const double alpha       = 0.0);

  void remove_obs(const std::string &obs_name);

private:
  ros::NodeHandle _nh;

  ros::ServiceClient _add_poly_client;
  ros::ServiceClient _add_circle_client;
  ros::ServiceClient _del_client;
  
  amrl_display::AddPolygon _poly_srv;
  amrl_display::AddCircle _circle_srv;
  
  const std::string _node_name;
  std::string _fig_label;

  std::map<std::string, std::pair<ros::Publisher, amrl_display::UpdatePolygon>> _poly_pub;
  std::map<std::string, std::pair<ros::Publisher, amrl_display::UpdateCircle>> _circle_pub;

  static std::map<Obstacle::ObstacleType, std::string> kColorMap;

  std::set<std::string> _obs_names;

  uint32_t _cnt_obs;
  uint32_t _cnt_dyn;
  uint32_t _cnt_misc;

  static const std::string kDefaultObsName;
  static const std::string kDefaultDynName;
  static const std::string kDefaultMiscName;
};

}