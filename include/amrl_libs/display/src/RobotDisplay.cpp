
#include <amrl_libs/display/RobotDisplay.hpp>
#include <amrl_common/util/util.hpp>

#include <amrl_display/DeleteElement.h>
#include <amrl_display/AddRobot.h>

#include <nlohmann/json.hpp>
#include <geometry_msgs/Pose.h>

#include <fstream>


namespace amrl {

const std::string RobotDisplayManager::kDefaultRbtColor("#0000cc");

RobotDisplayManager::RobotDisplayManager(ros::NodeHandle &nh, const std::string &figure_label) :
  _nh(nh),
  _fig_label(figure_label)
{
  _add_robot_client = _nh.serviceClient<amrl_display::AddRobot>("/display/add_robot");
  _del_client      = _nh.serviceClient<amrl_display::DeleteElement>("/display/delete_element");
}

void RobotDisplayManager::set_figure_label(const std::string &figure_label)
{
  _fig_label = figure_label;
}

void RobotDisplayManager::initialize_from_json_file(const std::string &filename)
{
  using json = nlohmann::json;

  std::ifstream f(filename);
  if (f.is_open()) {
    json data = json::parse(f);
    json robots = data["robots"];

    for(json::iterator it = robots.begin(); it != robots.end(); ++it) {
      geometry_msgs::Pose pose;
      std::vector<double> dimensions;
      std::string topic    = ""; 
      uint8_t display_type = amrl_display::AddRobot::Request::TYPE_POINT;
      double scaling       = kDefaultScaling;
      double theta         = kDefaultTheta;
      double alpha         = kDefaultAlpha;
      double zorder        = kDefaultZOrder;
      std::string color    = kDefaultRbtColor;

      if(it->contains("topic")) { topic = it.value()["topic"]; }
      if(it->contains("theta")) { theta = it.value()["theta"]; }

      pose.position.x = it.value()["position"][0];
      pose.position.y = it.value()["position"][1];
      util::yaw_to_quaterinion(theta, pose.orientation);

      if (it->contains("display")) {
        json config = it->at("display");  
          if(config.contains("type"))    { display_type = config["type"]; }
          if(config.contains("scaling")) { scaling = config["scaling"]; }
          if(config.contains("color"))   { color   = config["color"]; }
          if(config.contains("alpha"))   { alpha   = config["alpha"];   }
          if(config.contains("zorder"))  { zorder  = config["zorder"];  }
          if(config.contains("dimensions")) {
            dimensions = (config.find("dimensions"))->get<std::vector<double>>();
          }
      } else {
        std::cout << "oh no" << std::endl;
      }

      initialize_single(it.value()["name"],
        topic, pose, display_type,
        color, alpha, zorder, scaling, dimensions);
    }

  } else {
    ROS_WARN("Failed to open robot JSON file: %s", filename.c_str());
  }
}

void RobotDisplayManager::initialize_single(
  const std::string &robot_name,
  const std::string &topic,
  const geometry_msgs::Pose &robot_pose,
  const uint8_t robot_display_type,
  const std::string &color,
  const double alpha,
  const double zorder,
  const double scaling,
  const std::vector<double> &data)
{
  if(_robot_names.find(robot_name) == _robot_names.end()) {
    if(_add_robot_client.waitForExistence(ros::Duration(2.0))) {
      amrl_display::AddRobot srv;
      srv.request.header.fig_label = _fig_label;
      srv.request.header.name      = robot_name;
      srv.request.header.topic     = topic;

      srv.request.type       = robot_display_type;
      srv.request.pose       = robot_pose;
      srv.request.dimensions = data;
      srv.request.scaling    = scaling;
      srv.request.color      = color;
      srv.request.alpha      = alpha;
      srv.request.zorder     = zorder;

      if (_add_robot_client.call(srv)) {
        _robot_names.insert(robot_name);

        if(!topic.empty()) {
          _rbt_pubs[robot_name] = std::pair<ros::Publisher, geometry_msgs::Pose>();
          _rbt_pubs[robot_name].first = _nh.advertise<geometry_msgs::Pose>(topic, 1);

          _rbt_pubs[robot_name].second = srv.request.pose;
        }
      } else {
        ROS_WARN("Call to Add Robot Service failed");
      }
    } else {
      ROS_WARN("Add robot display client not ready");
    }
  } else {
    ROS_WARN("Robot '%s' already added to display", robot_name.c_str());
  }
}

void RobotDisplayManager::update(const std::string &rbt_name, 
  const geometry_msgs::Point &rbt_pt,
  const geometry_msgs::Quaternion &rbt_orientation)
{
  if(_rbt_pubs.find(rbt_name) != _rbt_pubs.end()) {
    _rbt_pubs[rbt_name].second.position    = rbt_pt;
    _rbt_pubs[rbt_name].second.orientation = rbt_orientation;

    _rbt_pubs[rbt_name].first.publish(_rbt_pubs[rbt_name].second);
  }
}

void RobotDisplayManager::remove_robot(const std::string &rbt_name)
{

}


}