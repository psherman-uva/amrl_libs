#include <amrl_libs/robots/RobotHelpers.hpp>
#include <amrl_libs/util/ros_util.hpp>
#include <amrl_common/util/util.hpp>

#include <nlohmann/json.hpp>
#include <ros/ros.h>
#include <fstream>

namespace amrl {
namespace util {


RobotSetupdata::RobotSetupdata(void) : 
  rbt_name(""),
  model("point"),
  cmd_topic(""),
  vel_limits(Eigen::Vector2d({-10.0,   10.0})),
  acc_limits(Eigen::Vector2d({-100.0, 100.0})),
  position(Eigen::Vector2d({1.0, 1.0})),
  theta(0.0)
{
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = kDefaultPoseZ;
  pose.orientation = util::quaternion_from_yaw(theta);
}

std::string RobotSetupdata::to_string(void) const
{
  std::stringstream ss;

  ss << "Robot: " << rbt_name << "\n";
  ss << "\t- Model: " << model << "\n";
  ss << "\t- Cmd Topic: " << cmd_topic << "\n";
  ss << "\t- Vel Limits: " << util::container_to_string(vel_limits) << "\n";
  ss << "\t- Accel Limits: " << util::container_to_string(acc_limits) << "\n";
  ss << "\t- Pose: " << util::pose_str(pose) << "\n";
  ss << "\t- Position: " << util::container_to_string(position) << "\n";
  ss << "\t- Theta: " << theta << "\n";

  return ss.str();
}

std::vector<RobotSetupdata> robot_setup_from_json(const std::string &json_file)
{
  std::vector<RobotSetupdata> rbt_data;

  std::ifstream f(json_file);
  if (f.is_open()) {
    ROS_INFO("Robots from file: %s", json_file.c_str());

    nlohmann::json data = nlohmann::json::parse(f)["robots"];
    for(nlohmann::json::iterator it = data.begin(); it != data.end(); ++it) {
      RobotSetupdata setup;

      setup.rbt_name  = it.value()["name"];
      setup.model     = it.value()["model"];
      setup.cmd_topic = it.value()["topic"];
      
      if(it->contains("theta")) { setup.theta = it.value()["theta"]; }
      if(it->contains("position")) {
        setup.position[0] = it.value()["position"][0];
        setup.position[1] = it.value()["position"][1];
      }

      setup.pose.position.x = setup.position[0];
      setup.pose.position.y = setup.position[1];;
      setup.pose.orientation = util::quaternion_from_yaw(setup.theta);

      rbt_data.push_back(setup);
    }
  } else {
    ROS_WARN("Not able to open simulated robot file: %s", json_file.c_str());
  }

  return rbt_data;
}


}
}