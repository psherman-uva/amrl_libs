/*
  @file:      RobotDisplay.hpp
  @author:    psherman
  @date       May 2024
  
  @brief Class for displaying robots
*/

#pragma once

#include <Eigen/Dense>
#include <string>
#include <geometry_msgs/Pose.h>

namespace amrl {
namespace util {

struct RobotSetupdata {
  RobotSetupdata(void);
  ~RobotSetupdata(void) = default;
  std::string to_string(void) const;

  std::string rbt_name;
  std::string model;
  std::string cmd_topic;
  Eigen::VectorXd vel_limits;
  Eigen::VectorXd acc_limits;
  geometry_msgs::Pose pose;
  Eigen::VectorXd position;
  double theta;

  static constexpr double kDefaultPoseZ = 0.0;
};


std::vector<RobotSetupdata> robot_setup_from_json(const std::string &json_file);

}
}