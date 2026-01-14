/*
  @file:      Ugv.hpp
  @author:    psherman
  @date       July 2023
  
  @brief Base class for simulating robots
*/

#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>

namespace amrl {

class RobotBase
{
public:
  
  /// Constructor
  /// @param pose0 Starting pose for the robot
  RobotBase(const geometry_msgs::Pose &pose0);

  /// Destructor
  virtual ~RobotBase(void) = default;

  /// Manually set the current pose of the robot
  /// @param pose New pose to set for robot
  void pose_set(const geometry_msgs::Pose &pose);

  /// @return Current pose of the robot
  geometry_msgs::Pose pose(void) const;

  /// Manually set the current velocity of the robot
  /// @param vel New velocity to set for robot
  void velocity_set(const geometry_msgs::Twist &vel);

  /// @return Current velocity of the robot
  geometry_msgs::Twist velocity(void) const;

  /// Set the input command to run on subsequent 'drives'
  void set_input(const Eigen::VectorXd &u);

  /// Drive using previously set command input
  /// @param u Commanded input for robot
  virtual void drive(const Eigen::VectorXd &u);

  /// Set command and drive robot on the same step
  virtual void drive(void) = 0;

protected:

  double theta_from_orientation(void) const;

  /// Pose of the robot
  geometry_msgs::Pose _pose;

  /// Velocity of the robot
  geometry_msgs::Twist _velocity;

  /// Saved command to robot to run on next drive command.
  Eigen::VectorXd _u;

};

} // namespace amrl