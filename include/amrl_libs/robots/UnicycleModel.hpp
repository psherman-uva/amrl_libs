/*
  @file:      UnicycleModel.hpp
  @author:    psherman
  @date       Aug 2024
*/

#pragma once

#include <amrl_libs/robots/RobotBase.hpp>
#include <amrl_libs/robots/Ugv.hpp>

#include <amrl_common/point/Point.hpp>
#include <amrl_common/ode_solver/RungeKutta.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>

#include <random>


namespace amrl {

class UnicycleModel : public UgvBase
{
private:
  static constexpr uint8_t kNumStates = 6;
  static constexpr uint8_t kNumInputs = 2;

  using X_t = Eigen::Vector<double, kNumStates>;
  using U_t = Eigen::Vector<double, kNumInputs>;

public:

  /// Constructor
  /// @param pose0     Initial pose for robot
  /// @param accel_max Maximum accleration [linear, rotational]
  UnicycleModel(
    const geometry_msgs::Pose &pose0,
    const double dt,
    const Eigen::Vector2d &velocity_limits,
    const Eigen::Vector2d &accel_limits);

  ~UnicycleModel(void) = default;

  /// Simulate the robot driving one time step forward
  /// @note Uses unicycle model for system dynamic equations
  /// @param u Input command
  void drive(const Eigen::VectorXd &u) final override;

  // Get the full current state of the robot
  Eigen::Vector<double, kNumStates> full_state(void) const;

private:

  // Check if robot pose would end up in obstacles
  bool collision_check(const X_t &x) const;

  // First order ODE describing simulated UGV kinematics
  X_t x_dot(const X_t &x, const U_t &u);

  void modify_u(const Eigen::VectorXd &u);

  // Time step to use for each simulation step
  const double _dt;

  // Current value of all robot states
  X_t _x0;

  // Input during last drive cycle
  U_t _u0;

  // Current value of robots linear velocity
  double _linear_velocity;
  double _theta_velocity;

  // Acceleration limits for robot
  Eigen::Vector2d _accel_limits;

  // Velocity limits for robot
  Eigen::Vector2d _vel_limits;

  // ODE solver for simulating robot dynamics forward in time one step
  RungeKutta<kNumStates, kNumInputs> _solver;


  static constexpr double KEps = 1.0e-6;
};




}