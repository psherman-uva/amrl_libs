/*
  @file:      Ugv.hpp
  @author:    psherman
  @date       July 2023
  
  @brief Classes defining UGV implementation
*/

#pragma once

#include <amrl_libs/robots/RobotBase.hpp>
#include <amrl_libs/obstacle/Obstacle.hpp>

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

class UgvBase : public RobotBase
{
public:

  /// Constructor
  /// @param pose0 Initial pose for robot
  UgvBase(const geometry_msgs::Pose &pose0);

  /// Destructor
  virtual ~UgvBase(void) = default;

  Point<double> pos_xy(void) const;
  double x(void) const;
  double y(void) const;
  double theta(void) const;
  void state_set_manually (const Point<double> &pos, const double theta);

  /// Set velocity limits used in drive command.
  /// If set, robot will limit input commands to:
  ///     u = min(+limit, max(-limit, u))
  /// @note Setting to 0 will disable check
  /// @param limit_linear Limit for linear X velocity
  /// @param limit_angular Limit for angular Z velocity
  void set_limits(
    const std::vector<double> &limits_lower,
    const std::vector<double> &limits_upper);

  /// Child classes must provide an implementation to drive robot
  virtual void drive(void) = 0;
  virtual void drive(const Eigen::VectorXd &u) override;

protected:

  // Velocity limits for robot
  bool _vel_limits_set;
  std::vector<double> _vel_limits_lower;
  std::vector<double> _vel_limits_upper;
};

class Ugv : public UgvBase
{
public:
  /// Constructor
  /// @param nh ROS nodehandle object
  /// @param pose0 Initial pose of robot
  /// @param rosbot_str String name of robot (.e.g "rosbot_2")
  Ugv(ros::NodeHandle &nh, 
    const geometry_msgs::Pose &pose0,
    const std::string &rosbot_str);

  /// Destructor
  ~Ugv(void) = default;

  /// Drive rosbot.
  void drive(void) final override;
  void drive(const Eigen::VectorXd &u) final override;
 
private:

  void viconCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);

  ros::Publisher _vel_pub;

  geometry_msgs::Twist _vel_msg;
};

class UgvSim : public UgvBase
{
private:
  static constexpr uint8_t kNumStates = 3;
  static constexpr uint8_t kNumInputs = 2;

  using X_t = Eigen::Vector<double, kNumStates>;
  using U_t = Eigen::Vector<double, kNumInputs>;

public:

  /// Constructor
  /// @param pose0 
  /// @param dt 
  UgvSim(const geometry_msgs::Pose &pose0, const double dt);

  /// Default desctructor
  ~UgvSim(void) = default;

  /// Simulate the robot driving one time step forward
  /// @note Uses unicycle model for system dynamic equations
  /// @param u Input command 
  void drive(void) final override;
  void drive(const Eigen::VectorXd &u) final override;

  /// Set standard deviation for randomness added to command to simulate
  /// input noise of a real system
  /// @param linear std-dev for linear command
  /// @param angular std-dev for angular command
  void set_control_noise(const double linear, const double angular);

  void add_obstacles(const std::vector<Obstacle> &obstacles);

private:

  // Check if robot pose would end up in obstacles
  bool collision_check(const X_t &x) const;

  // First order ODE describing simulated UGV kinematics: dx/dt = f(x, u)
  X_t x_dot(const X_t &x, const U_t &u);

  // Time step to use for each simulation step
  const double _dt;

  // Input during last drive cycle
  U_t _u0;

  // Obstacles in the environment. May or may not be currently mapped by robot
  std::vector<Obstacle> _obstacles;

  // ODE solver for simulating robot dynamics forward in time one step
  RungeKutta<kNumStates, kNumInputs> _solver;

  double _sigma_angular;
  double _sigma_linear;

  std::default_random_engine _generator;
  std::normal_distribution<double> _angular_distribution;
  std::normal_distribution<double> _linear_distribution;
};

class UgvAccel : public UgvBase
{
private:
  static constexpr uint8_t kNumStates = 5;
  static constexpr uint8_t kNumInputs = 2;

  using X_t = Eigen::Vector<double, kNumStates>;
  using U_t = Eigen::Vector<double, kNumInputs>;

public:

  /// Constructor
  /// @param pose0 
  /// @param dt 
  UgvAccel(const geometry_msgs::Pose &pose0, const double dt);

  /// Default desctructor
  ~UgvAccel(void) = default;

  /// Simulate the robot driving one time step forward
  /// @note Uses unicycle model for system dynamic equations
  void drive(void) final override;
  void drive(const Eigen::VectorXd &u) final override;


  /// Set standard deviation for randomness added to command to simulate
  /// input noise of a real system
  /// @param linear std-dev for linear command
  /// @param angular std-dev for angular command
  void set_control_noise(const double linear, const double angular);

  void set_accel_limits(const double acc_lin, const double acc_ang);

  void set_velocity_manually(const geometry_msgs::Twist &vel);
  void set_pose_manually(const geometry_msgs::Pose &pose);
  void set_state_manually(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &vel);

  void set_full_state_manually(const X_t &x_new);
  X_t get_full_state(void) const;
  Eigen::Vector2d velocity(void) const;

  void add_obstacles(const std::vector<Obstacle> &obstacles);

private:

  // Check if robot pose would end up in obstacles
  bool collision_check(const X_t &x) const;

  // First order ODE describing simulated UGV kinematics
  X_t x_dot(const X_t &x, const U_t &u);

  // Time step to use for each simulation step
  const double _dt;

  // Obstacles in the environment. May or may not be currently mapped by robot
  std::vector<Obstacle> _obstacles;

  // ODE solver for simulating robot dynamics forward in time one step
  RungeKutta<kNumStates, kNumInputs> _solver;

  /// Current State of the Robot: [x, y, theta, vel_x, theta_dot]
  X_t _x0;

  // Input during last drive cycle
  U_t _u0;

  // Acceleration Command limits
  bool _accel_limit_active;
  std::vector<double> _accel_limit_linear;
  std::vector<double> _accel_limit_angular;

  double _sigma_linear;
  double _sigma_angular;

  std::default_random_engine _generator;
  std::normal_distribution<double> _angular_distribution;
  std::normal_distribution<double> _linear_distribution;

  static constexpr double kLinDamping = 0.0; ;//-1.25;
  static constexpr double kAngDamping = 0.0; ;//-0.6;
};

} // namespace amrl
