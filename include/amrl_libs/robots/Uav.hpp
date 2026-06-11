/*
  @file:      Uav.hpp
  @author:    psherman
  @date       May 2024
  
  @brief Parent class defining UAV interactions
*/

#pragma once

#include <amrl_libs/robots/RobotBase.hpp>
#include <amrl_common/point/Point.hpp>
#include <amrl_common/ode_solver/RungeKutta.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>

#include <random>

namespace amrl {

// ----------------------------------- //
// ---         Base Class          --- //
// ----------------------------------- //

class UavBase : public RobotBase
{
public:

  UavBase(const geometry_msgs::Pose &pose0, const double dt);

  virtual ~UavBase(void) = default;

  Point<double> pos_xy(void) const;
  double x(void) const;
  double y(void) const;
  void pos_set_manually(const Point<double> &pos);

  /// Child classes must provide an implementation to drive robot
  virtual void drive(void) = 0;

protected:

  // Time step to use for each simulation step
  const double _dt;
};


// ------------------------------------ //
// ---    Simple Robot Simulation   --- //
// ------------------------------------ //

class UavSimple : public UavBase
{
private:
  static constexpr uint8_t kNumStates = 2;
  static constexpr uint8_t kNumInputs = 2;

  using X_t = Eigen::Vector<double, kNumStates>;
  using U_t = Eigen::Vector<double, kNumInputs>;

public:

  UavSimple(const geometry_msgs::Pose &pose0, const double dt);

  ~UavSimple(void) = default;

  void drive(void) override;

private:

  // System model: dx/dt = f(x, u)
  X_t x_dot(const X_t &x, const U_t &u);

  // ODE solver for simulating robot dynamics forward in time one step
  RungeKutta<kNumStates, kNumInputs> _solver;
};

class UavYaw : public UavBase
{
private:
  static constexpr uint8_t kNumStates = 6;
  static constexpr uint8_t kNumInputs = 3;

  using X_t = Eigen::Vector<double, kNumStates>;
  using U_t = Eigen::Vector<double, kNumInputs>;

public:

  UavYaw(const geometry_msgs::Pose &pose0, const double dt);

  ~UavYaw(void) = default;

  void drive(void) override;

private:

  // Current full state of the system
  X_t _x; 

  // System model: dx/dt = f(x, u)
  X_t x_dot(const X_t &x, const U_t &u);

  // ODE solver for simulating robot dynamics forward in time one step
  RungeKutta<kNumStates, kNumInputs> _solver;
};

} // namespace amrl