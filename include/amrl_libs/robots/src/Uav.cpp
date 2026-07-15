
#include <amrl_libs/robots/Uav.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace amrl {

// ----------------------------------- //
// ---         Base Class          --- //
// ----------------------------------- //

UavBase::UavBase(const geometry_msgs::Pose &pose0, const double dt) :
  RobotBase(pose0),
  _dt(dt)
{
}

Point<double> UavBase::pos_xy(void) const
{
  return Point<double>(_pose.position.x, _pose.position.y);
}

double UavBase::x(void) const
{
  return _pose.position.x;
}

double UavBase::y(void) const
{
  return _pose.position.y;
}

void UavBase::pos_set_manually(const Point<double> &pos)
{
  _pose.position.x = pos.x;
  _pose.position.y = pos.y;
}

// ------------------------------------ //
// ---    Simple Robot Simulation   --- //
// ------------------------------------ //

UavSimple::UavSimple(const geometry_msgs::Pose &pose0, const double dt) : 
  UavBase(pose0, dt),
 _solver([&](const X_t &x0, const U_t &u) { return this->x_dot(x0, u); },
                  RungeKutta<kNumStates, kNumInputs>::SolverType::kThirdOrder)
{
  _u = Eigen::VectorXd::Zero(kNumInputs);
}

void UavSimple::drive(void)
{
  X_t x0({_pose.position.x, _pose.position.y});
  X_t x1  = _solver(x0, _u, _dt);

  _pose.position.x = x1[0];
  _pose.position.y = x1[1];
}

UavSimple::X_t UavSimple::x_dot(const UavSimple::X_t &x, const UavSimple::U_t &u)
{
  X_t x_dot;
  x_dot[0] = u[0];
  x_dot[1] = u[1];
  return x_dot;
}

// ------------------------------------ //
// ---  Simple UAV with yaw action  --- //
// ------------------------------------ //

UavYaw::UavYaw(const geometry_msgs::Pose &pose0, const double dt) : 
  UavBase(pose0, dt),
  _solver([&](const X_t &x0, const U_t &u) { return this->x_dot(x0, u); },
                  RungeKutta<kNumStates, kNumInputs>::SolverType::kThirdOrder)
{
  _u = Eigen::VectorXd::Zero(kNumInputs);
  _x = Eigen::VectorXd::Zero(kNumStates);

  _x[0] = _pose.position.x;
  _x[1] = _pose.position.y;
  _x[2] = theta_from_orientation();
}

void UavYaw::drive(void)
{
  static tf2::Quaternion quat;

  _x = _solver(_x, _u, _dt);

  quat.setRPY(0.0, 0.0, _x[2]);

  _pose.position.x = _x[0];
  _pose.position.y = _x[1];

  _pose.orientation.x = quat.x();
  _pose.orientation.y = quat.y();
  _pose.orientation.z = quat.z();
  _pose.orientation.w = quat.w();
}

UavYaw::X_t UavYaw::x_dot(const UavYaw::X_t &x, const UavYaw::U_t &u)
{
  X_t x_dot;
  x_dot[0] = x[3];
  x_dot[1] = x[4];
  x_dot[2] = x[5];
  x_dot[3] = u[0];
  x_dot[4] = u[1];
  x_dot[5] = u[2];

  return x_dot;
}

}
