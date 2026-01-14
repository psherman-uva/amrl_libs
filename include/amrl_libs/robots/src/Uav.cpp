
#include <amrl_libs/robots/Uav.hpp>

namespace amrl {

// ----------------------------------- //
// ---         Base Class          --- //
// ----------------------------------- //

UavBase::UavBase(const geometry_msgs::Pose &pose0)
: RobotBase(pose0)
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

UavSimple::UavSimple(const geometry_msgs::Pose &pose0, const double dt)
: UavBase(pose0),
  _dt(dt),
 _solver([&](const X_t &x0, const U_t &u) { return this->x_dot(x0, u); },
                  RungeKutta<kNumStates, kNumInputs>::SolverType::kThirdOrder)
{
}

void UavSimple::drive(void)
{
  X_t x0({_pose.position.x, _pose.position.y});
  X_t x1  = _solver.step(x0, _u, _dt);

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

}