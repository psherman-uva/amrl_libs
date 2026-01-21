
#include <amrl_libs/robots/UnicycleModel.hpp>
#include <amrl_common/util/util.hpp>


namespace amrl {

UnicycleModel::UnicycleModel(
    const geometry_msgs::Pose &pose0,
    const double dt,
    const Eigen::Vector2d &velocity_limits,
    const Eigen::Vector2d &accel_limits) : 
  UgvBase(pose0),
  _dt(dt),
  _x0(X_t::Zero()),
  _u0(U_t::Zero()),
  _linear_velocity(0.0),
  _theta_velocity(0.0),
  _accel_limits(accel_limits),
  _vel_limits(velocity_limits),
  _solver([&](const X_t &x0, const U_t &u) { return this->x_dot(x0, u); },
                  RungeKutta<kNumStates, kNumInputs>::SolverType::kFourthOrderOptimal)
{
  _x0[0] = pose0.position.x;
  _x0[1] = pose0.position.y;
  _x0[2] = theta();

  _theta_velocity = _x0[2];

  set_limits(
    {0.0,           -_vel_limits[1]}, 
    {_vel_limits[0], _vel_limits[1]});
}

Eigen::Vector<double, UnicycleModel::kNumStates> UnicycleModel::full_state(void) const
{
  return _x0;
}

void UnicycleModel::drive(const Eigen::VectorXd &u)
{

  modify_u(u);
  _x0 = _solver.step(_x0, u, _dt);


  if(_x0[4] > KEps || _x0[3] > KEps) {
    _theta_velocity = atan2(_x0[4], _x0[3]);
  } else {
    _theta_velocity = _x0[2];
  }

  if (fabs(_theta_velocity - _x0[2]) > M_PI_4)
    _linear_velocity = -1*sqrt(util::square(_x0[3]) + util::square(_x0[4]));
  else
    _linear_velocity = sqrt(util::square(_x0[3]) + util::square(_x0[4]));

  UgvBase::state_set_manually({_x0[0], _x0[1]}, _x0[2]);
}

bool UnicycleModel::collision_check(const X_t &x) const
{
  return true;
}

void UnicycleModel::modify_u(const Eigen::VectorXd &u0)
{
  _u0[0] = u0[0];
  _u0[1] = u0[1];

  // Eigen::Vector2d vel_limited = u0;
  // vel_limited[0] = std::min(_vel_limit_linear, std::max(-_vel_limit_linear, vel_limited[0]));
  // vel_limited[1] = std::min(_vel_limit_angular, std::max(-_vel_limit_angular, vel_limited[1]));

  // Linear Acceleration Input
  static double dVelLimit = _accel_limits[0]*_dt;

  double dV = _u0[0] - _linear_velocity;
  if (fabs(dV) > dVelLimit) {
    _u0[0] = util::sgn(dV) * _accel_limits[0];
  } else {
    _u0[0] = dV / _dt;
  }

  // Rotational Acceleration Input
  static double dAngularLimit = _accel_limits[1]*_dt;

  double dW = _u0[1] - _x0[5];
  if (fabs(dW) > dAngularLimit) {
    _u0[1] = util::sgn(dW) * _accel_limits[1];
  } else {
    _u0[1] = dW / _dt;
  }
}

UnicycleModel::X_t UnicycleModel::x_dot(const X_t &x, const U_t &u)
{
  double v  = sqrt(util::square(x[3]) + util::square(x[4]));
  double ct = cos(x[2]);
  double st = sin(x[2]);

  X_t x_dot;
  x_dot[0] = x[3];
  x_dot[1] = x[4];
  x_dot[2] = x[5];

  x_dot[3] = u[0]*ct - v*x[5]*st;
  x_dot[4] = u[0]*st + v*x[5]*ct;
  x_dot[5] = u[1];

  return x_dot;
}


}