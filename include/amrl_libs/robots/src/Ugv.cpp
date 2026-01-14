
#include <amrl_libs/robots/Ugv.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <chrono>

namespace amrl {

// ----------------------------------- //
// ---         Base Class          --- //
// ----------------------------------- //

UgvBase::UgvBase(const geometry_msgs::Pose &pose0) : 
  RobotBase(pose0),
  _vel_limits_set(false),
  _vel_limits_lower({0.0, 0.0}),
  _vel_limits_upper({0.0, 0.0})
{
  _pose.position.z = 0.0;
}

Point<double> UgvBase::pos_xy(void) const
{
  return {_pose.position.x, _pose.position.y};
}

double UgvBase::x(void) const
{
  return _pose.position.x;
}

double UgvBase::y(void) const
{
  return _pose.position.y;
}

double UgvBase::theta(void) const
{
  return theta_from_orientation();
}

void UgvBase::state_set_manually (const Point<double> &pos, const double theta)
{
  static tf2::Quaternion quat;

  quat.setRPY( 0, 0, theta);
  _pose.orientation.x = quat.x();
  _pose.orientation.y = quat.y();
  _pose.orientation.z = quat.z();
  _pose.orientation.w = quat.w();

  _pose.position.x = pos.x;
  _pose.position.y = pos.y;
}

void UgvBase::set_limits(
  const std::vector<double> &limits_lower,
  const std::vector<double> &limits_upper)
{
  _vel_limits_set = false;

  if(limits_lower.size() != 2 || limits_upper.size() != 2) { return; }
  if(limits_lower[0] >= limits_upper[0]) { return; }
  if(limits_lower[1] >= limits_upper[1]) { return; }

  _vel_limits_lower = limits_lower;
  _vel_limits_upper = limits_upper;
  _vel_limits_set   = true;
}

void UgvBase::drive(const Eigen::VectorXd &u)
{
  RobotBase::drive(u);
}

// ----------------------------------- //
// ---         Real Robot          --- //
// ----------------------------------- //

Ugv::Ugv(ros::NodeHandle &nh, 
  const geometry_msgs::Pose &pose0,
  const std::string &rosbot_str) :
    UgvBase(pose0)
{
  _vel_pub  = nh.advertise<geometry_msgs::Twist>("/" + rosbot_str + "/cmd_vel", 1, true);
}

void Ugv::drive(void)
{
  _vel_msg.linear.x  = _u[0];
  _vel_msg.angular.z = _u[1];

  if(_vel_limits_set) {
    _vel_msg.linear.x  = std::min(_vel_limits_upper[0], std::max(_vel_limits_lower[0], _vel_msg.linear.x));
    _vel_msg.angular.z = std::min(_vel_limits_upper[1], std::max(_vel_limits_lower[1], _vel_msg.angular.z));
  }

  _vel_pub.publish(_vel_msg);
}

void Ugv::drive(const Eigen::VectorXd &u)
{
  UgvBase::drive(u);
}

// ----------------------------------- //
// ---       Simulated Robot       --- //
// ----------------------------------- //

UgvSim::UgvSim(const geometry_msgs::Pose &pose0, const double dt)
  :  UgvBase(pose0), 
    _dt(dt),
    _u0(U_t::Zero()),
    _obstacles({}),
    _solver([&](const X_t &x0, const U_t &u) { return this->x_dot(x0, u); },
                  RungeKutta<kNumStates, kNumInputs>::SolverType::kThirdOrder),
    _sigma_angular(0.0),
    _sigma_linear(0.0),
    _generator(std::chrono::system_clock::now().time_since_epoch().count())
{
}

void UgvSim::drive(void)
{
  _u0[0] = _u[0];
  _u0[1] = _u[1];

  if(_sigma_angular > 0.0 || _sigma_linear > 0.0) {
    _u0[0] += _linear_distribution(_generator);
    _u0[1] += _angular_distribution(_generator);
  }

  if(_vel_limits_set) {
    _u0[0] = std::min(_vel_limits_upper[0], std::max(_vel_limits_lower[0], _u0[0]));
    _u0[1] = std::min(_vel_limits_upper[1], std::max(_vel_limits_lower[1], _u0[1]));
  }

  X_t x0({_pose.position.x, _pose.position.y, theta()});
  X_t x1  = _solver.step(x0, _u0, _dt);

  if( _obstacles.empty() || (!collision_check(x1)) ) {
    UgvBase::state_set_manually({x1[0], x1[1]}, x1[2]);
  } else {
    UgvBase::state_set_manually({x0[0], x0[1]}, x1[2]);
  }
}

void UgvSim::drive(const Eigen::VectorXd &u)
{
  UgvBase::drive(u);
}

void UgvSim::set_control_noise(const double linear, const double angular)
{
  if(angular >= 0.0) { 
    _sigma_angular = angular;
    _angular_distribution = std::normal_distribution<double>(0.0, angular);
  }
  if(linear  >= 0.0) {
    _sigma_linear  = linear; 
    _linear_distribution = std::normal_distribution<double>(0.0, linear);
  }
}

void UgvSim::add_obstacles(const std::vector<Obstacle> &obstacles)
{
  _obstacles = obstacles;
}

bool UgvSim::collision_check(const X_t &x) const
{ 
  Point<float> pt(x[0], x[1]);
  for(const auto &obs : _obstacles) {
    if (obs.is_inside(pt)) { return true; }
  }
  return false;
}

UgvSim::X_t UgvSim::x_dot(const UgvSim::X_t &x, const UgvSim::U_t &u)
{
  X_t x_dot;
  x_dot[0] = u[0]*cos(x[2]);
  x_dot[1] = u[0]*sin(x[2]);
  x_dot[2] = u[1];
  return x_dot;
}


// ----------------------------------- //
// ---   Acceleration Input Robot  --- //
// ----------------------------------- //

UgvAccel::UgvAccel(const geometry_msgs::Pose &pose0, const double dt) :  
  UgvBase(pose0), 
  _dt(dt),
  _obstacles({}),
  _solver([&](const X_t &x0, const U_t &u) { return this->x_dot(x0, u); },
                RungeKutta<kNumStates, kNumInputs>::SolverType::kThirdOrder),
  _x0(X_t::Zero()),
  _u0(U_t::Zero()),
  _accel_limit_active(false),
  _accel_limit_linear({0.0, 0.0}),
  _accel_limit_angular({0.0, 0.0}),
  _sigma_linear(0.0),
  _sigma_angular(0.0),
  _generator(std::chrono::system_clock::now().time_since_epoch().count())
{
  // Assume Zero initial velocity always, but intialize pose
  _x0[0] = pose0.position.x;
  _x0[1] = pose0.position.y;
  _x0[2] = theta();
}

void UgvAccel::drive(void)
{
  _u0[0] = _u[0];
  _u0[1] = _u[1];

  if(_accel_limit_active) {
    _u0[0] = std::min(_accel_limit_linear[1], std::max(_accel_limit_linear[0], _u0[0]));
    _u0[1] = std::min(_accel_limit_angular[1], std::max(_accel_limit_angular[0], _u0[1]));
  }
  
  X_t x1 = _solver.step(_x0, _u0, _dt);

  if(_vel_limits_set) {
    x1[3] = std::min(_vel_limits_upper[0], std::max(_vel_limits_lower[0], x1[3]));
    x1[4] = std::min(_vel_limits_upper[1], std::max(_vel_limits_lower[1], x1[4]));
  }

  // If colliding with obstacle. Only change theta and omega
  if( _obstacles.empty() || (!collision_check(x1)) ) {
    _x0 = x1;
  } else {
    _x0[2] = x1[2];
    _x0[4] = x1[4];
  }

  UgvBase::state_set_manually({_x0[0], _x0[1]}, _x0[2]);
}

void UgvAccel::drive(const Eigen::VectorXd &u)
{
  UgvBase::drive(u);
}

void UgvAccel::set_velocity_manually(const geometry_msgs::Twist &vel)
{
  _x0[3] = vel.linear.x;
  _x0[4] = vel.angular.z;
}

void UgvAccel::set_pose_manually(const geometry_msgs::Pose &pose)
{
  pose_set(pose);
  _x0[0] = pose.position.x;
  _x0[1] = pose.position.y;
  _x0[2] = theta_from_orientation();
}

void UgvAccel::set_state_manually(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &vel)
{
  set_pose_manually(pose);
  set_velocity_manually(vel);
}

void UgvAccel::set_control_noise(const double linear, const double angular)
{
  if(angular >= 0.0) { 
    _sigma_angular = angular;
    _angular_distribution = std::normal_distribution<double>(0.0, angular);
  }
  if(linear  >= 0.0) {
    _sigma_linear  = linear; 
    _linear_distribution = std::normal_distribution<double>(0.0, linear);
  }
}

void UgvAccel::set_accel_limits(const double acc_lin, const double acc_ang)
{
  _accel_limit_active = (acc_lin > 0.0) && (acc_ang > 0.0);
  if(_accel_limit_active)  {
    _accel_limit_linear[0]  = -acc_lin;
    _accel_limit_linear[1]  =  acc_lin;
    _accel_limit_angular[0] = -acc_ang;
    _accel_limit_angular[1] =  acc_ang;
  }
}

void UgvAccel::add_obstacles(const std::vector<Obstacle> &obstacles)
{
  _obstacles = obstacles;
}

bool UgvAccel::collision_check(const X_t &x) const
{ 
  Point<float> pt(x[0], x[1]);
  for(const auto &obs : _obstacles) {
    if (obs.is_inside(pt)) { return true; }
  }
  return false;
}

void UgvAccel::set_full_state_manually(const X_t &x_new)
{
  _x0 = x_new;
}

UgvAccel::X_t UgvAccel::get_full_state(void) const
{
  return _x0;
}

Eigen::Vector2d UgvAccel::velocity(void) const
{
  return _x0.tail(2);
}

UgvAccel::X_t UgvAccel::x_dot(const UgvAccel::X_t &x, const UgvAccel::U_t &u)
{
  X_t x_dot;
  x_dot[0] = x[3]*cos(x[2]);
  x_dot[1] = x[3]*sin(x[2]);
  x_dot[2] = x[4];

  x_dot[3] = kLinDamping*x[3] + u[0];
  x_dot[4] = kAngDamping*x[4] + u[1];

  return x_dot;
}



}; // namespace amrl
