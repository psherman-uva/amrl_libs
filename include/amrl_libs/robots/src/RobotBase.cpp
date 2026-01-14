
#include <amrl_libs/robots/RobotBase.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>

namespace amrl {

RobotBase::RobotBase(const geometry_msgs::Pose &pose0)
 : _pose(pose0),
   _u(Eigen::VectorXd::Zero(2))
{
}

void RobotBase::pose_set(const geometry_msgs::Pose &pose)
{
  _pose = pose;
}

geometry_msgs::Pose RobotBase::pose(void) const
{
  return _pose;
}

void RobotBase::velocity_set(const geometry_msgs::Twist &vel)
{
  _velocity = vel;
}

geometry_msgs::Twist RobotBase::velocity(void) const
{
  return _velocity;
}

void RobotBase::set_input(const Eigen::VectorXd &u)
{
  _u = u;
}

void RobotBase::drive(const Eigen::VectorXd &u)
{
  set_input(u);
  drive();
}

double RobotBase::theta_from_orientation(void) const
{
  return tf2::impl::getYaw(tf2::impl::toQuaternion(_pose.orientation));
}

} // namespace amrl