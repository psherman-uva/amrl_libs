
#include <amrl_libs/pid/PIDGains.hpp>

namespace amrl {

PIDGains::PIDGains(const double Kp, const double Ki, const double Kd) :
  Kp(Kp), Ki(Ki), Kd(Kd)
{}

PIDGains::PIDGains(const std::vector<double> &gains) : 
  PIDGains(gains[0], gains[1], gains[2])
{}

PIDGains::PIDGains(const Eigen::Vector3d &gains) :
  PIDGains(gains[0], gains[1], gains[2])
{}

PIDGains::PIDGains(const std::initializer_list<double> &il) :
  PIDGains(*il.begin(), *(il.begin()+1), *(il.begin()+2))
{}

void PIDGains::set(const double Kp_new, const double Ki_new, const double Kd_new)
{
  Kp = Kp_new;
  Ki = Ki_new;
  Kd = Kd_new;
}

void PIDGains::set(const std::vector<double> &gains)
{
  Kp = gains[0];
  Ki = gains[1];
  Kd = gains[2];
}

void PIDGains::set(const Eigen::Vector3d &gains)
{
  Kp = gains[0];
  Ki = gains[1];
  Kd = gains[2];
}

} // namespace amrl