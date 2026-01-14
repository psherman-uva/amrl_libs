
#pragma once

#include <Eigen/Dense>
#include <vector>

namespace amrl {

struct PIDGains 
{
  PIDGains(const double Kp, const double Ki, const double Kd);
  PIDGains(const std::vector<double> &gains);
  PIDGains(const Eigen::Vector3d &gains);
  PIDGains(const std::initializer_list<double> &il);

  ~PIDGains(void) = default;

  void set(const double Kp_new, const double Ki_new, const double Kd_new);
  void set(const std::vector<double> &gains);
  void set(const Eigen::Vector3d &gains);

  double Kp;
  double Ki;
  double Kd;
};

}

