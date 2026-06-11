
#pragma once

#include <Eigen/Dense>
#include <vector>


namespace amrl {


struct Trajectory
{
  struct Point {
    double t;
    Eigen::VectorXd q;
  };

  Trajectory(void);
  ~Trajectory(void) = default;

  Eigen::VectorXd point_at_time(const double t0) const;
  size_t index_at_time(const double t0) const;
  std::vector<Point> horizon_time(const double t0, const double dt) const;
  std::vector<Point> horizon_num_steps(const double t0, const double steps) const;
  std::string to_string(void) const;

  std::vector<Point> X;
};
  
} // namespace amrl
