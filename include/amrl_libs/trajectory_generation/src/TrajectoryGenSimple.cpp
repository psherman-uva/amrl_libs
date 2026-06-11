
#include <amrl_libs/trajectory_generation/TrajectoryGenSimple.hpp>
#include <amrl_common/util/util.hpp>

namespace amrl {

TrajectoryGenSimple::TrajectoryGenSimple(void)
{
}

Trajectory TrajectoryGenSimple::generate(const std::vector<Point<double>> &path, 
  double v_ref, double dt) const
{ 
  Trajectory traj;
  
  double t0        = 0.0;
  double d0        = 0.0;
  Point<double> p0 = path[0];

  for(size_t i = 0; i < path.size() - 1; ++i) {
    double Di        = util::distance(path[i], path[i+1]);
    double t_limit   = 0.0;
    double Ti        = Di / v_ref;
    double theta_ref = atan2(path[i+1].y-path[i].y, path[i+1].x-path[i].x);

    while (t_limit < Ti) {
      Trajectory::Point pt;
      pt.t    = t0;
      pt.q = Eigen::Vector3d::Zero();
      pt.q[0] = p0.x;
      pt.q[1] = p0.y;
      pt.q[2] = theta_ref;

      t0      += dt;
      t_limit += dt;
      p0.x = path[i].x + (t_limit/Ti)*(path[i+1].x - path[i].x);
      p0.y = path[i].y + (t_limit/Ti)*(path[i+1].y - path[i].y);

      traj.X.push_back(pt);
    }
  }

  return traj;
}

}