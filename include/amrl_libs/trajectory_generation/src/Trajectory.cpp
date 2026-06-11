
#include <amrl_libs/trajectory_generation/Trajectory.hpp>
#include <amrl_common/util/util.hpp>


#include <iostream>
#include <sstream>

namespace amrl {

  
Trajectory::Trajectory(void)
{

}

Eigen::VectorXd Trajectory::point_at_time(const double t0) const
{
  return X[index_at_time(t0)].q;
}

size_t Trajectory::index_at_time(const double t0) const
{
  if(t0 < X[0].t)      { return 0; }
  if(t0 >= X.back().t) { return X.size() - 1; }

  size_t idx_0 = 0;
  size_t idx_f = X.size()-1;
  size_t curr  = (idx_f + idx_0) / 2;

  // int cnt = 0;
  
  while (idx_0 != idx_f) {
    // std::cout << idx_0 << "-" << curr << "-" << idx_f << std::endl;

    if(t0 >= X[curr].t && t0 < X[curr+1].t) { return curr; }

    if(t0 >= X[curr+1].t) { 
      idx_0 = curr;
    } else {
      idx_f = curr;
    }

    curr = (idx_f + idx_0) / 2;

    // ++cnt;
    // if(cnt >= 20) { return 101; }
  }

  return 0;
}

std::vector<Trajectory::Point> Trajectory::horizon_time(const double t0, const double dt) const
{
  size_t idx0 = index_at_time(t0);  
  size_t idxf = index_at_time(t0+dt);

  if (idxf+1 >= X.size()) {
    return std::vector<Point>(X.begin()+idx0, X.end());
  }
  return std::vector<Point>(X.begin()+idx0, X.begin()+idxf + 1);
}

std::vector<Trajectory::Point> Trajectory::horizon_num_steps(const double t0, const double steps) const
{
  size_t idx0 = index_at_time(t0);

  std::vector<Trajectory::Point> vec(steps);
  for(size_t i = 0; i < steps; ++i) {
    if((idx0 + i) < X.size()) {
      vec[i] = X[idx0 + i];
    } else {
      vec[i] = X.back();
    }
  }

  return vec;
}

std::string Trajectory::to_string(void) const
{
  std::stringstream ss;
  ss << "Trajectory\n";
  
  size_t N = X.size();
  for(size_t i = 0; i < N; ++i) {
    // ss << "idx: " << i;
    ss << "\tt: " << X[i].t;
    ss << "\t\t(x, y, theta): " << amrl::util::container_to_string(X[i].q) << std::endl;
  }

  return ss.str();
}

} // namespace amrl
