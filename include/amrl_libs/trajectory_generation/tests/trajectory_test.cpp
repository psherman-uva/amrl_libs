
#include <amrl_libs/trajectory_generation/Trajectory.hpp>
#include <amrl_libs/trajectory_generation/TrajectoryGenSimple.hpp>
#include <amrl_common/util/util.hpp>

#include <random>
#include <chrono>



static unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
static std::default_random_engine generator(seed);
static std::uniform_real_distribution<double> distribution;

void print_traj(const std::vector<amrl::Trajectory::Point> &traj);

int main (int argc, char *argvp[]) 
{ 
  std::vector<amrl::Point<double>> path({
    {0.0, 0.0},
    {5.0, 2.0},
    {2.0, 8.0},
  });

  amrl::TrajectoryGenSimple gen_traj;
  amrl::Trajectory traj = gen_traj.generate(path, 0.8, 0.1);
  print_traj(traj.X);

  return 0;
}


void print_traj(const std::vector<amrl::Trajectory::Point> &traj)
{
  size_t N = traj.size();
  std::cout << "Trajectory:" << std::endl;
  for(size_t i = 0; i < N; ++i) {
    std::cout << "idx: " << i;
    std::cout << "\tt: " << traj[i].t;
    std::cout << "\tq: " << amrl::util::container_to_string(traj[i].q) << std::endl;
  }
}