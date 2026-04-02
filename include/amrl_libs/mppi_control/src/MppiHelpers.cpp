
#include <amrl_libs/mppi_control/MppiHelpers.hpp>

#include <amrl_common/util/util.hpp>

#include <chrono>

namespace amrl {



// **************************************************** //
//                MPPI Configuration                    //
// **************************************************** //

MppiConfig::MppiConfig(void) :
  lambda(0),
  u_sigma(),
  u_upper_limits(),
  u_lower_limits(),
  vel_lower_limits(),
  vel_upper_limits(),
  num_samples(0),
  num_sim_steps(0),
  num_cycles(0),
  num_threads(0),
  sim_period(0.0)
{
}

std::string MppiConfig::to_string(void) const
{
  std::stringstream ss;

  ss << "Lambda: "           << lambda << "\n";
  ss << "u_sigma: "          << util::container_to_string(u_sigma) << "\n";
  ss << "u_lower_limits: "   << util::container_to_string(u_lower_limits) << "\n";
  ss << "u_upper_limits: "   << util::container_to_string(u_upper_limits) << "\n";
  ss << "vel_lower_limits: " << util::container_to_string(vel_lower_limits) << "\n";
  ss << "vel_upper_limits: " << util::container_to_string(vel_upper_limits) << "\n";
  ss << "num_samples: "      << num_samples << "\n";
  ss << "num_sim_steps: "    << num_sim_steps << "\n";
  ss << "num_cycles: "       << num_cycles << "\n";
  ss << "sim_period: "       << sim_period << "\n";

  return ss.str();
}

// **************************************************** //
//                MPPI Thread Data                      //
// **************************************************** //

MppiThreadData::MppiThreadData(
    const geometry_msgs::Pose &p0,
    const uint32_t num_sim_steps,
    const double sim_period,
    const std::list<Eigen::VectorXd> &u_initial,
    const std::vector<double> &u_sigma,
    const std::vector<double> &u_lower_limits, 
    const std::vector<double> &u_upper_limits,
    const std::vector<double> &vel_lower_limits, 
    const std::vector<double> &vel_upper_limits,
    uint32_t start_index,
    uint32_t num_samples) :
  start_pose(p0),
  sim_rbt(std::make_unique<UgvAccel>(p0, sim_period)),
  rbt_pos(num_sim_steps),
  rbt_heading(num_sim_steps),
  rbt_vel(num_sim_steps, Eigen::VectorXd::Zero(2)),
  u_best(u_initial),
  u_sim(),
  u_lower_limits(u_lower_limits),
  u_upper_limits(u_upper_limits),
  vel_lower_limits(vel_lower_limits),
  vel_upper_limits(vel_upper_limits),
  gen(),
  start_idx(start_index),
  num_samples(num_samples)
{
  gen.seed(std::chrono::system_clock::now().time_since_epoch().count());
  for(const auto &s : u_sigma) {
    u_dist.push_back(std::normal_distribution<double>(0.0, s));
  }

  sim_rbt->set_limits(vel_lower_limits, vel_upper_limits);
}

}
