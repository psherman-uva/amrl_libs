/*
  @file:      MppiHelpers.hpp
  @author:    psherman
  @date       June 2024
*/


#pragma once

#include <amrl_libs/robots/Ugv.hpp>
#include <amrl_common/point/Point.hpp>

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <list>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace amrl {

struct MppiConfig
{
  MppiConfig(void);
  ~MppiConfig(void) = default;

  std::string to_string(void) const;

  double lambda;

  std::vector<double> u_sigma;
  std::vector<double> u_upper_limits;
  std::vector<double> u_lower_limits;

  std::vector<double> vel_lower_limits;
  std::vector<double> vel_upper_limits;
  
  uint32_t num_samples;
  uint32_t num_sim_steps;
  uint32_t num_cycles;
  uint32_t num_threads;
  double sim_period;
};

struct MppiThreadData
{
  MppiThreadData(
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
    uint32_t num_samples);
  ~MppiThreadData(void) = default;

  geometry_msgs::Pose start_pose; // Starting pose of robot for this cycle
  geometry_msgs::Twist start_vel; // Starting vel of robot for this cycle
  
  std::unique_ptr<UgvAccel> sim_rbt;               // Robot Simulator
  std::vector<Point<double>> rbt_pos;              // Robot trajectory for a simulated sample
  std::vector<double> rbt_heading;                 // Robot headings for simulated sample
  std::vector<Eigen::VectorXd> rbt_vel;
  std::list<Eigen::VectorXd> u_best;               // Current "best" input command sequence
  std::vector<std::vector<Eigen::VectorXd>> u_sim; // Simulated stochastic input vector

  std::vector<std::vector<Point<double>>> rbt_pos_sim;
  std::vector<std::vector<double>> rbt_theta_sim;

  std::vector<double> u_lower_limits;
  std::vector<double> u_upper_limits;

  std::vector<double> vel_lower_limits;
  std::vector<double> vel_upper_limits;
  
  std::default_random_engine gen; 
  std::vector<std::normal_distribution<double>> u_dist;

  uint32_t start_idx;   // Starting sample number for this unique thread
  uint32_t num_samples; // Number of samples this particular thread is responsible for
};

}
