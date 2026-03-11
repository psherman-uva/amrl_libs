/*
  @file:      MppiController.hpp
  @author:    psherman
  @date       June 2024
*/

#pragma once

#include <amrl_libs/mppi_control/MppiCost.hpp>
#include <amrl_libs/mppi_control/MppiHelpers.hpp>

#include <amrl_libs/robots/Ugv.hpp>
#include <amrl_libs/filter/SavitzkyGolayFilter.hpp>
#include <amrl_libs/obstacle/Obstacle.hpp>
#include <amrl_libs/obstacle/DynamicObstacle.hpp>
#include <amrl_libs/mapping/OccupancyGrid.hpp>

#include <amrl_common/point/Point.hpp>
#include <logging_libs/SqliteDatabase.hpp>
#include <logging_libs/DataHandler.hpp>

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>


#include <cmath>
#include <memory>
#include <random>
#include <vector>
#include <list>
#include <thread>
#include <mutex>
#include <future>

namespace amrl {

class MppiController
{
public:

  MppiController(
    std::shared_ptr<MppiCost> sample_cost,
    const uint32_t num_threads,
    const double lambda,
    const std::vector<double> &u_sigma,
    const std::vector<double> &u_lower_limits, 
    const std::vector<double> &u_upper_limits,
    const std::vector<double> &vel_lower_limits, 
    const std::vector<double> &vel_upper_limits,
    const uint32_t num_samples,
    const uint32_t num_cycles,
    const uint32_t num_sim_steps,
    const double sim_period_sec);

  MppiController(
    std::shared_ptr<MppiCost> sample_cost,
    const MppiConfig controller_config
  );

  ~MppiController(void) = default;

  Eigen::VectorXd control(
    const geometry_msgs::Pose &rbt_pose,
    const geometry_msgs::Twist &rbt_vel,
    std::shared_ptr<OccupancyGrid> map,
    const std::vector<Point<double>> &path);

  void reset(void);

  const std::vector<std::vector<Eigen::VectorXd>>& control_input_samples(void) const;

  const std::vector<double>& weights_get(void) const;

  const std::vector<Point<double>>& best_robot_traj(void) const;

  const std::vector<double>& best_robot_heading(void) const;

  void enable_data_saving(const bool enable);
  
  uint32_t num_sim_steps(void) const;
  uint32_t num_samples(void) const;


  std::shared_ptr<MppiCost> cost_get(void);
  void save_data_set(const bool save_enable);

  const std::list<Eigen::VectorXd> &u_best_get(void) const;

  const std::vector<double> best_cost_components(void);
  
  const std::vector<std::vector<Point<double>>>& saved_rbt_pos(void);
  const std::vector<std::vector<double>>& saved_rbt_theta(void);
  const std::vector<std::vector<double>>& saved_cost_components(void) const;
  const std::vector<double>& saved_costs(void) const;

  std::pair<bool, std::string> logging_setup(
    const std::string &database_name,
    const std::string &table_name);

  void log_cycle(
    const int trial_num,
    const double test_time);

private:

  void single_control_cycle(
    const geometry_msgs::Pose &rbt_pose,
    const geometry_msgs::Twist &rbt_vel,
    const std::vector<Point<double>> &sub_path,
    std::shared_ptr<OccupancyGrid> map);

  void thread_cycle(
    std::shared_ptr<MppiThreadData> data, 
    const std::vector<Point<double>> &sub_path);

  void compute_weights(void);

  void log_data(void);


  // ----------------------------------- //
  //       Class Member Variables        //
  // ----------------------------------- //

  // Object with cost function for specific case
  std::shared_ptr<MppiCost> _sample_cost;

  std::vector<Point<double>> _rbt_pos_best; // Robot trajectory for a simulated sample
  std::vector<double> _rbt_heading_best;    // Robot headings for simulated sample
  std::list<Eigen::VectorXd> _u_full;       // Optimal input over full horizon
  Eigen::VectorXd _u_opt;                   // First element of optimal input
  Eigen::VectorXd _u_init;                   // Initialization input

  // Simulated inputs for all samples
  std::vector<std::vector<Eigen::VectorXd>> _u_sim;

  std::vector<std::vector<Point<double>>> _rbt_pos_saved;
  std::vector<std::vector<double>> _rbt_theta_saved;
  std::vector<std::vector<double>> _costs_saved;
  
  std::vector<double> _costs;   // Total costs for each sample
  std::vector<double> _weights; // Weights for sample computed by MPPI algorithm
  
  SavitzkyGolayFilter _sgf; // Filter control output
  std::vector<double> _lin_filtered;
  std::vector<double> _rot_filtered;
  
  // Low values of lambda result in many trajectories being rejected
  // High values of lambda results in an unweighted average
  const double _lambda;
  const double _lambda_recip;
  
  const uint32_t _num_samples;              //< Number of trajectories per cycle of MPPI
  const uint32_t _num_cycles;               //< Number of MPPI cycles per control request
  const uint32_t _num_sim_steps;      //< Number of control loops per sample [const for explicit thread-safe reads]
  const double _sim_period;           //< Simulation time been control loops [const for explicit thread-safe reads]
  std::atomic<bool> _save_everything; //< Flag to save all data for debugging/tuning [atomic for thread-safety].

  //
  // Stuff for threading
  //
  
  // Total number of threads to use for a control cycle
  const uint32_t _num_threads;

  // Signals when a thread has completed it's simulations for one cycle
  std::vector<std::future<void>> _sample_fut;

  // Info for each unique control calculations thread
  std::vector<std::shared_ptr<MppiThreadData>> _thread_data;

  //
  // Stuff for optional data logging
  //

  // Handle connection to SQLite database in memory
  std::shared_ptr<SqliteDatabase> _logging_db;

  // Handle the logging of data for a single table in database
  std::shared_ptr<DataHandler> _logger;

  // When true, save data for the next control cycle
  bool _logging_flag;

  // How many times logging has been called this trial
  int _log_batch;

  // Data structures for caching logging data
  std::vector<std::string> _data_labels;
  std::vector<int> _data_ints;
  std::vector<double> _data_reals;
};

} // namespace amrl
