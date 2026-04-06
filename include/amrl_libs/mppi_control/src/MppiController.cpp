
#include <amrl_libs/mppi_control/MppiController.hpp>
#include <amrl_libs/util/ros_util.hpp>

#include <amrl_common/util/util.hpp>
#include <logging_util/util.hpp>

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace amrl  {



MppiController::MppiController(
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
    const double sim_period_sec) : 

  _sample_cost(sample_cost),
  _rbt_pos_best(num_sim_steps),
  _rbt_heading_best(num_sim_steps),
  _u_full(),
  _u_opt(Eigen::VectorXd::Zero(2)),
  _u_init(Eigen::VectorXd::Zero(2)),
  _u_sim(),
  _costs_saved(num_samples, std::vector<double>(sample_cost->num_components_get(), 0.0)),
  _costs(num_samples),
  _weights(num_samples),
  _sgf(num_sim_steps),
  _lambda(lambda),
  _lambda_recip(-1.0/lambda),
  _num_samples(num_samples),
  _num_cycles(num_cycles),
  _num_sim_steps(num_sim_steps),
  _sim_period(sim_period_sec),
  _save_everything(false),
  _num_threads(num_threads),
  _sample_fut(num_threads),
  _thread_data(),
  _logging_db(nullptr),
  _logger(nullptr),
  _logging_flag(false),
  _log_batch(0),
  _data_labels(),
  _data_ints(),
  _data_reals()
{
  
  _u_sim = std::vector<std::vector<Eigen::VectorXd>>(
    num_samples, std::vector<Eigen::VectorXd>(num_sim_steps, Eigen::VectorXd::Zero(2)));
    
  _u_init[0] = 0.2*u_upper_limits[0];
  _u_init[1] = 0.0;

  _u_full.resize(num_sim_steps, _u_init);
  _lin_filtered.resize(num_sim_steps);
  _rot_filtered.resize(num_sim_steps);


  geometry_msgs::Pose p0;
  util::zero_pose(p0);

  uint32_t num_samples_thread = num_samples / _num_threads;
  uint32_t remainder_samples  = num_samples - (_num_threads*num_samples_thread);
  uint32_t start_idx = 0;

  for(uint32_t i = 0; i < _num_threads; ++i) {
    std::shared_ptr<MppiThreadData> data(std::make_shared<MppiThreadData>(
        p0, num_sim_steps, _sim_period, _u_full, 
        u_sigma, u_lower_limits, u_upper_limits,
        vel_lower_limits, vel_upper_limits,
        start_idx, num_samples_thread));

    if(remainder_samples > 0) {
      data->num_samples += 1;
      remainder_samples -= 1;
    }
    
    start_idx += data->num_samples;
    data->u_sim = std::vector<std::vector<Eigen::VectorXd>>(
      data->num_samples, std::vector<Eigen::VectorXd>(num_sim_steps, Eigen::VectorXd::Zero(2)));
      
      _thread_data.emplace_back(std::move(data));
  }
}

void MppiController::reset(void)
{
  std::fill(_u_full.begin(), _u_full.end(), _u_init);
}

MppiController::MppiController(
  std::shared_ptr<MppiCost> sample_cost,
  const MppiConfig controller_config)
: MppiController(sample_cost, 
    controller_config.num_threads,
    controller_config.lambda,
    controller_config.u_sigma,
    controller_config.u_lower_limits,
    controller_config.u_upper_limits,
    controller_config.vel_lower_limits,
    controller_config.vel_upper_limits,
    controller_config.num_samples,
    controller_config.num_cycles,
    controller_config.num_sim_steps,
    controller_config.sim_period)
{
}

Eigen::VectorXd MppiController::control(
  const geometry_msgs::Pose &rbt_pose,
  const geometry_msgs::Twist &rbt_vel,
  std::shared_ptr<OccupancyGrid> map,
  const std::vector<Point<double>> &path)
{
  // Modify optimal plan over horizon to use previous best (shifted by 1) as
  // initial input for next cycle
  _u_full.pop_front();
  _u_full.push_back(_u_full.back());

  for (size_t i = 0; i < _num_cycles; ++i) {
    single_control_cycle(rbt_pose, rbt_vel, path, map);
  }

  // Simulate "best" control trajectory
  _thread_data[0]->sim_rbt->set_state_manually(rbt_pose, rbt_vel);
  std::list<Eigen::VectorXd>::iterator vel_it = _u_full.begin(); 
  for (uint32_t tk = 0; tk < _num_sim_steps; ++tk) {
    _thread_data[0]->sim_rbt->drive(*vel_it);
    _rbt_pos_best[tk]     = _thread_data[0]->sim_rbt->pos_xy();
    _rbt_heading_best[tk] = _thread_data[0]->sim_rbt->theta();
    ++vel_it;
  }

  if(_logging_flag) { log_data(); }

  _u_opt[0] = _u_full.front()[0];
  _u_opt[1] = _u_full.front()[1];
  return _u_opt;
}

void MppiController::single_control_cycle(
    const geometry_msgs::Pose &rbt_pose,
    const geometry_msgs::Twist &rbt_vel,
    const std::vector<Point<double>> &sub_path,
    std::shared_ptr<OccupancyGrid> map)
{
  _sample_cost->reset_min_max_costs();

  for(uint32_t i = 0; i < _num_threads; ++i) {
    _thread_data[i]->u_best     = _u_full;
    _thread_data[i]->start_pose = rbt_pose;
    _thread_data[i]->start_vel  = rbt_vel;
    _sample_fut[i] = std::async(std::launch::async, &MppiController::thread_cycle, this, _thread_data[i], sub_path);
  }
  for(auto &fut : _sample_fut) { fut.get(); } 
  
  uint32_t start_idx = 0;
  for(auto data : _thread_data) {
    std::copy_n(data->u_sim.begin(), data->num_samples, _u_sim.begin() + start_idx);
    start_idx += data->num_samples;
  }

  // - Normalize cost components
  // - Multiply components by alpha weights
  // - Compute total cost for each sample
  _sample_cost->final_cost_operations();
  _sample_cost->costs_get(_costs);
  if(_save_everything) {
    _sample_cost->components_get(_costs_saved);
  }

  compute_weights();

  // Smooth the command input
  for(uint32_t i = 0; i < _num_sim_steps; ++i) {
    _lin_filtered[i] = 0.0;
    _rot_filtered[i] = 0.0;

    for (uint32_t j = 0; j < _num_samples; ++j) {
      _lin_filtered[i] += _weights[j] * _u_sim[j][i][0];
      _rot_filtered[i] += _weights[j] * _u_sim[j][i][1];
    }
  }
  _lin_filtered = _sgf.filter(_lin_filtered);
  _rot_filtered = _sgf.filter(_rot_filtered);

  uint32_t tk = 0;
  for (auto it = _u_full.begin(); it != _u_full.end(); ++it) {
    (*it)[0] = _lin_filtered[tk];
    (*it)[1] = _rot_filtered[tk];
    ++tk;
  }
}

void MppiController::thread_cycle(
  std::shared_ptr<MppiThreadData> data, 
  const std::vector<Point<double>> &sub_path)
{
  std::list<Eigen::VectorXd>::iterator u_itr;
  
  uint32_t cost_idx = data->start_idx;
  for(uint32_t i = 0; i < data->num_samples; ++i) {
    data->sim_rbt->set_state_manually(data->start_pose, data->start_vel);
    u_itr = _u_full.begin();

    for(uint32_t tk = 0; tk < _num_sim_steps; ++tk) {
      double u0 = (*u_itr)[0] + data->u_dist[0](data->gen);
      double u1 = (*u_itr)[1] + data->u_dist[1](data->gen);

      data->u_sim[i][tk][0] = std::max(data->u_lower_limits[0], std::min(data->u_upper_limits[0], u0));
      data->u_sim[i][tk][1] = std::max(data->u_lower_limits[1], std::min(data->u_upper_limits[1], u1));

      data->sim_rbt->drive(data->u_sim[i][tk]);
      data->rbt_pos[tk]     = data->sim_rbt->pos_xy();
      data->rbt_heading[tk] = data->sim_rbt->theta();
      data->rbt_vel[tk]     = data->sim_rbt->velocity();

      if(_save_everything) {
        data->rbt_pos_sim[i][tk]   = data->rbt_pos[tk];
        data->rbt_theta_sim[i][tk] = data->rbt_heading[tk];
      }

      ++u_itr;
    }


    _sample_cost->compute_costs(cost_idx, data->rbt_pos, data->rbt_heading, data->rbt_vel, sub_path);
    _sample_cost->min_max_cost_check(cost_idx);
    ++cost_idx;
  }
}

void MppiController::compute_weights(void)
{
  const double min_cost = *std::min_element(_costs.begin(), _costs.end());
  
  double eta      = 0.0; // Normalization factor
  for (uint32_t i = 0; i < _num_samples; ++i) {
    _weights[i] = exp(_lambda_recip * (_costs[i] - min_cost));
    eta += _weights[i];
  }
  eta = 1/eta;

  // Normalize Weights
  std::for_each(_weights.begin(), _weights.end(), [eta](double &w) { w *= eta; });
}

void MppiController::save_data_set(const bool save_enable)
{
  _save_everything = save_enable;

  if(save_enable) {
    // Combined full vectors
    _rbt_pos_saved.resize(_num_samples);
    for(auto &vec : _rbt_pos_saved) {
      vec.resize(_num_sim_steps);
    }

    _rbt_theta_saved.resize(_num_samples);
    for(auto &vec : _rbt_theta_saved) {
      vec.resize(_num_sim_steps);
    }

    // Data each thread is responsible for
    for(auto data : _thread_data) {
      data->rbt_pos_sim.resize(data->num_samples);
      for(auto &vec : data->rbt_pos_sim) {
        vec.resize(_num_sim_steps);
      }

      data->rbt_theta_sim.resize(data->num_samples);
      for(auto &vec : data->rbt_theta_sim) {
        vec.resize(_num_sim_steps);
      }
    }
  }
}

const std::vector<double> MppiController::best_cost_components(void)
{
  _sample_cost->components_get(_costs_saved);
  uint32_t N = _sample_cost->num_components_get();

  std::vector<double> best_costs(N, 0.0);
  for (uint32_t i = 0; i < N; ++i) {
    for (uint32_t j = 0; j < _num_samples; ++j) {
      best_costs[i] += _weights[j] * _costs_saved[j][i];
    }
  }

  return best_costs;
}

const std::vector<std::vector<Eigen::VectorXd>>& MppiController::control_input_samples(void) const
{
  return _u_sim;
}

const std::vector<double>& MppiController::weights_get(void) const
{
  return _weights;
}

uint32_t MppiController::num_sim_steps(void) const
{
  return _num_sim_steps;
}

uint32_t MppiController::num_samples(void) const
{
  return _num_samples;
}

const std::vector<amrl::Point<double>>& MppiController::best_robot_traj(void) const
{
  return _rbt_pos_best;
}

const std::vector<double>& MppiController::best_robot_heading(void) const
{
  return _rbt_heading_best;
}

std::shared_ptr<MppiCost> MppiController::cost_get(void)
{
  return _sample_cost;
}

const std::list<Eigen::VectorXd>& MppiController::u_best_get(void) const
{
  return _u_full;
}

const std::vector<std::vector<Point<double>>>& MppiController::saved_rbt_pos(void)
{
  uint32_t start_idx = 0;
  for(auto data : _thread_data) {
    std::copy_n(data->rbt_pos_sim.begin(), data->num_samples, _rbt_pos_saved.begin() + start_idx);
    start_idx += data->num_samples;
  }
  return _rbt_pos_saved;
}

const std::vector<std::vector<double>>& MppiController::saved_rbt_theta(void)
{
  uint32_t start_idx = 0;
  for(auto data : _thread_data) {
    std::copy_n(data->rbt_theta_sim.begin(), data->num_samples, _rbt_theta_saved.begin() + start_idx);
    start_idx += data->num_samples;
  }
  return _rbt_theta_saved;
}

const std::vector<std::vector<double>>& MppiController::saved_cost_components(void) const
{
  return _costs_saved;
}

const std::vector<double>& MppiController::saved_costs(void) const
{
  return _costs;
}

std::pair<bool, std::string> MppiController::logging_setup(
  const std::string &database_name,
  const std::string &table_name)
{
  _logging_flag   = false;
  _log_batch      = 0;
  std::string msg = "";

  std::vector<std::string> db_labels_hdr({});
  std::vector<std::string> db_ints_hdr({"trial", "batch", "traj_idx"});
  std::vector<std::string> db_reals_hdr({"t_step"});
  db_reals_hdr.push_back("cost_dist");
  db_reals_hdr.push_back("cost_traj");
  db_reals_hdr.push_back("cost_obs");
  db_reals_hdr.push_back("cost_dyn");
  db_reals_hdr.push_back("cost_gain");
  for(size_t i = 0; i < _num_sim_steps; ++i) {
    db_reals_hdr.push_back("x_" + std::to_string(i));
    db_reals_hdr.push_back("y_" + std::to_string(i));
    db_reals_hdr.push_back("theta_" + std::to_string(i));
    db_reals_hdr.push_back("u0_" + std::to_string(i));
    db_reals_hdr.push_back("u1_" + std::to_string(i));
  }

  _logging_db = std::make_shared<SqliteDatabase>();
  if(_logging_db->open_database(database_name)) {
    _logger = std::make_shared<amrl::DataHandler>(_logging_db);
    if(_logger->create_table(table_name, db_labels_hdr, db_ints_hdr, db_reals_hdr)) {
      save_data_set(true);
      _logger->logging_begin();
      _data_labels.resize(db_labels_hdr.size());
      _data_ints.resize(db_ints_hdr.size());
      _data_reals.resize(db_reals_hdr.size());

      // Pre-set a trial number
      _data_ints[0] = get_next_trial_num(database_name, table_name);

      msg = "Successful setup for MPPI logging";
      return std::pair<bool, std::string>({true, msg});;
    } else {
      msg = "Failed to create table: " + table_name;
    }
  } else {
    msg = "Failed to open database";
  }

  _logging_db = nullptr;
  _logger     = nullptr;
  return std::pair<bool, std::string>({false, msg});
}

void MppiController::log_cycle(
  const int trial_num,
  const double test_time)
{
  if(_logger) {
    std::cout << "Logging Cycle Request" << std::endl;
    _logging_flag  = true;
    _data_ints[0]  = trial_num;
    _data_reals[0] = test_time;
  }
}

void MppiController::log_data(void)
{
  uint32_t start_idx = 0;
  for(auto data : _thread_data) {
    std::copy_n(data->rbt_pos_sim.begin(), data->num_samples, _rbt_pos_saved.begin() + start_idx);
    std::copy_n(data->rbt_theta_sim.begin(), data->num_samples, _rbt_theta_saved.begin() + start_idx);
    start_idx += data->num_samples;
  }

  std::cout << "Log Data" << std::endl;
  _data_ints[1] = _log_batch;
  for(uint32_t i = 0; i < _num_samples; ++i) {
    _data_ints[2] = i;

    // Cost Components
    for(uint32_t k = 1; k < 6; ++k) { _data_reals[k] = _costs_saved[i][k-1]; }

    // Simulated robot trajectory
    for(uint32_t j = 0; j < _num_sim_steps; ++j) {
      _data_reals[6  + j*5]  = _rbt_pos_saved[i][j].x; // x_j
      _data_reals[7  + j*5]  = _rbt_pos_saved[i][j].y; // y_j
      _data_reals[8  + j*5]  = _rbt_theta_saved[i][j]; // theta_j
      _data_reals[9  + j*5]  = _u_sim[i][j][0];        // u0_j
      _data_reals[10 + j*5]  = _u_sim[i][j][1];        // u1_j
    }

    _logger->buffer_data(_data_labels, _data_ints, _data_reals);
  }

  _logging_flag = false;
  ++_log_batch;
}

}
