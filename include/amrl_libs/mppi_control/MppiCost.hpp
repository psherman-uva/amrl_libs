/*
  @file:      MppiCost.hpp
  @author:    psherman
  @date       Oct. 2024
*/
#pragma once

#include <amrl_common/point/Point.hpp>

#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <mutex>

namespace amrl {

class MppiCost
{
public:

  MppiCost(
    const std::vector<double>& alphas,
    uint32_t num_samples,
    uint32_t num_sim_steps,
    uint32_t num_cost_components);

  virtual ~MppiCost(void) = default;

  // ----------------------------------- //
  // --    Class Public Methods       -- //
  // ----------------------------------- //

  virtual void compute_costs(
    uint32_t idx,
    const std::vector<Point<double>> &rbt_pos,
    const std::vector<double> &rbt_heading,
    const std::vector<Eigen::VectorXd> &rbt_u,
    const std::vector<Point<double>> &path) = 0;

  void costs_get(std::vector<double> &costs);

  void components_get(std::vector<std::vector<double>> &costs);

  void final_cost_operations(void);

  void min_max_cost_check(const uint32_t idx);

  void reset_min_max_costs(void);

  uint32_t num_samples_get(void) const;

  uint32_t num_sim_steps(void) const;

  uint32_t num_components_get(void) const;


  std::vector<double> min_costs_get(void) const;

  std::vector<double> max_costs_get(void) const;

  void alpha_set(size_t idx, double alpha_new);
  

protected:

  // ----------------------------------- //
  // --   Class Protected Methods     -- //
  // ----------------------------------- //


  // ----------------------------------- //
  // --     Class Member Objects      -- //
  // ----------------------------------- //

  // Alpha weights for cost function
  std::vector<double> _alphas;

  /// The computed costs for one sample
  std::vector<double> _costs;

  /// Various components of the cost function
  std::vector<std::vector<double>> _cost_components;

  /// Maximum value of costs components for one cycle
  std::vector<double> _max_costs;
  
  /// Minimum value of costs components for one cycle
  std::vector<double> _min_costs;

  /// Number of samples per cycle
  uint32_t _num_samples;

  /// Number of simulated time steps per cycle
  uint32_t _num_sim_steps;

  /// Number of cost components
  uint32_t _num_components;

  /// Mutex for thread safety
  mutable std::mutex _cost_mtx;

  /// Mutex for mission gain data
  mutable std::mutex _gain_mtx;


  // ----------------------------------- //
  // --        Class Constants        -- //
  // ----------------------------------- //

  static constexpr double kAlmostZero = 1.0e-15;
};


}