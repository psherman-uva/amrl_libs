
#include <amrl_libs/mppi_control/MppiCost.hpp>
#include <amrl_common/util/util.hpp>

#include <limits>

namespace amrl {

MppiCost::MppiCost(
  const std::vector<double>& alphas, 
  uint32_t num_samples, 
  uint32_t num_sim_steps, 
  uint32_t num_cost_components) : 
    _alphas(alphas),
    _costs(num_samples, 0.0), 
    _cost_components(num_samples, std::vector<double>(num_cost_components, 0.0)),
    _max_costs(num_cost_components),
    _min_costs(num_cost_components),
    _num_samples(num_samples),
    _num_sim_steps(num_sim_steps),
    _num_components(num_cost_components)
{
  reset_min_max_costs();
}

void MppiCost::costs_get(std::vector<double> &costs)
{
  costs.swap(_costs);
}

void MppiCost::components_get(std::vector<std::vector<double>> &cost_components)
{
  cost_components.swap(_cost_components);
}

void MppiCost::final_cost_operations(void)
{
  // Normalization factor for each cost components
  std::vector<double> diff(_num_components);
  for(size_t i = 0; i < _num_components; ++i) {
    diff[i] = _max_costs[i] - _min_costs[i];
  }

  for(size_t idx = 0; idx < _num_samples; ++idx) {
    // Normalize each cost component and multiply by alpha weight
    for(size_t j = 0; j < _num_components; ++j) {
      if (diff[j] >= kAlmostZero) {
        _cost_components[idx][j] = _alphas[j] * (_cost_components[idx][j] - _min_costs[j]) / diff[j];
      } else {
        _cost_components[idx][j] = 0.0;
      }
    }

    // Total cost of each sample
    _costs[idx] = std::accumulate(
                    _cost_components[idx].begin(),
                    _cost_components[idx].end(), 0.0);
  }
}

void MppiCost::min_max_cost_check(const uint32_t idx)
{
  for(size_t i = 0; i < _num_components; ++i) {
    _min_costs[i] = std::min(_cost_components[idx][i], _min_costs[i]);
    _max_costs[i] = std::max(_cost_components[idx][i], _max_costs[i]);
  }
}

void MppiCost::reset_min_max_costs(void)
{
  std::fill(_min_costs.begin(), _min_costs.end(), std::numeric_limits<double>::max());
  std::fill(_max_costs.begin(), _max_costs.end(), std::numeric_limits<double>::lowest());
}

void MppiCost::alpha_set(size_t idx, double alpha_new)
{
  if(idx < _alphas.size()) {
    _alphas[idx] = alpha_new;
  }
}

std::vector<double> MppiCost::min_costs_get(void) const
{
  return _min_costs;
}

std::vector<double> MppiCost::max_costs_get(void) const
{
  return _max_costs;
}

uint32_t MppiCost::num_samples_get(void) const
{
  return _num_samples;
}

uint32_t MppiCost::num_sim_steps(void) const
{
  return _num_sim_steps;
}

uint32_t MppiCost::num_components_get(void) const
{
  return _num_components;
}

}