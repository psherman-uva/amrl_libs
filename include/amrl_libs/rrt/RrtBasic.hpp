#pragma once

#include <amrl_libs/rrt/PathTree.hpp>

#include <Eigen/Dense>
#include <functional>
#include <random>

namespace amrl {

class RrtBasic
{
private:
  enum class Result {
    Reached,
    Advanced,
    Trapped
  };

public:
  RrtBasic(
    std::vector<double> &lower_limits,
    std::vector<double> &upper_limits,
    const double d_eps,
    std::function<double(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2)> dist_func,
    std::function<bool(const Eigen::VectorXd &q)> collision_func);
  
  ~RrtBasic(void) = default;

  std::vector<Eigen::VectorXd> find_path(const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_goal);
  void initialize(const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_goal);
  bool step(void);
  std::shared_ptr<PathTree> get_tree(void) const;
  std::vector<Eigen::VectorXd> final_path(void);

private:

  Result extend(const Eigen::VectorXd &q, Eigen::VectorXd &q_new);
  bool new_config(const Eigen::VectorXd &q, const Eigen::VectorXd &q_near, Eigen::VectorXd &q_new);
  std::shared_ptr<PathTree::Node> nearest_neighbor(const Eigen::VectorXd &q_test) const;
  Eigen::VectorXd random_config(void);
  bool points_equal(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2);

  std::function<double(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2)> _dist_func;
  std::function<bool(const Eigen::VectorXd &q)> _collision_func;


  Eigen::VectorXd _q_goal;

  std::shared_ptr<PathTree> _T;

  std::shared_ptr<PathTree::Node> _node_near;

  Eigen::VectorXd _q_attract;
  
  const double _d_eps;
  size_t _N;

  std::default_random_engine _generator;
  std::vector<std::uniform_real_distribution<double>> _distribution;

  static constexpr double kZero     = 1.0e-6;
  static constexpr size_t kMaxIters = 100000;

};


}