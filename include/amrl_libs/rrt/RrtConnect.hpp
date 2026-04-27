#pragma once

#include <amrl_libs/rrt/PathTree.hpp>

#include <Eigen/Dense>
#include <functional>
#include <random>

namespace amrl {

class RrtConnect
{
private:
  enum class Result {
    Reached,
    Advanced,
    Trapped
  };

public:
  RrtConnect(
    std::vector<double> &lower_limits,
    std::vector<double> &upper_limits,
    const double d_eps,
    std::function<double(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2)> dist_func,
    std::function<bool(const Eigen::VectorXd &q)> collision_func);
  ~RrtConnect(void) = default;

  std::vector<Eigen::VectorXd> find_path(const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_goal);
  
  std::pair<std::shared_ptr<PathTree>, std::shared_ptr<PathTree>> get_trees(void) const;
  
private:
  void initialize(const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_goal);
  
  bool step(void);

  Result extend(
    std::shared_ptr<PathTree> T, 
    const Eigen::VectorXd &q,
    Eigen::VectorXd &q_new,
    bool add_if_reached);

  Result connect(std::shared_ptr<PathTree> T, const Eigen::VectorXd &q);

  bool new_config(const Eigen::VectorXd &q, const Eigen::VectorXd &q_near, Eigen::VectorXd &q_new);

  Eigen::VectorXd random_config(void); 
  std::shared_ptr<PathTree::Node> nearest_neighbor(std::shared_ptr<PathTree> T, const Eigen::VectorXd &q_test) const;


  std::vector<Eigen::VectorXd> final_path(void);

  void combine_trees(void);

  bool points_equal(
    const Eigen::VectorXd &q1, 
    const Eigen::VectorXd &q2);

  std::function<double(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2)> _dist_func;
  std::function<bool(const Eigen::VectorXd &q)> _collision_func;

  std::shared_ptr<PathTree> _Ta;
  std::shared_ptr<PathTree> _Tb;

  std::shared_ptr<PathTree::Node> _node_near;
  Eigen::VectorXd _q_attract;
  
  const double _d_eps;
  bool _swapped;
  size_t _N;

  std::default_random_engine _generator;
  std::vector<std::uniform_real_distribution<double>> _distribution;

  static constexpr double kZero     = 1.0e-6;
  static constexpr size_t kMaxIters = 10000;
};

// theta error = atan2(sin(theta2 - theta1), cos(theta2 - theta1))

}