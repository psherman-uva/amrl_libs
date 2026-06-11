#pragma once

#include <amrl_libs/rrt/PathTree.hpp>

#include <Eigen/Dense>
#include <functional>
#include <random>

namespace amrl {

class RrtStar
{
private:
  using NodePtr = std::shared_ptr<PathTree::Node>;

  enum class Result {
    Reached,
    Advanced,
    Trapped
  };

public:
  RrtStar(
    std::vector<double> &lower_limits,
    std::vector<double> &upper_limits,
    const double d_eps,
    std::function<double(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2)> dist_func,
    std::function<bool(const Eigen::VectorXd &q)> collision_func);
  
  ~RrtStar(void) = default;

  std::vector<Eigen::VectorXd> find_path(const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_goal);
  void initialize(const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_goal);
  bool step(const size_t num_iters);
  std::shared_ptr<PathTree> get_tree(void) const;
  std::vector<Eigen::VectorXd> final_path(void);

private:
  Result extend(const Eigen::VectorXd &q, Eigen::VectorXd &q_new);
  bool new_config(const Eigen::VectorXd &q, const Eigen::VectorXd &q_near, Eigen::VectorXd &q_new);
  NodePtr nearest_neighbor(const Eigen::VectorXd &q_test) const;
  std::vector<NodePtr> near_neighbors(const Eigen::VectorXd &q_test) const;
  Eigen::VectorXd random_config(void);
  bool points_equal(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2);

  std::function<double(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2)> _dist_func;
  std::function<bool(const Eigen::VectorXd &q)> _collision_func;


  Eigen::VectorXd _q_goal;

  std::shared_ptr<PathTree> _T;

  NodePtr _node_near;

  Eigen::VectorXd _q_attract;
  
  const double _d_eps;    // Step size
  double _r_search;       // Search Radius for nearest neighbors
  size_t _N;              //
  bool _path_found;

  std::default_random_engine _generator;
  std::vector<std::uniform_real_distribution<double>> _distribution;
  std::uniform_real_distribution<double> _goal_bias_dist;
  
  
  static constexpr double kGoalBiasProb = 0.05;
  static constexpr double kZero         = 1.0e-6;
  static constexpr size_t kMaxIters     = 50000;

};


}