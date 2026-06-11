
#include <amrl_libs/rrt/RrtStar.hpp>
#include <amrl_common/util/util.hpp>

#include <chrono>
#include <limits>
#include <iostream>

namespace amrl {


RrtStar::RrtStar(
    std::vector<double> &lower_limits,
    std::vector<double> &upper_limits,
    const double d_eps,
    std::function<double(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2)> dist_func,
    std::function<bool(const Eigen::VectorXd &q)> collision_func) :
  _T(std::make_shared<PathTree>()),
  _d_eps(d_eps),
  _r_search(4*d_eps),
  _N(lower_limits.size()),
  _dist_func(std::move(dist_func)),
  _collision_func(std::move(collision_func)),
  _generator(std::chrono::system_clock::now().time_since_epoch().count()),
  _goal_bias_dist()
{
  for(size_t i = 0; i < _N; ++i) {
    double min_val = lower_limits[i];
    double max_val = upper_limits[i];
    _distribution.push_back(std::uniform_real_distribution<double>(min_val, max_val));
  }
}

std::shared_ptr<PathTree> RrtStar::get_tree(void) const
{
  return _T;
}

void RrtStar::initialize(const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_goal)
{
  _T->init(q_start);
  _path_found = false;
  _q_goal     = q_goal;
}

std::vector<Eigen::VectorXd> RrtStar::find_path(const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_goal)
{
  initialize(q_start, q_goal);

  for(size_t i = 0; i < kMaxIters; ++i) {
    if(step(1)) { return final_path(); }
  }

  return std::vector<Eigen::VectorXd>();
}

bool RrtStar::step(const size_t num_iters )
{
  for (size_t i = 0; i < num_iters; ++i) {
    Eigen::VectorXd q_rand = random_config();
    Eigen::VectorXd q_new;
  
    if (extend(q_rand, q_new) != Result::Trapped) {
      double dist = _dist_func(q_new, _q_goal);
      if(dist <= _d_eps) {
        _path_found = true;
      }
    }
  }

  return _path_found;
}

RrtStar::Result RrtStar::extend(const Eigen::VectorXd &q, Eigen::VectorXd &q_new)
{
  Result result = Result::Trapped;
  NodePtr node_near = nearest_neighbor(q);
  Eigen::VectorXd q_near = node_near->q;
  
  if(new_config(q, q_near, q_new)) {
    bool pts_equal = points_equal(q, q_new);
    if(pts_equal) {
      result = Result::Reached;
    } else {
      result = Result::Advanced;
    }

    // Pick the "Best" parent node. Not just the nearest node.
    auto Q_near = near_neighbors(q_new);

    NodePtr node_best = node_near;
    double cost_best  = node_near->cost + _dist_func(q_new, node_near->q);

    for(auto nd_near : Q_near) {
      double cost_potential = nd_near->cost + _dist_func(q_new, nd_near->q);
      if(cost_potential < cost_best) {
        node_best = nd_near;
        cost_best = cost_potential;
      }
    }
    
    NodePtr node_new = _T->add_vertex(q_new);
    _T->add_edge(node_best, node_new);
    node_new->parent = node_best;
    node_new->cost   = node_best->cost + _dist_func(q_new, node_best->q);

    // Rewiring
    for(auto nd_near : Q_near) {
      if (nd_near->id != node_best->id) {
        double cost_rewire = node_new->cost + _dist_func(node_new->q, nd_near->q);
        if(cost_rewire < nd_near->cost) {
          _T->remove_edge(nd_near, nd_near->parent);
          _T->add_edge(nd_near, node_new);
          nd_near->cost   = cost_rewire;
          nd_near->parent = node_new; 
        }
      } 
    }
  }

  return result;
}

bool RrtStar::new_config(const Eigen::VectorXd &q, const Eigen::VectorXd &q_near, Eigen::VectorXd &q_new)
{
  Eigen::VectorXd dir = q - q_near;
  double len = _dist_func(q_near, q);
  if(len <= _d_eps) {
    q_new = q;
  } else {
    dir /= len;
    q_new = q_near + (_d_eps * dir);
  }

  return !_collision_func(q_new);
}

RrtStar::NodePtr RrtStar::nearest_neighbor(const Eigen::VectorXd &q_test) const
{
  NodePtr node = nullptr;
  double d_min = std::numeric_limits<double>::max();

  auto nodes = _T->nodes_list();
  for(const auto &nd : nodes) {
    double dist = _dist_func(q_test, nd->q);
    if(dist < d_min) {
      d_min = dist;
      node  = nd;
    }
  }

  return node;
}

std::vector<RrtStar::NodePtr> RrtStar::near_neighbors(const Eigen::VectorXd &q_test) const
{
  std::vector<NodePtr> neighbors;

  auto nodes = _T->nodes_list();
  for(const auto &nd : nodes) {
    if(_dist_func(q_test, nd->q) <= _r_search) {
      neighbors.push_back(nd);
    }
  }

  return neighbors;
}

std::vector<Eigen::VectorXd> RrtStar::final_path(void)
{
  if(!_path_found) { return std::vector<Eigen::VectorXd>(); }

  std::vector<Eigen::VectorXd> path({_q_goal});
  NodePtr nd = nearest_neighbor(_q_goal);
  std::cout << "Cost: " << nd->cost << std::endl;
  while(nd) {
    path.push_back(nd->q);
    nd = nd->parent;
  }
  
  std::reverse(path.begin(), path.end());
  return path;
}

Eigen::VectorXd RrtStar::random_config(void)
{
  if(_goal_bias_dist(_generator) <= kGoalBiasProb) { return _q_goal; } 

  std::vector<double> data(_N);
  for(size_t i = 0; i < _N; ++i) {
    data[i] = _distribution[i](_generator);
  }
  return Eigen::Map<Eigen::VectorXd>(data.data(), data.size());
}

bool RrtStar::points_equal(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2)
{
  double dist = _dist_func(q1, q2);
  return dist <= kZero;
}

}
