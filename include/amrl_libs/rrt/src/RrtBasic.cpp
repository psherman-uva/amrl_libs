
#include <amrl_libs/rrt/RrtBasic.hpp>
#include <amrl_common/util/util.hpp>

#include <chrono>
#include <limits>
#include <iostream>

namespace amrl {


RrtBasic::RrtBasic(
    std::vector<double> &lower_limits,
    std::vector<double> &upper_limits,
    const double d_eps,
    std::function<double(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2)> dist_func,
    std::function<bool(const Eigen::VectorXd &q)> collision_func) :
  _T(std::make_shared<PathTree>()),
  _d_eps(d_eps),
  _N(lower_limits.size()),
  _dist_func(std::move(dist_func)),
  _collision_func(std::move(collision_func)),
  _generator(std::chrono::system_clock::now().time_since_epoch().count())
{
  for(size_t i = 0; i < _N; ++i) {
    double min_val = lower_limits[i];
    double max_val = upper_limits[i];
    _distribution.push_back(std::uniform_real_distribution<double>(min_val, max_val));
  }
}
  
std::shared_ptr<PathTree> RrtBasic::get_tree(void) const
{
  return _T;
}

void RrtBasic::initialize(const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_goal)
{
  _q_goal = q_goal;
  _T->init(q_start);
}

std::vector<Eigen::VectorXd> RrtBasic::find_path(const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_goal)
{
  initialize(q_start, q_goal);

  for(size_t i = 0; i < kMaxIters; ++i) {
    if(step()) { return final_path(); }
  }

  return std::vector<Eigen::VectorXd>();
}

bool RrtBasic::step(void)
{
  Eigen::VectorXd q_rand = random_config();
  Eigen::VectorXd q_new;

  if (extend(q_rand, q_new) != Result::Trapped) {
    double dist = _dist_func(q_new, _q_goal);
    return dist <= _d_eps;
  }
  return false;
}

RrtBasic::Result RrtBasic::extend(const Eigen::VectorXd &q, Eigen::VectorXd &q_new)
{
  Result result = Result::Trapped;
  std::shared_ptr<PathTree::Node> node_near = nearest_neighbor(q);
  Eigen::VectorXd q_near = node_near->q;
  
  if(new_config(q, q_near, q_new)) {
    bool pts_equal = points_equal(q, q_new);
    if(pts_equal) { result = Result::Reached; } 
    else          { result = Result::Advanced; }

    auto node_new = _T->add_vertex(q_new);
    _T->add_edge(node_near, node_new);
    node_new->parent = node_near;
  }

  return result;
}

bool RrtBasic::new_config(const Eigen::VectorXd &q, const Eigen::VectorXd &q_near, Eigen::VectorXd &q_new)
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

std::shared_ptr<PathTree::Node> RrtBasic::nearest_neighbor(const Eigen::VectorXd &q_test) const
{
  std::shared_ptr<PathTree::Node> node = nullptr;
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

std::vector<Eigen::VectorXd> RrtBasic::final_path(void)
{
  std::vector<Eigen::VectorXd> path({_q_goal});
  
  std::shared_ptr<PathTree::Node> nd = nearest_neighbor(_q_goal);
  while(nd) {
    path.push_back(nd->q);
    nd = nd->parent;
  }
  
  std::reverse(path.begin(), path.end());
  return path;
}

Eigen::VectorXd RrtBasic::random_config(void)
{
  std::vector<double> data(_N);
  for(size_t i = 0; i < _N; ++i) {
    data[i] = _distribution[i](_generator);
  }
  return Eigen::Map<Eigen::VectorXd>(data.data(), data.size());
}

bool RrtBasic::points_equal(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2)
{
  double dist = _dist_func(q1, q2);
  return dist <= kZero;
}

}
