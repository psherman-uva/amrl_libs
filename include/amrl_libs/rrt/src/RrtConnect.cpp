
#include <amrl_libs/rrt/RrtConnect.hpp>
#include <amrl_common/util/util.hpp>

#include <chrono>
#include <limits>
#include <iostream>

namespace amrl {


RrtConnect::RrtConnect(
    std::vector<double> &lower_limits,
    std::vector<double> &upper_limits,
    const double d_eps,
    const std::vector<std::shared_ptr<Obstacle>> &obs) :
  _Ta(std::make_shared<PathTree>()),
  _Tb(std::make_shared<PathTree>()),
  _d_eps(d_eps),
  _N(lower_limits.size()),
  _obs(obs),
  _generator(std::chrono::system_clock::now().time_since_epoch().count())
{
  for(size_t i = 0; i < _N; ++i) {
    double min_val = lower_limits[i];
    double max_val = upper_limits[i];
    _distribution.push_back(std::uniform_real_distribution<double>(min_val, max_val));
  }
}

std::vector<Eigen::VectorXd> RrtConnect::find_path(const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_goal)
{
  initialize(q_start, q_goal);

  for(size_t i = 0; i < kMaxIters; ++i) {
    if(step()) { return final_path(); }
  }

  return std::vector<Eigen::VectorXd>();
}

void RrtConnect::initialize(const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_goal)
{
  _swapped = false;
  _Ta->init(q_start);
  _Tb->init(q_goal);
}

bool RrtConnect::step(void)
{
  Eigen::VectorXd q_rand = random_config();

  if(extend(_Ta, q_rand, _q_attract, true) != Result::Trapped) {
    if(connect(_Tb, _q_attract) == Result::Reached) {
      return true;
    }
  }

  if(_Tb->num_nodes() < _Ta->num_nodes()) {
    std::swap(_Ta, _Tb);
    _swapped = !_swapped;
  }
  return false;
}

RrtConnect::Result RrtConnect::extend(
  std::shared_ptr<PathTree> T,
  const Eigen::VectorXd &q,
  Eigen::VectorXd &q_new,
  bool add_if_reached)
{
  Result result  = Result::Trapped;
  _node_near = nearest_neighbor(T, q);
  Eigen::VectorXd q_near = _node_near->q;
  
  if(new_config(q, q_near, q_new)) {
    bool pts_equal = points_equal(q, q_new);
    if(pts_equal) {
      result = Result::Reached;
    } else {
      result = Result::Advanced;
    }

    if((!pts_equal) || add_if_reached) {
      auto node_new = T->add_vertex(q_new);
      T->add_edge(_node_near, node_new);
      node_new->parent = _node_near;
    }
  }

  return result;
}

RrtConnect::Result RrtConnect::connect(std::shared_ptr<PathTree> T, const Eigen::VectorXd &q)
{
  Eigen::VectorXd q_new;
  RrtConnect::Result S = Result::Advanced;
  while (S == Result::Advanced) {
    S = extend(T, q, q_new, false);
  }
  
  return S;
}

bool RrtConnect::new_config(const Eigen::VectorXd &q, const Eigen::VectorXd &q_near, Eigen::VectorXd &q_new)
{
  Eigen::VectorXd dir = q - q_near;
  double len = dir.norm();
  if(len <= _d_eps) {
    q_new = q;
  } else {
    dir.normalize();
    q_new = q_near + (_d_eps * dir);
  }

  return !collision_check(q_new);
}


std::shared_ptr<PathTree::Node> RrtConnect::nearest_neighbor(std::shared_ptr<PathTree> T, const Eigen::VectorXd &q_test) const
{
  std::shared_ptr<PathTree::Node> node = nullptr;
  double d_min = std::numeric_limits<double>::max();

  auto nodes = T->nodes_list();
  for(const auto &nd : nodes) {
    double dist = distance_func(q_test, nd->q);

    if(dist < d_min) {
      d_min = dist;
      node  = nd;
    }
  }

  return node;
}

std::vector<Eigen::VectorXd> RrtConnect::final_path(void)
{
  std::vector<Eigen::VectorXd> path_head;
  std::vector<Eigen::VectorXd> path_tail;

  std::shared_ptr<PathTree::Node> curr_head = nearest_neighbor(_Ta, _node_near->q);
  std::shared_ptr<PathTree::Node> curr_tail = _node_near;
  if(_swapped) { std::swap(curr_head, curr_tail); }

  while(curr_head) {
    path_head.push_back(curr_head->q);
    curr_head = curr_head->parent;
  }

  while(curr_tail) {
    path_tail.push_back(curr_tail->q);
    curr_tail = curr_tail->parent;
  }

  std::reverse(path_head.begin(), path_head.end());
  path_head.insert(path_head.end(), std::make_move_iterator(path_tail.begin()), std::make_move_iterator(path_tail.end()));

  return path_head;
}


void RrtConnect::combine_trees(void)
{
  if(_swapped) { std::swap(_Ta, _Tb); }
}


bool RrtConnect::points_equal(
    const Eigen::VectorXd &q1, 
    const Eigen::VectorXd &q2)
{
  double dist = distance_func(q1, q2);
  return dist <= kZero;
}

double RrtConnect::distance_func(
    const Eigen::VectorXd &q1,
    const Eigen::VectorXd &q2) const
{
  Eigen::VectorXd q_test = q2 - q1;
  return q_test.norm();
}

bool RrtConnect::collision_check(const Eigen::VectorXd &q) const
{
  return std::any_of(_obs.begin(), _obs.end(), [&q] (const std::shared_ptr<Obstacle> o)
    { return o->is_inside({q[0], q[1]}); });
}

Eigen::VectorXd RrtConnect::random_config(void)
{
  std::vector<double> data(_N);
  for(size_t i = 0; i < _N; ++i) {
    data[i] = _distribution[i](_generator);
  }
  return Eigen::Map<Eigen::VectorXd>(data.data(), data.size());
}

std::pair<std::shared_ptr<PathTree>, std::shared_ptr<PathTree>> RrtConnect::get_trees(void) const
{
  if (_swapped) {
    return std::pair<std::shared_ptr<PathTree>, std::shared_ptr<PathTree>>(_Tb, _Ta);
  }
  return std::pair<std::shared_ptr<PathTree>, std::shared_ptr<PathTree>>(_Ta, _Tb);
}


}