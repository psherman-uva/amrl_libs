#include <amrl_libs/rrt/PathTree.hpp>

#include <algorithm>
#include <set>
#include <limits>

namespace amrl {

PathTree::Node::Node(const Eigen::VectorXd &q, uint64_t id, std::shared_ptr<Node> parent)
  : q(q), id(id), edges(), parent(parent)
{
}

PathTree::Node::Node(const Eigen::VectorXd &q, uint64_t id)
  : Node(q, id, nullptr)
{
}

bool PathTree::Node::is_neighbor(std::shared_ptr<Node> node) const
{
  uint64_t test_id = node->id;

  return std::any_of(this->edges.begin(), this->edges.end(), 
    [test_id](std::shared_ptr<Node> n) { return n->id == test_id; });
}

PathTree::PathTree(void) :
  _curr_id(0),
  _root(nullptr)
{
}

PathTree::PathTree(const Eigen::VectorXd &q_init) :
  PathTree()
{
  init(q_init);
}

void PathTree::init(const Eigen::VectorXd &q_init)
{
  _curr_id = 0,
  _root    = nullptr;
  _vertices.clear();

  _root = std::make_shared<Node>(q_init, _curr_id);
  _vertices.push_back(_root);
}

std::shared_ptr<PathTree::Node> PathTree::root_node(void)
{
  return _root;
}

std::shared_ptr<PathTree::Node> PathTree::add_vertex(const Eigen::VectorXd &q)
{
  ++_curr_id;
  std::shared_ptr<Node> n_new = std::make_shared<Node>(q, _curr_id);
  _vertices.push_back(n_new);
  return n_new;
}

void PathTree::add_edge(std::shared_ptr<Node> node1, std::shared_ptr<Node> node2)
{
  if(!node1->is_neighbor(node2)) {
    node1->edges.push_back(node2);
  }

  if(!node2->is_neighbor(node1)) {
    node2->edges.push_back(node1);
  }
}

std::vector<Eigen::VectorXd> PathTree::vertices_list(void) const
{
  std::vector<Eigen::VectorXd> verts;
  for(const auto &v : _vertices) {
    verts.push_back(v->q);
  }

  return verts;
}

std::vector<std::shared_ptr<PathTree::Node>> PathTree::nodes_list(void) const
{
  return _vertices;
}

std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> PathTree::edge_list(void) const
{
  std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;

  std::set<std::pair<uint64_t, uint64_t>> check_set;
  
  for(const auto v : _vertices) {
    uint64_t id1 = v->id;
    auto q1      = v->q;
    
    for(const auto e : v->edges) { 
      uint64_t id2 = e->id;
      std::pair<uint64_t, uint64_t> pr({id1, id2});

      if (check_set.find(pr) == check_set.end()) {
        edges.push_back({q1, e->q});
        check_set.insert(pr);
        check_set.insert(std::pair<uint64_t, uint64_t>({id2, id1}));
      }
    }
  }


  return edges;
}

size_t PathTree::num_nodes(void) const
{
  return _vertices.size();
}

}

