
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <set>

namespace amrl {

class PathTree 
{
public:
  struct Node {
    Node(const Eigen::VectorXd &q, uint64_t id, std::shared_ptr<Node> parent);
    Node(const Eigen::VectorXd &q, uint64_t id);
    bool is_neighbor(std::shared_ptr<Node> node) const;

    Eigen::VectorXd q;
    uint64_t id;
    std::vector<std::shared_ptr<Node>> edges;
    std::shared_ptr<Node> parent;
  };

  PathTree(void);
  PathTree(const Eigen::VectorXd &q_init);
  ~PathTree(void) = default;

  void init(const Eigen::VectorXd &q_init);
  std::shared_ptr<Node> root_node(void);
  std::shared_ptr<Node> add_vertex(const Eigen::VectorXd &q);
  void add_edge(std::shared_ptr<Node> node1, std::shared_ptr<Node> node2);

  std::vector<std::shared_ptr<Node>> nodes_list(void) const;
  std::vector<Eigen::VectorXd> vertices_list(void) const;
  std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edge_list(void) const;
  size_t num_nodes(void) const;

private:

  uint64_t _curr_id;
  std::shared_ptr<Node> _root;
  std::vector<std::shared_ptr<Node>> _vertices;
};

}
