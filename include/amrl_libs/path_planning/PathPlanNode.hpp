/*
  @file:      PathPlanNode.hpp
  @author:    psherman
  @date       July 2024
  
  @brief Graph vertices with links to neighbor
*/

#pragma once

#include <cstdint>
#include <memory>
#include <limits>
#include <vector>

namespace amrl {

// Forward declare Edge
struct PathPlanEdge;

// Node object for path planner
// Conatins an id and vector of edges
struct PathPlanNode
{
  uint32_t id;                     // Label for node
  std::vector<PathPlanEdge> edges; // Edges to all neighbors in graphs
  double dist_cost;                // Current best distance cost to reach node

  PathPlanNode(void) :
    PathPlanNode(0, {})
  {}
  PathPlanNode(uint32_t id, const std::vector<PathPlanEdge> &edges) :
    id(id), edges(edges), dist_cost(std::numeric_limits<double>::max())
  {}
};

/// Representation for a graph edge object.
struct PathPlanEdge
{
  std::shared_ptr<PathPlanNode> n; // Node edge points to
  double cost;                     // Cost of the edge

  PathPlanEdge(void) :
    PathPlanEdge(nullptr, 0.0)
  {}
  PathPlanEdge(std::shared_ptr<PathPlanNode> n, double cost) : 
    n(n), cost(cost)
  {}
};

/// Comparison operator for nodes.
/// Useful for use with priority queue container as part
/// of path planning algorithms
struct CmpPathNodePtrs
{
  bool operator()(const std::shared_ptr<PathPlanNode> lhs, 
    const std::shared_ptr<PathPlanNode> rhs) const
  {
    // Implementing as greater than, since we want a 
    // priority queue to pop the lowest cost first
    return lhs->dist_cost > rhs->dist_cost;
  }
};


} // namespace amrl
