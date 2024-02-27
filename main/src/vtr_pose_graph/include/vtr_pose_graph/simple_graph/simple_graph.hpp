// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file simple_graph.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <algorithm>
#include <cstdint>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "vtr_pose_graph/evaluator_base/types.hpp"
#include "vtr_pose_graph/simple_graph/linear_component.hpp"

namespace vtr {
namespace pose_graph {

template <class V, class E>
class GraphBase;

namespace simple {

class SimpleGraphIterator;

class SimpleGraph {
 public:
  using VertexVec = std::vector<VertexId>;
  using VertexSet = std::unordered_set<VertexId>;
  using VertexList = std::list<VertexId>;
  using EdgeVec = std::vector<EdgeId>;
  using EdgeList = std::list<EdgeId>;

  /** \brief Simple node class that implements the adjacent list paradigm */
  class SimpleNode {
   public:
    SimpleNode(const VertexId &id = VertexId::Invalid()) : id_(id) {}

    VertexId getId() const { return id_; }

    void addAdjacent(const VertexId &id) { adjacent_.push_back(id); }
    const VertexList &getAdjacent() const { return adjacent_; }

   private:
    VertexId id_;
    VertexList adjacent_;
  };

  using NodeMap = std::unordered_map<VertexId, SimpleNode>;
  using BacktraceMap = std::unordered_map<VertexId, VertexId>;

  using VertexIter = NodeMap::const_iterator;
  using EdgeIter = EdgeList::const_iterator;
  using OrderedIter = SimpleGraphIterator;

  using SimpleComponent = LinearComponent<VertexId>;
  using ComponentList = std::list<SimpleComponent>;
  using JunctionSet = std::unordered_set<VertexId>;

  /** \brief Default constructor */
  SimpleGraph() = default;
  /** \brief Construct from list of edges */
  SimpleGraph(const EdgeList &edges);
  /** \brief Construct a path/cycle from a list of vertices */
  SimpleGraph(const VertexList &vertices, bool cyclic = false);

  /** \brief Add a vertex */
  void addVertex(const VertexId &vertex);
  /**
   * \brief Add an edge (inserts leaf nodes, but can only insert both if the
   * graph is fresh)
   */
  void addEdge(const EdgeId &edge);
  void addEdge(const VertexId &id1, const VertexId &id2);

  /** \brief Get the number of nodes */
  unsigned int numberOfNodes() const { return node_map_.size(); }
  /** \brief Get the number of edges */
  unsigned int numberOfEdges() const { return edges_.size(); }

  /** \brief Determine if the simplegraph contains a vertex or not */
  bool hasVertex(const VertexId &v) const {
    return node_map_.find(v) != node_map_.end();
  }

  /** \brief Determine if the simplegraph contains an edge or not */
  bool hasEdge(const EdgeId &e) const {
    if (!this->hasVertex(e.id1())) return false;
    const VertexList &adj = node_map_.at(e.id1()).getAdjacent();
    return std::find(adj.begin(), adj.end(), e.id2()) != adj.end();
  }

  /** \brief Get an Breadth-First iterator for the graph. */
  OrderedIter begin(VertexId root = VertexId::Invalid(), double max_depth = 0,
                    const eval::mask::Ptr &mask =
                        std::make_shared<eval::mask::ConstEval>(true,
                                                                true)) const;

  /** \brief Get a Breadth-First iterator for the graph */
  OrderedIter beginBfs(VertexId root = VertexId::Invalid(),
                       double max_depth = 0,
                       const eval::mask::Ptr &mask =
                           std::make_shared<eval::mask::ConstEval>(true,
                                                                   true)) const;

  /** \brief Get a Depth-First iterator for the graph */
  OrderedIter beginDfs(VertexId root = VertexId::Invalid(),
                       double max_depth = 0,
                       const eval::mask::Ptr &mask =
                           std::make_shared<eval::mask::ConstEval>(true,
                                                                   true)) const;

  /** \brief Get a Dijkstra iterator for the graph */
  OrderedIter beginDijkstra(
      VertexId root = VertexId::Invalid(), double max_depth = 0,
      const eval::mask::Ptr &mask =
          std::make_shared<eval::mask::ConstEval>(true, true),
      const eval::weight::Ptr &weight =
          std::make_shared<eval::weight::ConstEval>(1.f, 1.f)) const;

  /** \brief Get the end iterator for this graph */
  OrderedIter end() const;

  /** \brief Get an iterator to the beginning of the vertex map */
  VertexIter beginVertex() const { return node_map_.begin(); }
  /** \brief Get an iterator to the end of the vertex map */
  VertexIter endVertex() const { return node_map_.end(); }
  /** \brief Get an iterator to the beginning of the edge map */
  EdgeIter beginEdge() const { return edges_.begin(); }
  /** \brief Get an iterator to the end of the edge map */
  EdgeIter endEdge() const { return edges_.end(); }
  
  VertexVec getNodeIds() const;

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  SimpleGraph getSubgraph(
      const VertexVec &nodes,
      const eval::mask::Ptr &mask =
          std::make_shared<eval::mask::ConstEval>(true, true)) const;

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  SimpleGraph getSubgraph(VertexId root_id, const eval::mask::Ptr &mask) const;

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  SimpleGraph getSubgraph(VertexId root_id, double max_depth,
                          const eval::mask::Ptr &mask) const;

  /** \brief Merge two graphs in place, as a set union */
  SimpleGraph &operator+=(const SimpleGraph &other);

  /** \brief Merge two graphs, as a set union */
  friend SimpleGraph operator+(SimpleGraph lhs, const SimpleGraph &rhs) {
    lhs += rhs;
    return lhs;
  }

  /** \brief Use dijkstra's algorithm to traverse up to a depth */
  SimpleGraph dijkstraTraverseToDepth(
      VertexId root_id, double max_depth,
      const eval::weight::Ptr &weights =
          std::make_shared<eval::weight::ConstEval>(1, 1),
      const eval::mask::Ptr &mask =
          std::make_shared<eval::mask::ConstEval>(true, true)) const;

  /** \brief Use dijkstra's algorithm to search for an id */
  SimpleGraph dijkstraSearch(
      VertexId root_id, VertexId search_id,
      const eval::weight::Ptr &weights =
          std::make_shared<eval::weight::ConstEval>(1, 1),
      const eval::mask::Ptr &mask =
          std::make_shared<eval::mask::ConstEval>(true, true)) const;

  /**
   * \brief Use dijkstra's algorithm to search for multiple ids (weighted
   * edges)
   */
  SimpleGraph dijkstraMultiSearch(
      VertexId root_id, const VertexVec &search_ids,
      const eval::weight::Ptr &weights =
          std::make_shared<eval::weight::ConstEval>(1, 1),
      const eval::mask::Ptr &mask =
          std::make_shared<eval::mask::ConstEval>(true, true)) const;

  /** \brief Use breadth first traversal up to a depth */
  SimpleGraph breadthFirstTraversal(VertexId root_id, double max_depth) const;

  /** \brief Use breadth first traversal up to a depth */
  SimpleGraph breadthFirstTraversal(VertexId root_id, double max_depth,
                                    const eval::mask::Ptr &mask) const;

  /** \brief Use breadth first search for an id */
  SimpleGraph breadthFirstSearch(VertexId root_id, VertexId search_id) const;

  /** \brief Use breadth first search for multiple ids */
  SimpleGraph breadthFirstMultiSearch(VertexId root_id,
                                      const VertexVec &search_ids) const;

  /** \brief Get minimal spanning tree */
  SimpleGraph getMinimalSpanningTree(
      const eval::weight::Ptr &weights,
      const eval::mask::Ptr &mask =
          std::make_shared<eval::mask::ConstEval>(true, true)) const;

  /**
   * \brief Get a decomposition of the graph containing only linear, acyclic
   * components
   * \returns A list of junction/dead end vertices
   */
  JunctionSet pathDecomposition(ComponentList &paths,
                                ComponentList &cycles) const;



  /** \brief Print the structure of the graph */
  void print() const;



 private:
  /**
   * \brief Backtrace edges to root by following parents; all edges taken are
   * appended to the list of edges.
   */
  static void backtraceEdgesToRoot(const BacktraceMap &nodeParents,
                                   VertexId node, EdgeList *edges);

 private:
  /** \brief Node database */
  NodeMap node_map_;

  /** \brief List of edges */
  EdgeList edges_;

  friend class SimpleGraphIterator;

  template <class V, class E>
  friend class pose_graph::GraphBase;
};

}  // namespace simple
}  // namespace pose_graph
}  // namespace vtr
