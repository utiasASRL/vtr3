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
 * \file graph_base.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/index/edge_base.hpp"
#include "vtr_pose_graph/index/graph_iterator.hpp"
#include "vtr_pose_graph/index/vertex_base.hpp"
#include "vtr_pose_graph/simple_graph/simple_graph.hpp"

namespace vtr {
namespace pose_graph {

template <class V, class E>
class GraphBase {
 public:
  PTR_TYPEDEFS(GraphBase);

  using Base = GraphBase<V, E>;

  using SimpleGraph = simple::SimpleGraph;
  using JunctionSet = SimpleGraph::JunctionSet;
  using ComponentList = SimpleGraph::ComponentList;

  using Vertex = V;
  using VertexPtr = typename V::Ptr;

  using Edge = E;
  using EdgePtr = typename E::Ptr;

  // Internal mapping between SimpleGraph and our data types
  using VertexMap = std::unordered_map<VertexId, VertexPtr>;
  using EdgeMap = std::unordered_map<EdgeId, EdgePtr>;

  // Proxied iterators
  using VertexIter = VertexIterator<Base>;
  using EdgeIter = EdgeIterator<Base>;
  using OrderedIter = OrderedGraphIterator<Base>;

  /** \brief Pseudo-constructor to make shared pointers */
  static Ptr MakeShared() { return std::make_shared<GraphBase>(); }

  static Ptr MakeShared(const GraphBase& other, const SimpleGraph& graph) {
    return std::make_shared<GraphBase>(other, graph);
  }
  static Ptr MakeShared(const GraphBase& other, SimpleGraph&& graph) {
    return std::make_shared<GraphBase>(other, graph);
  }

  /** \brief Only constructor exposed to initialize the pose graph */
  GraphBase() = default;

  /** \brief Constructor to create subgraphs */
  GraphBase(const GraphBase& other, const SimpleGraph& graph);
  /** \brief Constructor to create subgraphs, using move on the structure */
  GraphBase(const GraphBase& other, SimpleGraph&& graph);

  virtual ~GraphBase() = default;

 public:
  /** Get the number of vertices */
  unsigned int numberOfVertices() const {
    std::shared_lock lock(mutex_);
    return graph_.numberOfNodes();
  }

  /** \brief Get the number of edges */
  unsigned int numberOfEdges() const {
    std::shared_lock lock(mutex_);
    return graph_.numberOfEdges();
  }

  /** \brief Determine if this graph/subgraph contains a specific vertex */
  bool contains(const VertexId& v) const {
    std::shared_lock lock(mutex_);
    return graph_.hasVertex(v);
  }

  /** \brief Determine if this graph/subgraph contains a specific edge */
  bool contains(const EdgeId& e) const {
    std::shared_lock lock(mutex_);
    return graph_.hasEdge(e);
  }

  /** \brief Const map interface for vertices */
  VertexPtr at(const VertexId& v) const {
    std::shared_lock lock(mutex_);
    try {
      return vertices_.at(v);
    } catch (...) {
      std::stringstream err;
      err << "Could not find " << v << " in the graph.";
      CLOG(ERROR, "pose_graph") << err.str();
      throw std::range_error(err.str());
    }
    // just so it compiles...
    return vertices_.at(v);
  }

  /** \brief Const map interface for edges */
  EdgePtr at(const EdgeId& e) const {
    std::shared_lock lock(mutex_);
    try {
      return edges_.at(e);
    } catch (...) {
      std::stringstream err;
      err << "Could not find " << e << " in the graph.";
      CLOG(ERROR, "pose_graph") << err.str();
      throw std::range_error(err.str());
    }
    // just so it compiles...
    return edges_.at(e);
  }

  /** \brief Get the neighbors of a vertex that are in this graph */
  VertexId::Set neighbors(const VertexId& v) const;

  /** \brief Get an iterator for the graph. Defaults to BFS. */
  OrderedIter begin(VertexId root = VertexId::Invalid(), double max_depth = 0,
                    const eval::mask::Ptr& mask =
                        std::make_shared<eval::mask::ConstEval>(true,
                                                                true)) const {
    return OrderedIter(this, graph_.begin(root, max_depth, mask));
  }

  /** \brief Get a Breadth-First iterator for the graph */
  OrderedIter beginBfs(
      VertexId root = VertexId::Invalid(), double max_depth = 0,
      const eval::mask::Ptr& mask =
          std::make_shared<eval::mask::ConstEval>(true, true)) const {
    return OrderedIter(this, graph_.beginBfs(root, max_depth, mask));
  }

  /** \brief Get a Depth-First iterator for the graph */
  OrderedIter beginDfs(
      VertexId root = VertexId::Invalid(), double max_depth = 0,
      const eval::mask::Ptr& mask =
          std::make_shared<eval::mask::ConstEval>(true, true)) const {
    return OrderedIter(this, graph_.beginDfs(root, max_depth, mask));
  }

  /** \brief Get a Dijkstra iterator for the graph */
  OrderedIter beginDijkstra(
      VertexId root = VertexId::Invalid(), double max_depth = 0,
      const eval::mask::Ptr& mask =
          std::make_shared<eval::mask::ConstEval>(true, true),
      const eval::weight::Ptr& weight =
          std::make_shared<eval::weight::ConstEval>(1.f, 1.f)) const {
    return OrderedIter(this,
                       graph_.beginDijkstra(root, max_depth, mask, weight));
  }

  /** \brief Get the end iterator for this graph */
  OrderedIter end() const { return OrderedIter(this, graph_.end()); }

  // clang-format off
  /** \brief Iterator interface to all vertices in this subgraph */
  VertexIter beginVertex() const { return VertexIter(this, graph_.beginVertex()); }
  /** \brief End iterator for all vertices in this subgraph */
  VertexIter endVertex() const { return VertexIter(this, graph_.endVertex()); }
  /** \brief Iterator interface to all vertices in this subgraph */
  EdgeIter beginEdge() const { return EdgeIter(this, graph_.beginEdge()); }
  /** \brief End iterator for all vertices in this subgraph */
  EdgeIter endEdge() const { return EdgeIter(this, graph_.endEdge()); }
  // clang-format on

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const VertexId::Vector& nodes) const {
    std::shared_lock lock(mutex_);
    return MakeShared(*this, graph_.getSubgraph(nodes));
  }

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const VertexId& root_id, const eval::mask::Ptr& mask) const {
    std::shared_lock lock(mutex_);
    return MakeShared(*this, graph_.getSubgraph(root_id, mask));
  }

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges) to a max depth
   */
  Ptr getSubgraph(const VertexId& root_id, double max_depth,
                  const eval::mask::Ptr& mask) const {
    std::shared_lock lock(mutex_);
    return MakeShared(*this, graph_.getSubgraph(root_id, max_depth, mask));
  }

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const eval::mask::Ptr& mask) const {
    std::shared_lock lock(mutex_);
    for (auto it = graph_.beginVertex(); it != graph_.endVertex(); ++it) {
      if (mask->operator[](it->first))
        return MakeShared(*this, graph_.getSubgraph(it->first, mask));
    }
    return MakeShared(*this, SimpleGraph());
  }

  /** \brief Use dijkstra's alg to traverse up to a depth (weighted edges) */
  Ptr dijkstraTraverseToDepth(
      const VertexId& root_id, double max_depth,
      const eval::weight::Ptr& weights =
          std::make_shared<eval::weight::ConstEval>(1, 1),
      const eval::mask::Ptr& mask =
          std::make_shared<eval::mask::ConstEval>(true, true)) const {
    std::shared_lock lock(mutex_);
    return MakeShared(*this, graph_.dijkstraTraverseToDepth(root_id, max_depth,
                                                            weights, mask));
  }

  /** \brief Use dijkstra's algorithm to search for an id (weighted edges) */
  Ptr dijkstraSearch(const VertexId& root_id, VertexId search_id,
                     const eval::weight::Ptr& weights =
                         std::make_shared<eval::weight::ConstEval>(1, 1),
                     const eval::mask::Ptr& mask =
                         std::make_shared<eval::mask::ConstEval>(true,
                                                                 true)) const {
    std::shared_lock lock(mutex_);
    return MakeShared(*this,
                      graph_.dijkstraSearch(root_id, search_id, weights, mask));
  }

  /**
   * \brief Use dijkstra's algorithm to search for multiple ids (weighted
   * edges)
   */
  Ptr dijkstraMultiSearch(
      const VertexId& root_id, const typename VertexId::Vector& search_ids,
      const eval::weight::Ptr& weights =
          std::make_shared<eval::weight::ConstEval>(1, 1),
      const eval::mask::Ptr& mask =
          std::make_shared<eval::mask::ConstEval>(true, true)) const {
    std::shared_lock lock(mutex_);
    return MakeShared(
        *this, graph_.dijkstraMultiSearch(root_id, search_ids, weights, mask));
  }

  /** \brief Use breadth first traversal up to a depth */
  Ptr breadthFirstTraversal(const VertexId& root_id, double max_depth) const {
    std::shared_lock lock(mutex_);
    return MakeShared(*this, graph_.breadthFirstTraversal(root_id, max_depth));
  }

  /** \brief Use breadth first search for an id */
  Ptr breadthFirstSearch(const VertexId& root_id, VertexId search_id) const {
    std::shared_lock lock(mutex_);
    return MakeShared(*this, graph_.breadthFirstSearch(root_id, search_id));
  }

  /** \brief Use breadth first search for multiple ids */
  Ptr breadthFirstMultiSearch(
      const VertexId& root_id,
      const typename VertexId::Vector& search_ids) const {
    std::shared_lock lock(mutex_);
    return MakeShared(*this,
                      graph_.breadthFirstMultiSearch(root_id, search_ids));
  }

  /** \brief Get minimal spanning tree */
  Ptr getMinimalSpanningTree(
      const eval::weight::Ptr& weights,
      const eval::mask::Ptr& mask =
          std::make_shared<eval::mask::ConstEval>(true, true)) const {
    std::shared_lock lock(mutex_);
    return MakeShared(*this, graph_.getMinimalSpanningTree(weights, mask));
  }

  /**
   * \brief Get a decomposition of the graph containing only linear, acyclic
   * components
   * \returns A list of junction/dead end vertices
   */
  JunctionSet pathDecomposition(ComponentList& paths,
                                ComponentList& cycles) const;

 protected:
  /** \brief protects access to graph_, vertices_ and edges_ */
  mutable std::shared_mutex mutex_;

  /** \brief SimpleGraph object to hold the structure */
  SimpleGraph graph_;

  /** \brief Map from SimpleVertexId to vertex object */
  VertexMap vertices_;

  /** \brief Map from SimpleEdgeId to edge object */
  EdgeMap edges_;
};

extern template class GraphBase<VertexBase, EdgeBase>;
using BasicGraphBase = GraphBase<VertexBase, EdgeBase>;

}  // namespace pose_graph
}  // namespace vtr

#include "vtr_pose_graph/index/graph_base.inl"