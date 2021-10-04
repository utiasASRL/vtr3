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
 * \file rc_graph_base.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_common/utils/lockable.hpp>
#include <vtr_pose_graph/index/graph_base.hpp>
#include <vtr_pose_graph/serializable/rc_edge.hpp>
#include <vtr_pose_graph/serializable/rc_run.hpp>
#include <vtr_pose_graph/serializable/rc_vertex.hpp>

namespace vtr {
namespace pose_graph {

class RCGraphBase : public virtual GraphBase<RCVertex, RCEdge, RCRun> {
 public:
  using Base = GraphBase<RCVertex, RCEdge, RCRun>;
  using RType = RCGraphBase;

  using Base::edges_;
  using Base::graph_;
  using Base::runs_;
  using Base::vertices_;

  using PersistentIdType = RCVertex::PersistentIdType;

  /** \brief Shared pointer type definitions for this class */
  PTR_TYPEDEFS(RCGraphBase)

  /** \brief Pseudo-constructor to make shared pointers */
  static Ptr MakeShared() { return Ptr(new RCGraphBase()); }

  RCGraphBase() : GraphBase<RCVertex, RCEdge, RCRun>() {}

#if false
  RCGraphBase(const RCGraphBase&) = default;
  RCGraphBase(RCGraphBase&& other)
      : Base(std::move(other)),
        persistent_map_(std::move(other.persistent_map_)) {}

  RCGraphBase& operator=(const RCGraphBase&) = default;
  RCGraphBase& operator=(RCGraphBase&& other) {
    Base::operator=(std::move(other));
    this->persistent_map_ = std::move(other.persistent_map_);
    return *this;
  }
#endif

  // Get the persistent id from this vertex id (unchanged on graph refactor)
  PersistentIdType toPersistent(const VertexIdType& vid) const;

  // Get the vertex id from persistent id (unchanged on graph refactor)
  VertexIdType fromPersistent(const PersistentIdType& pid) const;

  /**
   * The graph functions are given thin re-implementations to handle casting
   */

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const typename VertexIdType::Vector& nodes) const {
    std::shared_lock lock(simple_graph_mutex_);
    return MakeShared(*this, graph_.getSubgraph(makeSimple(nodes)));
  }

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const VertexIdType& rootId,
                  const eval::Mask::Ptr& mask) const {
    std::shared_lock lock(simple_graph_mutex_);
    return MakeShared(*this, graph_.getSubgraph(rootId, mask));
  }

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const VertexIdType& rootId, double maxDepth,
                  const eval::Mask::Ptr& mask) const {
    std::shared_lock lock(simple_graph_mutex_);
    return MakeShared(*this, graph_.getSubgraph(rootId, maxDepth, mask));
  }

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const eval::Mask::Ptr& mask) const {
    for (auto it = this->beginVertex(); it != this->endVertex(); ++it) {
      if (mask->operator[](it->id())) return this->getSubgraph(it->id(), mask);
    }
    return MakeShared(*this, SimpleGraph());
  }

#if false
  /** \brief Convenience function to get a manual subgraph, left here for reference. */
  Ptr getManualSubgraph();
#endif

  /** \brief Get the induced subgraph of another subgraph */
  inline Ptr induced(const RCGraphBase& subgraph) const {
    // lock simple graph of both simutaneously
    std::shared_lock lock1(simple_graph_mutex_, std::defer_lock);
    std::shared_lock lock2(subgraph.simple_graph_mutex_, std::defer_lock);
    std::lock(lock1, lock2);
    return Ptr(new RCGraphBase(*this, graph_.induced(subgraph.graph_)));
  }

  /** \brief Merge two graphs in place, as a set union */
  RCGraphBase& operator+=(const RCGraphBase& other) {
    // lock simple graph of both simutaneously
    std::shared_lock lock1(simple_graph_mutex_, std::defer_lock);
    std::shared_lock lock2(other.simple_graph_mutex_, std::defer_lock);
    std::lock(lock1, lock2);

    graph_ += other.graph_;
    return *this;
  }
#if false
  /** \brief Merge two graphs, as a set union */
  friend RCGraphBase operator+(RCGraphBase lhs, const RCGraphBase& rhs) {
    lhs += rhs;
    return lhs;
  }
#endif
  /** \brief Merge two graphs, as a set union */
  Ptr setUnion(const Ptr& other) const {
    // lock simple graph of both simutaneously
    std::shared_lock lock1(simple_graph_mutex_, std::defer_lock);
    std::shared_lock lock2(other->simple_graph_mutex_, std::defer_lock);
    std::lock(lock1, lock2);

    return Ptr(new RCGraphBase(*this, graph_ + other->graph_));
  }

  /** \brief Use dijkstra's algorithm to traverse up to a depth (weighted edges)
   */
  Ptr dijkstraTraverseToDepth(
      const VertexIdType& rootId, double maxDepth,
      const eval::Weight::Ptr& weights = eval::Weight::Const::MakeShared(1, 1),
      const eval::Mask::Ptr& mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const {
    std::shared_lock lock(simple_graph_mutex_);
    return MakeShared(
        *this, graph_.dijkstraTraverseToDepth(rootId, maxDepth, weights, mask));
  }

  /** \brief Use dijkstra's algorithm to search for an id (weighted edges) */
  Ptr dijkstraSearch(
      const VertexIdType& rootId, VertexIdType searchId,
      const eval::Weight::Ptr& weights = eval::Weight::Const::MakeShared(1, 1),
      const eval::Mask::Ptr& mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const {
    std::shared_lock lock(simple_graph_mutex_);
    return MakeShared(*this,
                      graph_.dijkstraSearch(rootId, searchId, weights, mask));
  }

  /**
   * \brief Use dijkstra's algorithm to search for multiple ids (weighted
   * edges)
   */
  Ptr dijkstraMultiSearch(
      const VertexIdType& rootId,
      const typename VertexIdType::Vector& searchIds,
      const eval::Weight::Ptr& weights = eval::Weight::Const::MakeShared(1, 1),
      const eval::Mask::Ptr& mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const {
    std::shared_lock lock(simple_graph_mutex_);
    return MakeShared(*this, graph_.dijkstraMultiSearch(
                                 rootId, makeSimple(searchIds), weights, mask));
  }

  /** \brief Use breadth first traversal up to a depth */
  Ptr breadthFirstTraversal(const VertexIdType& rootId, double maxDepth) const {
    std::shared_lock lock(simple_graph_mutex_);
    return MakeShared(*this, graph_.breadthFirstTraversal(rootId, maxDepth));
  }

  /** \brief Use breadth first search for an id */
  Ptr breadthFirstSearch(const VertexIdType& rootId,
                         VertexIdType searchId) const {
    std::shared_lock lock(simple_graph_mutex_);
    return MakeShared(*this, graph_.breadthFirstSearch(rootId, searchId));
  }

  /** \brief Use breadth first search for multiple ids */
  Ptr breadthFirstMultiSearch(
      const VertexIdType& rootId,
      const typename VertexIdType::Vector& searchIds) const {
    std::shared_lock lock(simple_graph_mutex_);
    return MakeShared(
        *this, graph_.breadthFirstMultiSearch(rootId, makeSimple(searchIds)));
  }

  /** \brief Get minimal spanning tree */
  Ptr getMinimalSpanningTree(
      const eval::Weight::Ptr& weights,
      const eval::Mask::Ptr& mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const {
    std::shared_lock lock(simple_graph_mutex_);
    return MakeShared(*this, graph_.getMinimalSpanningTree(weights, mask));
  }

 protected:
  static Ptr MakeShared(const RCGraphBase& other, const SimpleGraph& graph) {
    return Ptr(new RCGraphBase(other, graph));
  }
  static Ptr MakeShared(const RCGraphBase& other, SimpleGraph&& graph) {
    return Ptr(new RCGraphBase(other, std::move(graph)));
  }

  RCGraphBase(const RCGraphBase& other, const SimpleGraph& graph)
      : GraphBase<RCVertex, RCEdge, RCRun>(other, graph),
        persistent_map_(other.persistent_map_) {}
  RCGraphBase(const RCGraphBase& other, SimpleGraph&& graph)
      : GraphBase<RCVertex, RCEdge, RCRun>(other, std::move(graph)),
        persistent_map_(std::move(other.persistent_map_)) {}

 protected:
  using PersistentMap = std::unordered_map<PersistentIdType, VertexIdType>;
  // A map from persistent id to vertex id for long-lasting streams indexing
  // into a changing graph, currently persistent id is the keyframe time
  std::shared_ptr<common::SharedLockable<PersistentMap>> persistent_map_ =
      std::make_shared<common::SharedLockable<PersistentMap>>();
};

}  // namespace pose_graph
}  // namespace vtr
