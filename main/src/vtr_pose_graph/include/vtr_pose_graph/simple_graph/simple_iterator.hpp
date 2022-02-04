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
 * \file simple_iterator.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <unordered_map>

#include "vtr_pose_graph/simple_graph/simple_graph.hpp"
#include "vtr_pose_graph/simple_graph/simple_queue.hpp"

namespace vtr {
namespace pose_graph {
namespace simple {

/** \brief Struct that functions as the SimpleGraphIterator "value" */
struct NodeParent {
  NodeParent() = default;

  /** \brief Construct from a parent and child */
  NodeParent(const VertexId &child_, const VertexId &parent_)
      : child(child_), parent(parent_) {}
  NodeParent(VertexId &&child_, VertexId &&parent_)
      : child(child_), parent(parent_) {}

  /** \brief Construct an existing parent-child pair (SimpleEdge) */
  NodeParent(const std::pair<VertexId, VertexId> &np)
      : child(np.first), parent(np.second) {}
  NodeParent(std::pair<VertexId, VertexId> &&np)
      : child(np.first), parent(np.second) {}

  /** \brief Ordering operation, so that this can be used in a map */
  bool operator<(const NodeParent &other) const {
    return (this->child < other.child) ||
           ((this->child == other.child) && (this->parent < other.parent));
  }

  bool operator==(const NodeParent &other) const {
    return (this->child == other.child) && (this->parent == other.parent);
  }

  /** \brief Implicit conversion to Edge/Vertex Id for indexing convenience */
  operator VertexId() const { return child; }
  operator EdgeId() const { return EdgeId(child, parent); }

  /** \brief Get the VertexId of this graph element */
  VertexId v() const { return child; }
  /** \brief Get the SimpleEdge from the current vertex to its ancestor */
  EdgeId e() const { return EdgeId(child, parent); }

  VertexId child = VertexId::Invalid();
  VertexId parent = VertexId::Invalid();
};

/** \brief A generic iterator interface for simplegraph */
class SimpleGraphIterator
    : public std::iterator<std::forward_iterator_tag, const NodeParent> {
 public:
  using DepthNodeParent = std::pair<double, NodeParent>;

  using QueuePtr = typename QueueBase<DepthNodeParent>::Ptr;
  using DijkstraQueue = PriorityQueue<DepthNodeParent>;
  using DfsQueue = SimpleStack<DepthNodeParent>;
  using BfsQueue = SimpleQueue<DepthNodeParent>;

  /** \brief Construct a Dijkstra iterator */
  static SimpleGraphIterator Dijkstra(
      const SimpleGraph *graph, VertexId root, double maxDepth = 0,
      const eval::mask::Ptr &mask =
          std::make_shared<eval::mask::ConstEval>(true, true),
      const eval::weight::Ptr &weight =
          std::make_shared<eval::weight::ConstEval>(1, 1));

  /** \brief Construct a Depth First iterator */
  static SimpleGraphIterator DFS(
      const SimpleGraph *graph, VertexId root, double maxDepth = 0,
      const eval::mask::Ptr &mask =
          std::make_shared<eval::mask::ConstEval>(true, true),
      const eval::weight::Ptr &weight =
          std::make_shared<eval::weight::ConstEval>(1, 1));

  /** \brief Construct a Breadth First iterator */
  static SimpleGraphIterator BFS(
      const SimpleGraph *graph, VertexId root, double maxDepth = 0,
      const eval::mask::Ptr &mask =
          std::make_shared<eval::mask::ConstEval>(true, true),
      const eval::weight::Ptr &weight =
          std::make_shared<eval::weight::ConstEval>(1, 1));

  /** \brief Construct an empty end iterator */
  static SimpleGraphIterator End(const SimpleGraph *graph);

  /** \brief Constructor */
  SimpleGraphIterator(const SimpleGraph *graph, QueuePtr searchQueue,
                      double maxDepth = 0,
                      const eval::mask::Ptr &mask =
                          std::make_shared<eval::mask::ConstEval>(true, true),
                      const eval::weight::Ptr &weight =
                          std::make_shared<eval::weight::ConstEval>(1.f, 1.f));

  /**
   * \brief Copy constructor.  Custom implementation to deep copy the queue
   * pointer.
   */
  SimpleGraphIterator(const SimpleGraphIterator &other);
  SimpleGraphIterator(SimpleGraphIterator &&) = default;

  /**
   * \brief Copy assignment.  Custom implementation to deep copy the queue
   * pointer.
   */
  SimpleGraphIterator &operator=(const SimpleGraphIterator &other);
  SimpleGraphIterator &operator=(SimpleGraphIterator &&) = default;

  /** \brief Equality comparison */
  bool operator==(const SimpleGraphIterator &other) const;

  /** \brief Inequality comparison */
  bool operator!=(const SimpleGraphIterator &other) const;

  /** \brief Dereferencing.  Virtual due to queue interface differences */
  const NodeParent &operator*() const;

  /** \brief Dereferencing.  Virtual due to queue interface differences */
  const NodeParent *operator->() const;

  /** \brief Pre-increment */
  SimpleGraphIterator &operator++();

  /** \brief Post-increment */
  SimpleGraphIterator operator++(int);

  /** \brief Query the state of the search queue */
  inline bool empty() const { return searchQueue_->empty(); }

  /** \brief Get the top element (if it exists) or NULL */
  const NodeParent *topIfExists() const;

 private:
  /**
   * \brief Internal search function
   * \details This is the same for all search methods, we just change out the
   * queue type
   */
  void incrementInternal_();

  /**
   * \brief Determine if the top element of the queue has been seen
   * \details This is the same for all search methods, we just change out the
   * queue type
   */
  bool checkQueue_();

  /**
   * \brief Internally initialize the root element
   * \details Adds the root vertex to the search queue and depth map
   */
  void initRootInternal_(const VertexId &root);

  /** \brief Underlying SimpleGraph */
  const SimpleGraph *graph_;

  /** \brief Generic queue pointer */
  QueuePtr searchQueue_;

  /** \brief Maximum traversal depth */
  double maxDepth_;

  /** \brief Optional mask to filter out certain sections */
  eval::mask::Ptr mask_;

  /** \brief Optional weight evaluator to tune ordering */
  eval::weight::Ptr weight_;

  /** \brief Vertex weight cache */
  std::unordered_map<VertexId, double> nodeDepths_;

  /**
   * \brief Whether or not to check consistency of vertex costs on revisting
   * (false for DFS)
   */
  bool checkCosts_;
};
}  // namespace simple
}  // namespace pose_graph
}  // namespace vtr
