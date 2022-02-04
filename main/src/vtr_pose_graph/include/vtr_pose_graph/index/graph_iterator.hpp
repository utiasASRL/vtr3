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
 * \file graph_iterator.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_logging/logging.hpp"
#include "vtr_pose_graph/simple_graph/simple_iterator.hpp"

namespace vtr {
namespace pose_graph {

/** \brief Generic, proxied iterator for vertices */
template <class G>
class VertexIterator {
 public:
  using Vertex = typename G::Vertex;
  using VertexPtr = typename G::VertexPtr;

  using IterType = simple::SimpleGraph::VertexIter;

  VertexIterator(const G *graph, const IterType &internal_iter);

  VertexPtr &operator*() const;
  Vertex *operator->() const;

  VertexIterator &operator++();
  VertexIterator operator++(int);

  bool operator==(const VertexIterator &other) const;
  bool operator!=(const VertexIterator &other) const;

 private:
  const G *graph_;

  IterType internal_iter_;
};

/** \brief Generic, proxied iterator for edges */
template <class G>
class EdgeIterator {
 public:
  using Edge = typename G::Edge;
  using EdgePtr = typename G::EdgePtr;

  using IterType = simple::SimpleGraph::EdgeIter;

  EdgeIterator(const G *graph, const IterType &internal_iter);

  EdgePtr &operator*() const;
  Edge *operator->() const;

  EdgeIterator &operator++();
  EdgeIterator operator++(int);

  EdgeIterator &operator--();
  EdgeIterator operator--(int);

  bool operator==(const EdgeIterator &other) const;
  bool operator!=(const EdgeIterator &other) const;

 private:
  const G *graph_;

  IterType internal_iter_;
};

/** \brief Struct that serves as the "value type" for OrderedGraphIterator */
template <class G>
struct NodeParent {
  using VertexPtr = typename G::VertexPtr;
  using EdgePtr = typename G::EdgePtr;

  NodeParent(const G *graph = nullptr,
             const simple::NodeParent &top = simple::NodeParent())
      : graph_(graph), top_(top) {}

  /** \brief Implicit conversion to VertexId/EdgeId for indexing convenience */
  operator VertexId() const { return top_.v(); }
  operator EdgeId() const { return top_.e(); }

  /** \brief Get the referenced Vertex pointer */
  VertexPtr v() const { return graph_->at(top_.v()); }
  /** \brief Get the referenced Edge pointer */
  EdgePtr e() const { return graph_->at(top_.e()); }

  /** \brief Get the ancestor VertexId */
  VertexId from() const { return top_.parent; }
  /** \brief Get the child VertexId */
  VertexId to() const { return top_.child; }

  /** \brief Get the edge transform, properly oriented to T_prev_current */
  EdgeTransform T() const {
    if (!from().isValid() || e() == nullptr) return EdgeTransform(true);
    // The edge has T_to_from, and we want T_prev_current
    bool invert = from() == e()->from();
    // is edge from == iter prev? (then invert)
    return invert ? e()->T().inverse() : e()->T();
  }

  const G *graph_;
  simple::NodeParent top_;
};

/** \brief Iterator to a graph in search order */
template <class G>
class OrderedGraphIterator
    : std::iterator<std::forward_iterator_tag, const NodeParent<G> > {
 public:
  using IterType = simple::SimpleGraphIterator;

  OrderedGraphIterator(const G *graph, const IterType &internal_iter);

  const NodeParent<G> &operator*() const;
  const NodeParent<G> *operator->() const;

  OrderedGraphIterator &operator++();
  OrderedGraphIterator operator++(int);

  bool operator==(const OrderedGraphIterator &other) const;
  bool operator!=(const OrderedGraphIterator &other) const;

 private:
  IterType internal_iter_;
  NodeParent<G> data_;
};

}  // namespace pose_graph
}  // namespace vtr

#include "vtr_pose_graph/index/graph_iterator.inl"