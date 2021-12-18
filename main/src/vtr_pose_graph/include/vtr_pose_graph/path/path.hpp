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
 * \file path.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 * \brief Path class definition
 */
#pragma once

#include "vtr_pose_graph/index/graph.hpp"
#include "vtr_pose_graph/path/accumulators.hpp"
#include "vtr_pose_graph/path/path_iterator.hpp"

namespace vtr {
namespace pose_graph {

template <class GraphT, class TF = typename GraphT::EdgeType::TransformType>
class Path {
 public:
  PTR_TYPEDEFS(Path);
  // Graph typedefs
  using GraphType = GraphT;
  using VertexId = typename GraphT::VertexIdType;
  using Sequence = typename VertexId::Vector;
  // an iterator to this path
  using Iterator = PathIterator<Path>;
  // thread safety
  using Mutex = std::recursive_mutex;
  using UniqueLock = std::unique_lock<Mutex>;
  using LockGuard = std::lock_guard<Mutex>;

  Path(const typename GraphT::Ptr& graph) : graph_(graph) {}

 public:
  /** \brief Sequence setters */
  void setSequence(const Sequence& s);
  void setSequence(Sequence&& s);

  /** \brief Verification (not verified on set for performance) */
  bool verifySequence();

  /** \brief Put the entire sequence in a single privileged integrated frame */
  void expand();
  /** \brief Put the sequence up to this id into a single privileged frame */
  void expand(unsigned seq_id);

  /** \brief Get the pose at a sequence index */
  TF pose(unsigned seq_id) const;
  /** \brief Get the pose at an iterator position */
  TF pose(const Iterator& it) const { return pose(unsigned(it)); }
  /** \brief Vertex id implicitly converts to unsigned */
  TF pose(VertexId vtx_id) const = delete;

  /** \brief Gets the cumu. distance along the path at a sequence index */
  double dist(unsigned seq_id) const;
  /** \brief Gets the cumu. distance along the path at an iterator position */
  double dist(const Iterator& it) const { return dist(unsigned(it)); }
  /** \brief Vertex id implicitly converts to unsigned */
  double dist(VertexId vtx_id) const = delete;

  /** \brief Returns the current sequence */
  Sequence sequence() const;
  /** \brief Total number of poses in the sequence */
  size_t size() const;
  /** \brief Total length of the path */
  double length() const;

  /** \brief Acquires a lock object that blocks modifications. */
  UniqueLock guard() const { return UniqueLock(mutex_); }

 protected:
  virtual void initSequence();

  /** \brief An iterator to a specified id along the path */
  Iterator begin(const unsigned& seq_id = 0) const;
  /** \brief An iterator to the end of the path (beyond the last vertex) */
  Iterator end() const;

  typename GraphT::Ptr graph_;
  Sequence sequence_;
  mutable std::vector<TF> poses_;
  mutable std::vector<double> distances_;

  /** \brief for thread safety, use whenever read from/write to the path */
  mutable Mutex mutex_;

  friend class PathIterator<Path>;
};

template <class GraphT, class TF>
void Path<GraphT, TF>::setSequence(const Sequence& s) {
  LockGuard lock(mutex_);
  sequence_ = s;
  initSequence();
}

template <class GraphT, class TF>
void Path<GraphT, TF>::setSequence(Sequence&& s) {
  LockGuard lock(mutex_);
  sequence_ = std::move(s);
  initSequence();
}

template <class GraphT, class TF>
bool Path<GraphT, TF>::verifySequence() {
  LockGuard lock(mutex_);
  // An empty sequence is fine
  if (sequence_.empty()) return true;
  // Make sure the first vertex was found OK.
  Iterator it = begin();
  if (!graph_->contains(it->to())) return false;
  for (++it; it != end(); ++it) {
    // Make sure the edge was found OK.
    if (!graph_->contains(it->to(), it->from())) return false;
  }
  return true;
}

template <class GraphT, class TF>
void Path<GraphT, TF>::expand() {
  LockGuard lock(mutex_);
  expand(sequence_.size() - 1);
}

template <class GraphT, class TF>
void Path<GraphT, TF>::expand(unsigned seq_id) {
  LockGuard lock(mutex_);
  if (seq_id >= sequence_.size())
    throw std::range_error("[Path][expand] id out of range.");
  // We've already done up to this point
  if (seq_id < poses_.size()) return;
  // Initialize if it's the first pose
  Iterator it = begin(poses_.size());
  if (poses_.empty()) {
    poses_.emplace_back(true);
    ++it;
  }
  for (; unsigned(it) <= seq_id; ++it) {
    auto&& it_pose = eval::ComposeTfAccumulator(it, it + 1, poses_.back());
    poses_.emplace_back(it_pose);
  }
}

template <class GraphT, class TF>
auto Path<GraphT, TF>::pose(unsigned seq_id) const -> TF {
  LockGuard lock(mutex_);
  if (seq_id >= sequence_.size())
    throw std::range_error("[Path][pose] id out of range.");
  const_cast<Path<GraphT>*>(this)->expand(
      seq_id);  // Cheating so we can JIT expand
  return poses_[seq_id];
}

template <class GraphT, class TF>
auto Path<GraphT, TF>::dist(unsigned seq_id) const -> double {
  LockGuard lock(mutex_);
  if (seq_id >= sequence_.size())
    throw std::range_error("[Path][dist] id out of range.");
  // We've already done up to this point
  if (seq_id < distances_.size()) return distances_[seq_id];

  // expand on demand
  auto it = begin(distances_.size());
  if (distances_.empty()) {
    distances_.emplace_back(0.);
    ++it;
  }
  for (; unsigned(it) <= seq_id; ++it) {
    const_cast<Path<GraphT>*>(this)->distances_.push_back(
        distances_.back() + it->T().r_ab_inb().norm());
  }

  return distances_[seq_id];
}

template <class GraphT, class TF>
auto Path<GraphT, TF>::sequence() const -> Sequence {
  LockGuard lock(mutex_);
  return sequence_;
}

template <class GraphT, class TF>
size_t Path<GraphT, TF>::size() const {
  LockGuard lock(mutex_);
  return sequence_.size();
}

template <class GraphT, class TF>
double Path<GraphT, TF>::length() const {
  LockGuard lock(mutex_);
  return dist(sequence_.size() - 1);
}

template <class GraphT, class TF>
void Path<GraphT, TF>::initSequence() {
  poses_.clear();
  poses_.reserve(sequence_.size());
  distances_.clear();
  distances_.reserve(sequence_.size());
}

/** \brief An iterator to a specified id along the path */
template <class GraphT, class TF>
auto Path<GraphT, TF>::begin(const unsigned& seq_id) const -> Iterator {
  if (seq_id >= sequence_.size()) return end();
  return Iterator(this, seq_id);
}

/** \brief An iterator to the end of the path (beyond the last vertex) */
template <class GraphT, class TF>
auto Path<GraphT, TF>::end() const -> Iterator {
  return Iterator(this, sequence_.end());
}

using BasicPathBase = Path<BasicGraphBase>;

}  // namespace pose_graph
}  // namespace vtr
