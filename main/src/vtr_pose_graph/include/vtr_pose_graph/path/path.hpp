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
 */
#pragma once

#include "vtr_pose_graph/index/graph.hpp"
#include "vtr_pose_graph/path/accumulators.hpp"
#include "vtr_pose_graph/path/path_iterator.hpp"

namespace vtr {
namespace pose_graph {

template <class GraphT>
class Path {
 public:
  PTR_TYPEDEFS(Path);
  // Graph typedefs
  using GraphType = GraphT;
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
  EdgeTransform pose(unsigned seq_id) const;
  /** \brief Get the pose at an iterator position */
  EdgeTransform pose(const Iterator& it) const { return pose(unsigned(it)); }
  /** \brief Vertex id implicitly converts to unsigned */
  EdgeTransform pose(VertexId vtx_id) const = delete;

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
  mutable std::vector<EdgeTransform> poses_;
  mutable std::vector<double> distances_;

  /** \brief for thread safety, use whenever read from/write to the path */
  mutable Mutex mutex_;

  friend class PathIterator<Path>;
};

template <class GraphT>
void Path<GraphT>::setSequence(const Sequence& s) {
  LockGuard lock(mutex_);
  sequence_ = s;
  initSequence();
}

template <class GraphT>
void Path<GraphT>::setSequence(Sequence&& s) {
  LockGuard lock(mutex_);
  sequence_ = std::move(s);
  initSequence();
}

template <class GraphT>
bool Path<GraphT>::verifySequence() {
  LockGuard lock(mutex_);
  // An empty sequence is fine
  if (sequence_.empty()) return true;
  // Make sure the first vertex was found OK.
  Iterator it = begin();
  if (!graph_->contains(it->to())) return false;
  for (++it; it != end(); ++it) {
    // Make sure the edge was found OK.
    if (!graph_->contains(EdgeId(it->to(), it->from()))) return false;
  }
  return true;
}

template <class GraphT>
void Path<GraphT>::expand() {
  LockGuard lock(mutex_);
  expand(sequence_.size() - 1);
}

template <class GraphT>
void Path<GraphT>::expand(unsigned seq_id) {
  LockGuard lock(mutex_);
  if (seq_id >= sequence_.size()) {
    std::string err{"[Path][expand] id out of range."};
    CLOG(ERROR, "pose_graph") << err;
    throw std::range_error(err);
  }
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

template <class GraphT>
EdgeTransform Path<GraphT>::pose(unsigned seq_id) const {
  LockGuard lock(mutex_);
  if (seq_id >= sequence_.size()) {
    std::string err{"[Path][pose] id out of range."};
    CLOG(ERROR, "pose_graph") << err;
    throw std::range_error(err);
  }
  // cheating so we can JIT expand
  const_cast<Path<GraphT>*>(this)->expand(seq_id);
  return poses_[seq_id];
}

template <class GraphT>
double Path<GraphT>::dist(unsigned seq_id) const {
  LockGuard lock(mutex_);
  if (seq_id >= sequence_.size()) {
    std::string err{"[Path][dist] id out of range."};
    CLOG(ERROR, "pose_graph") << err;
    throw std::range_error(err);
  }
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

template <class GraphT>
auto Path<GraphT>::sequence() const -> Sequence {
  LockGuard lock(mutex_);
  return sequence_;
}

template <class GraphT>
size_t Path<GraphT>::size() const {
  LockGuard lock(mutex_);
  return sequence_.size();
}

template <class GraphT>
double Path<GraphT>::length() const {
  LockGuard lock(mutex_);
  return dist(sequence_.size() - 1);
}

template <class GraphT>
void Path<GraphT>::initSequence() {
  poses_.clear();
  poses_.reserve(sequence_.size());
  distances_.clear();
  distances_.reserve(sequence_.size());
}

/** \brief An iterator to a specified id along the path */
template <class GraphT>
auto Path<GraphT>::begin(const unsigned& seq_id) const -> Iterator {
  if (seq_id >= sequence_.size()) return end();
  return Iterator(this, seq_id);
}

/** \brief An iterator to the end of the path (beyond the last vertex) */
template <class GraphT>
auto Path<GraphT>::end() const -> Iterator {
  return Iterator(this, sequence_.end());
}

using BasicPathBase = Path<BasicGraphBase>;

}  // namespace pose_graph
}  // namespace vtr
