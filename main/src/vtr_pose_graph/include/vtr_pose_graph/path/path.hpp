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
 * \brief Path class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
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

  // shared pointer type definitions for this class
  PTR_TYPEDEFS(Path)

  Path(const typename GraphT::Ptr& graph) : graph_(graph) {}
  Path(const Path&) = default;
  Path(Path&&) = default;

  Path& operator=(const Path&) = default;
  Path& operator=(Path&&) = default;

 public:
  // Sequence setters
  void setSequence(Sequence&& s) {
    LockGuard lock(mutex_);
    sequence_ = s;
    initSequence();
  }
  void setSequence(const Sequence& s) {
    LockGuard lock(mutex_);
    sequence_ = s;
    initSequence();
  }
  void appendSequence(const Sequence& s) {
    LockGuard lock(mutex_);
    sequence_.insert(sequence_.end(), s.begin(), s.end());
  }

  virtual Path& reverse() {
    LockGuard lock(mutex_);
    std::reverse(sequence_.begin(), sequence_.end());
    initSequence();
    return *this;
  }

  Path reversed() const {
    LockGuard lock(mutex_);
    Path rval(*this);
    rval.reverse();
    return rval;
  }

  /// Verification (not verified on set for performance)
  bool verifySequence();

  /// An iterator to the start of the path (or a specified id along the path)
  /// \note this is not thread safe
  Iterator begin(unsigned seq_id = 0) const {
    if (seq_id >= sequence_.size()) return end();
    return Iterator(this, seq_id);
  }

  /// An iterator to the end of the path (beyond the last vertex)
  Iterator end() const { return Iterator(this, sequence_.end()); }

  bool append(const VertexId& v);

  /// Display vertices in the path
  friend std::ostream& operator<<(std::ostream& os, const Path& me) {
    for (auto s = me.begin(); s != me.end(); ++s) os << s << " ";
    return os;
  }
  template <class P>
  friend std::ostream& operator<<(std::ostream& os, const PathIterator<P>&);

  /// Put the entire sequence in a single privileged integrated frame
  void expand() {
    LockGuard lock(mutex_);
    expand(sequence_.size() - 1);
  }
  /// Put the sequence up to this id into a single privileged frame
  void expand(unsigned seq_id);

  /// Get the pose at a sequence index
  TF pose(unsigned seq_id) const;
  /// Get the pose at an iterator position
  TF pose(const Iterator& it) const { return pose(unsigned(it)); }
  // This should not be called with a vertex id, which implicitly casts to
  // unsigned :(
  TF pose(VertexId vtx_id) const = delete;

  VertexId endVertexID() const {
    LockGuard lock(mutex_);
    return sequence_.back();
  }

  /// Total length of the path
  double length() const {
    LockGuard lock(mutex_);
    return dist(sequence_.size() - 1);
  }
  /// Total number of poses in the sequence
  size_t size() const {
    LockGuard lock(mutex_);
    return sequence_.size();
  }

  /// Get the cumulative distance along the path at a sequence index
  double dist(unsigned seq_id) const;
  /// Get the cumulative distance along the path at an iterator position
  double dist(const Iterator& it) const { return dist(unsigned(it)); }
  // This should not be called with a vertex id, which implicitly casts to
  // unsigned :(
  double dist(VertexId vtx_id) const = delete;

  /** \brief Acquires a lock object that blocks modifications. */
  UniqueLock guard() const { return UniqueLock(mutex_); }
  /** \brief Manually locks the chain. */
  void lock() const { mutex_.lock(); }
  /** \brief Manually unlocks the chain. */
  void unlock() const { mutex_.unlock(); }
  /** \brief Get a reference to the mutex */
  Mutex& mutex() const { return mutex_; }

 protected:
  virtual void initSequence() {
    poses_.clear();
    poses_.reserve(sequence_.size());

    distances_.clear();
    distances_.reserve(sequence_.size());
  }

  typename GraphT::Ptr graph_;
  Sequence sequence_;
  mutable std::vector<TF> poses_;
  mutable std::vector<double> distances_;

  /** \brief for thread safety, use whenever read from/write to the path */
  mutable Mutex mutex_;

  friend class PathIterator<Path>;
};

template <class GraphT, class TF>
bool Path<GraphT, TF>::append(const VertexId& v) {
  LockGuard lock(mutex_);
  if (!graph_->contains(v) || !graph_->contains(sequence_.back(), v))
    return false;

  sequence_.push_back(v);
  return true;
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

using BasicPathBase = Path<BasicGraphBase>;

}  // namespace pose_graph
}  // namespace vtr
