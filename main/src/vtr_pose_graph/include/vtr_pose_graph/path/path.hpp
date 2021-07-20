#pragma once

#include <vtr_pose_graph/evaluator/accumulators.hpp>
#include <vtr_pose_graph/index/graph.hpp>
#include <vtr_pose_graph/path/path_iterator.hpp>

namespace vtr {
namespace pose_graph {

template <class G, class TF = typename G::EdgeType::TransformType>
class Path {
 public:
  // Graph typedefs
  using GraphType = G;
  using GraphPtr = typename G::Ptr;
  using VertexIdType = typename G::VertexIdType;
  using SequenceType = typename VertexIdType::Vector;
  using tf_t = TF;

  // an iterator to this path
  using Iterator = PathIterator<Path>;

  // shared pointer type definitions for this class
  PTR_TYPEDEFS(Path)

  Path(const GraphPtr& graph) : graph_(graph){};
  Path(const Path&) = default;
  Path(Path&&) = default;

  Path& operator=(const Path&) = default;
  Path& operator=(Path&&) = default;

  // Sequence getters
  SequenceType& sequence() { return sequence_; }
  const SequenceType& sequence() const { return sequence_; }

  // Sequence setters
  void setSequence(SequenceType&& s) {
    sequence_ = s;
    initSequence();
  }
  void setSequence(const SequenceType& s) {
    sequence_ = s;
    initSequence();
  }
  void appendSequence(const SequenceType& s) {
    sequence_.insert(sequence_.end(), s.begin(), s.end());
  }

  virtual inline Path& reverse() {
    std::reverse(sequence_.begin(), sequence_.end());
    initSequence();
    return *this;
  }

  inline Path reversed() const {
    Path rval(*this);
    rval.reverse();
    return rval;
  }

  /// Verification (not verified on set for performance)
  bool verifySequence();

  /// An iterator to the start of the path (or a specified id along the path)
  Iterator begin(unsigned seq_id = 0) const {
    if (seq_id >= sequence_.size()) {
      return end();
    }
    return Iterator(this, seq_id);
  }

  /// An iterator to the end of the path (beyond the last vertex)
  Iterator end() const { return Iterator(this, sequence_.end()); }

  bool append(const VertexId& v);

  /// Get a graph reference
  const G& graph() const { return *graph_; }

  /// Display vertices in the path
  friend std::ostream& operator<<(std::ostream& os, const Path& me) {
    for (auto s = me.begin(); s != me.end(); ++s) os << s << " ";
    return os;
  }

  /// Put the entire sequence in a single privileged integrated frame
  void expand() { expand(sequence_.size() - 1); }
  /// Put the sequence up to this id into a single privileged frame
  void expand(unsigned seq_id);

  /// Get the pose at a sequence index
  const tf_t& pose(unsigned seq_id) const;
  /// Get the pose at an iterator position
  inline const tf_t& pose(const Iterator& it) const {
    return pose(unsigned(it));
  }
  // This should not be called with a vertex id, which implicitly casts to
  // unsigned :(
  const tf_t& pose(VertexIdType vtx_id) const = delete;

  const VertexIdType& endVertexID() const { return sequence_.back(); }

  /// Total length of the path
  inline const double& length() const { return dist(sequence_.size() - 1); }
  /// Total number of poses in the sequence
  inline const double& size() const { return sequence_.size(); }

  /// Get the cumulative distance along the path at a sequence index
  const double& dist(unsigned seq_id) const;
  /// Get the cumulative distance along the path at an iterator position
  inline const double& dist(const Iterator& it) const {
    return dist(unsigned(it));
  }
  // This should not be called with a vertex id, which implicitly casts to
  // unsigned :(
  const double& dist(VertexIdType vtx_id) const = delete;

 protected:
  virtual void initSequence() {
    poses_.clear();
    poses_.reserve(sequence_.size());

    distances_.clear();
    distances_.reserve(sequence_.size());
    /// \todo move this to 'dist' function
    if (sequence_.size() > 0) distances_.push_back(0.);
  }

  GraphPtr graph_;
  SequenceType sequence_;
  mutable std::vector<tf_t> poses_;
  mutable std::vector<double> distances_;
};

template <class G, class TF>
bool Path<G, TF>::append(const VertexId& v) {
  if (!graph_->contains(v) || !graph_->contains(sequence_.back(), v)) {
    return false;
  }

  sequence_.push_back(v);
  return true;
}

template <class G, class TF>
bool Path<G, TF>::verifySequence() {
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

template <class G, class TF>
void Path<G, TF>::expand(unsigned seq_id) {
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

template <class G, class TF>
auto Path<G, TF>::pose(unsigned seq_id) const -> const tf_t& {
  if (seq_id >= sequence_.size())
    throw std::range_error("[Path][pose] id out of range.");
  const_cast<Path<G>*>(this)->expand(seq_id);  // Cheating so we can JIT expand
  return poses_[seq_id];
}

template <class G, class TF>
auto Path<G, TF>::dist(unsigned seq_id) const -> const double& {
  if (seq_id >= sequence_.size())
    throw std::range_error("[Path][dist] id out of range.");

  if (seq_id >= distances_.size()) {
    for (auto it = begin(distances_.size()); unsigned(it) <= seq_id; ++it) {
      const_cast<Path<G>*>(this)->distances_.push_back(
          distances_.back() + it->T().r_ab_inb().norm());
    }
  }

  return distances_[seq_id];
}

using BasicPathBase = Path<BasicGraphBase>;

}  // namespace pose_graph
}  // namespace vtr
