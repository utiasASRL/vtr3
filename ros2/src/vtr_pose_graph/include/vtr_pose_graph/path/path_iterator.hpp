#pragma once

#include <vtr_pose_graph/index/graph_iterator.hpp>

namespace vtr {
namespace pose_graph {

/// Iterator to a path in search order
template <class P>
class PathIterator : std::iterator<std::random_access_iterator_tag,
                                   const NodeParent<typename P::GraphType> > {
 public:
  using GraphType = typename P::GraphType;
  using VertexIdType = typename P::VertexIdType;
  using SequenceType = typename P::SequenceType;
  using InternalIterType = typename SequenceType::const_iterator;
  using ValueType = NodeParent<GraphType>;

  // Invalid default for construction
  constexpr ValueType InvalidValue() {
    return ValueType(&path_->graph(), top_);
  }

  PathIterator(const P* path, const InternalIterType& seq_iter)
      : path_(path), top_(), data_(InvalidValue()), seq_iter_(seq_iter) {
    makeData();
  }
  PathIterator(const P* path, unsigned seq_id = 0)
      : path_(path),
        top_(),
        data_(InvalidValue()),
        seq_iter_(path_->sequence().begin() + seq_id) {
    makeData();
  }

  PathIterator(const PathIterator&) = default;
  PathIterator(PathIterator&&) = default;

  PathIterator& operator=(const PathIterator&) = default;
  PathIterator& operator=(PathIterator&&) = default;

  // Dereference
  const ValueType& operator*() const {
    return data_;
  }
  const ValueType* operator->() const {
    return &data_;
  }

  // Increment
  PathIterator& operator++() {
    seq_iter_++;
    makeData();
    return *this;
  }
  PathIterator operator++(int) {
    auto orig = *this;
    ++seq_iter_;
    makeData();
    return orig;
  }

  // Equality comparison (we're trusting that the graph is the same)
  bool operator==(const PathIterator& o) const {
    return seq_iter_ == o.seq_iter_;
  }
  bool operator!=(const PathIterator& o) const {
    return !(*this == o);
  }

  // Decrement
  PathIterator& operator--() {
    seq_iter_--;
    makeData();
    return *this;
  }
  PathIterator operator--(int) {
    auto orig = *this;
    --seq_iter_;
    makeData();
    return orig;
  }

  // Arithmetic (lazy, and not overloading n + it, just it + n
  template <typename T>
  PathIterator operator+(T n) {
    return PathIterator(*this) += n;
  }
  template <typename T>
  PathIterator operator-(T n) {
    return PathIterator(*this) -= n;
  }

  // Inequality comparison
  bool operator<(const PathIterator& o) {
    return seq_iter_ < o.seq_iter_;
  }
  bool operator<=(const PathIterator& o) {
    return *this == o || *this < o;
  }
  bool operator>(const PathIterator& o) {
    return seq_iter_ > o.seq_iter_;
  }
  bool operator>=(const PathIterator& o) {
    return *this == o || *this > o;
  }

  // Compound assignment
  template <typename T>
  PathIterator& operator+=(T n) {
    seq_iter_ += n;
    makeData();
    return *this;
  }
  template <typename T>
  PathIterator& operator-=(T n) {
    seq_iter_ -= n;
    makeData();
    return *this;
  }

  // Offset dereference
  template <typename T>
  ValueType operator[](T n) {
    return *(*this + n);
  }

  // Cast to index (for convenience)
  explicit operator unsigned() const {
    return seq_iter_ - path_->sequence().begin();
  }

  // Display
  template <typename P_>
  friend std::ostream& operator<<(std::ostream& os, const PathIterator<P_>& me);

 private:
  // Creates the underlying value when the iterator is modified
  ValueType& makeData();

  const P* path_;
  simple::NodeParent top_;
  ValueType data_;
  InternalIterType seq_iter_;
};

template <class P>
auto PathIterator<P>::makeData() -> ValueType& {
  // Check for special cases of start/end, and invalidate ids if necessary
  bool is_end = seq_iter_ == path_->sequence().end();
  bool is_begin = seq_iter_ == path_->sequence().begin();
  VertexIdType child_id = is_end ? VertexIdType::Invalid() : *seq_iter_;
  VertexIdType parent_id =
      is_begin ? VertexIdType::Invalid() : *(seq_iter_ - 1);

  // Create the directed-edge value type
  top_ = simple::NodeParent(child_id, parent_id);
  return data_ = ValueType(&path_->graph(), top_);
}

template <class P>
std::ostream& operator<<(std::ostream& os, const PathIterator<P>& me) {
  // The end of the path is invalid to display
  if (me.seq_iter_ == me.path_->sequence().end())
    throw std::runtime_error("[PathIterator][<<] The end iterator is invalid.");

  // In the middle, put an arrow / broken arrow
  auto& child_id = me.top_.child;
  if (me.seq_iter_ != me.path_->sequence().begin())
    os << (me.path_->graph().contains(child_id, me.top_.parent) ? "-> "
                                                                : "!> ");

  // Our sequence id
  os << "[" << (me.seq_iter_ - me.path_->sequence().begin()) << "]";
  // The vertex
  if (me.path_->graph().contains(child_id))
    os << *me.path_->graph().at(child_id);
  else
    os << "<!" << child_id << ">";

  // Done
  return os;
}

}  // namespace pose_graph
}  // namespace vtr
