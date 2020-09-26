#pragma once

#include <vtr_pose_graph/index/graph_iterator.hpp>

namespace vtr {
namespace pose_graph {

template <class G>
VertexIterator<G>::VertexIterator(const G* graph, const IterType& internalIter)
    : graph_(graph), internalIter_(internalIter) {}

template <class G>
auto VertexIterator<G>::operator*() const -> VertexPtr& {
  return graph_->at(internalIter_->first);
}

template <class G>
auto VertexIterator<G>::operator->() const -> VertexType* {
  return graph_->at(internalIter_->first).get();
}

template <class G>
VertexIterator<G>& VertexIterator<G>::operator++() {
  ++internalIter_;
  return *this;
}

template <class G>
VertexIterator<G> VertexIterator<G>::operator++(int) {
  VertexIterator tmp(graph_, internalIter_);
  ++internalIter_;
  return tmp;
}

// template<class G>
// VertexIterator<G>& VertexIterator<G>::operator--() {
//  --internalIter_;
//  return *this;
//}
//
// template<class G>
// VertexIterator<G> VertexIterator<G>::operator--(int) {
//  VertexIterator tmp(graph_, internalIter_);
//  --internalIter_;
//  return tmp;
//}

template <class G>
bool VertexIterator<G>::operator==(const VertexIterator& other) const {
  return (this->graph_ == other.graph_) &&
         (this->internalIter_ == other.internalIter_);
}

template <class G>
bool VertexIterator<G>::operator!=(const VertexIterator& other) const {
  return (this->graph_ != other.graph_) ||
         (this->internalIter_ != other.internalIter_);
}

template <class G>
EdgeIterator<G>::EdgeIterator(const G* graph, const IterType& internalIter)
    : graph_(graph), internalIter_(internalIter) {}

template <class G>
auto EdgeIterator<G>::operator*() const -> EdgePtr& {
  return graph_->at(*internalIter_);
}

template <class G>
auto EdgeIterator<G>::operator->() const -> EdgeType* {
  return graph_->at(*internalIter_).get();
}

template <class G>
EdgeIterator<G>& EdgeIterator<G>::operator++() {
  ++internalIter_;
  return *this;
}

template <class G>
EdgeIterator<G> EdgeIterator<G>::operator++(int) {
  EdgeIterator tmp(graph_, internalIter_);
  ++internalIter_;
  return tmp;
}

template <class G>
EdgeIterator<G>& EdgeIterator<G>::operator--() {
  --internalIter_;
  return *this;
}

template <class G>
EdgeIterator<G> EdgeIterator<G>::operator--(int) {
  EdgeIterator tmp(graph_, internalIter_);
  --internalIter_;
  return tmp;
}

template <class G>
bool EdgeIterator<G>::operator==(const EdgeIterator& other) const {
  return (this->graph_ == other.graph_) &&
         (this->internalIter_ == other.internalIter_);
}

template <class G>
bool EdgeIterator<G>::operator!=(const EdgeIterator& other) const {
  return (this->graph_ != other.graph_) ||
         (this->internalIter_ != other.internalIter_);
}

}  // namespace pose_graph
}  // namespace vtr
