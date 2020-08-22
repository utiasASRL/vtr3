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

template <class G>
OrderedGraphIterator<G>::OrderedGraphIterator(
    const G* graph, const simple::SimpleGraphIterator& internalIter)
    : internalIter_(internalIter) {
  if (internalIter_.empty()) {
    data_ = NodeParent<G>(graph);
  } else {
    data_ = NodeParent<G>(graph, *internalIter_);
  }
}

template <class G>
const NodeParent<G>& OrderedGraphIterator<G>::operator*() const {
  return data_;
}

template <class G>
const NodeParent<G>* OrderedGraphIterator<G>::operator->() const {
  return &data_;
}

template <class G>
OrderedGraphIterator<G>& OrderedGraphIterator<G>::operator++() {
  ++internalIter_;
  if (!internalIter_.empty()) {
    data_ = NodeParent<G>(data_.graph_, *internalIter_);
  } else {
    data_ = NodeParent<G>(data_.graph_);
  }
  return *this;
}

template <class G>
OrderedGraphIterator<G> OrderedGraphIterator<G>::operator++(int) {
  OrderedGraphIterator<G> tmp(data_.graph_, internalIter_);
  ++(*this);
  return tmp;
}

template <class G>
bool OrderedGraphIterator<G>::operator==(
    const OrderedGraphIterator& other) const {
  return (this->data_.graph_ == other.data_.graph_) &&
         (this->internalIter_ == other.internalIter_);
}

template <class G>
bool OrderedGraphIterator<G>::operator!=(
    const OrderedGraphIterator& other) const {
  return (this->data_.graph_ != other.data_.graph_) ||
         (this->internalIter_ != other.internalIter_);
}

}  // namespace pose_graph
}  // namespace vtr
