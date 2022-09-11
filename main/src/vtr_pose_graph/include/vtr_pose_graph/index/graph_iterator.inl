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
 * \file graph_iterator.inl
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/index/graph_iterator.hpp"

namespace vtr {
namespace pose_graph {

template <class G>
VertexIterator<G>::VertexIterator(const G* graph, const IterType& internal_iter)
    : graph_(graph), internal_iter_(internal_iter) {}

template <class G>
auto VertexIterator<G>::operator*() const -> VertexPtr& {
  return graph_->at(internal_iter_->first);
}

template <class G>
auto VertexIterator<G>::operator->() const -> Vertex* {
  return graph_->at(internal_iter_->first).get();
}

template <class G>
VertexIterator<G>& VertexIterator<G>::operator++() {
  ++internal_iter_;
  return *this;
}

template <class G>
VertexIterator<G> VertexIterator<G>::operator++(int) {
  VertexIterator tmp(graph_, internal_iter_);
  ++internal_iter_;
  return tmp;
}

template <class G>
bool VertexIterator<G>::operator==(const VertexIterator& other) const {
  return (this->graph_ == other.graph_) &&
         (this->internal_iter_ == other.internal_iter_);
}

template <class G>
bool VertexIterator<G>::operator!=(const VertexIterator& other) const {
  return (this->graph_ != other.graph_) ||
         (this->internal_iter_ != other.internal_iter_);
}

template <class G>
EdgeIterator<G>::EdgeIterator(const G* graph, const IterType& internal_iter)
    : graph_(graph), internal_iter_(internal_iter) {}

template <class G>
auto EdgeIterator<G>::operator*() const -> EdgePtr& {
  return graph_->at(*internal_iter_);
}

template <class G>
auto EdgeIterator<G>::operator->() const -> Edge* {
  return graph_->at(*internal_iter_).get();
}

template <class G>
EdgeIterator<G>& EdgeIterator<G>::operator++() {
  ++internal_iter_;
  return *this;
}

template <class G>
EdgeIterator<G> EdgeIterator<G>::operator++(int) {
  EdgeIterator tmp(graph_, internal_iter_);
  ++internal_iter_;
  return tmp;
}

template <class G>
EdgeIterator<G>& EdgeIterator<G>::operator--() {
  --internal_iter_;
  return *this;
}

template <class G>
EdgeIterator<G> EdgeIterator<G>::operator--(int) {
  EdgeIterator tmp(graph_, internal_iter_);
  --internal_iter_;
  return tmp;
}

template <class G>
bool EdgeIterator<G>::operator==(const EdgeIterator& other) const {
  return (this->graph_ == other.graph_) &&
         (this->internal_iter_ == other.internal_iter_);
}

template <class G>
bool EdgeIterator<G>::operator!=(const EdgeIterator& other) const {
  return (this->graph_ != other.graph_) ||
         (this->internal_iter_ != other.internal_iter_);
}

template <class G>
OrderedGraphIterator<G>::OrderedGraphIterator(const G* graph,
                                              const IterType& internal_iter)
    : internal_iter_(internal_iter) {
  if (internal_iter_.empty())
    data_ = NodeParent<G>(graph);
  else
    data_ = NodeParent<G>(graph, *internal_iter_);
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
  ++internal_iter_;
  if (!internal_iter_.empty())
    data_ = NodeParent<G>(data_.graph_, *internal_iter_);
  else
    data_ = NodeParent<G>(data_.graph_);

  return *this;
}

template <class G>
OrderedGraphIterator<G> OrderedGraphIterator<G>::operator++(int) {
  OrderedGraphIterator<G> tmp(data_.graph_, internal_iter_);
  ++(*this);
  return tmp;
}

template <class G>
bool OrderedGraphIterator<G>::operator==(
    const OrderedGraphIterator& other) const {
  return (this->data_.graph_ == other.data_.graph_) &&
         (this->internal_iter_ == other.internal_iter_);
}

template <class G>
bool OrderedGraphIterator<G>::operator!=(
    const OrderedGraphIterator& other) const {
  return (this->data_.graph_ != other.data_.graph_) ||
         (this->internal_iter_ != other.internal_iter_);
}

}  // namespace pose_graph
}  // namespace vtr
