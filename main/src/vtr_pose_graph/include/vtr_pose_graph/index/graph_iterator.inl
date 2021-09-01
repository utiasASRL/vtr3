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
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
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
