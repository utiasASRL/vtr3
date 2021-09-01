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
 * \file simple_iterator.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_logging/logging.hpp>
#include <vtr_pose_graph/simple_graph/simple_iterator.hpp>

namespace vtr {
namespace pose_graph {
namespace simple {

SimpleGraphIterator SimpleGraphIterator::Dijkstra(
    const SimpleGraph *graph, SimpleVertex root, double maxDepth,
    const eval::Mask::Ptr &mask, const eval::Weight::Ptr &weight) {
  SimpleGraphIterator rval(graph, QueuePtr(new DijkstraQueue()), maxDepth, mask,
                           weight);
  rval.initRootInternal_(root);
  return rval;
}

SimpleGraphIterator SimpleGraphIterator::DFS(const SimpleGraph *graph,
                                             SimpleVertex root, double maxDepth,
                                             const eval::Mask::Ptr &mask,
                                             const eval::Weight::Ptr &weight) {
  SimpleGraphIterator rval(graph, QueuePtr(new DfsQueue()), maxDepth, mask,
                           weight);
  rval.initRootInternal_(root);
  rval.checkCosts_ = false;
  return rval;
}

SimpleGraphIterator SimpleGraphIterator::BFS(const SimpleGraph *graph,
                                             SimpleVertex root, double maxDepth,
                                             const eval::Mask::Ptr &mask,
                                             const eval::Weight::Ptr &weight) {
  SimpleGraphIterator rval(graph, QueuePtr(new BfsQueue()), maxDepth, mask,
                           weight);
  rval.initRootInternal_(root);
  return rval;
}

SimpleGraphIterator SimpleGraphIterator::End(const SimpleGraph *graph) {
  return SimpleGraphIterator(graph, QueuePtr(new BfsQueue()));
}

SimpleGraphIterator::SimpleGraphIterator(const SimpleGraph *graph,
                                         QueuePtr searchQueue, double maxDepth,
                                         const eval::Mask::Ptr &mask,
                                         const eval::Weight::Ptr &weight)
    : graph_(graph),
      searchQueue_(searchQueue),
      maxDepth_(maxDepth),
      mask_(mask),
      weight_(weight),
      checkCosts_(true) {}

SimpleGraphIterator::SimpleGraphIterator(const SimpleGraphIterator &other)
    : graph_(other.graph_),
      searchQueue_(other.searchQueue_->clone()),
      maxDepth_(other.maxDepth_),
      mask_(other.mask_),
      weight_(other.weight_),
      nodeDepths_(other.nodeDepths_),
      checkCosts_(other.checkCosts_) {}

SimpleGraphIterator &SimpleGraphIterator::operator=(
    const SimpleGraphIterator &other) {
  graph_ = other.graph_;
  searchQueue_ = other.searchQueue_->clone();
  maxDepth_ = other.maxDepth_;
  mask_ = other.mask_;
  weight_ = other.weight_;
  nodeDepths_ = other.nodeDepths_;
  checkCosts_ = other.checkCosts_;

  return *this;
}

void SimpleGraphIterator::initRootInternal_(const SimpleVertex &root) {
  searchQueue_->push({0.0, {root, SimpleVertex(-1)}});
  nodeDepths_[root] = 0.0;
}

bool SimpleGraphIterator::operator==(const SimpleGraphIterator &other) const {
  return (this->graph_ == other.graph_) &&
         ((this->searchQueue_->empty() && other.searchQueue_->empty()) ||
          (!other.searchQueue_->empty() && (this->searchQueue_->top().second ==
                                            other.searchQueue_->top().second)));
}

bool SimpleGraphIterator::operator!=(const SimpleGraphIterator &other) const {
  return !this->operator==(other);
}

const NodeParent &SimpleGraphIterator::operator*() const {
  return searchQueue_->top().second;
}

const NodeParent *SimpleGraphIterator::operator->() const {
  return &searchQueue_->top().second;
}

SimpleGraphIterator &SimpleGraphIterator::operator++() {
  incrementInternal_();
  return *this;
}

SimpleGraphIterator SimpleGraphIterator::operator++(int) {
  SimpleGraphIterator tmp(*this);
  ++(*this);
  return tmp;
}

const NodeParent *SimpleGraphIterator::topIfExists() const {
  if (searchQueue_->empty())
    return NULL;
  else
    return &searchQueue_->top().second;
}

void SimpleGraphIterator::incrementInternal_() {
  // Extract all the things from the pairs
  // NOTE: We assume that the top of the queue is a unique vertex; this is
  // enforced external
  //       to this function, because
  const DepthNodeParent &currDepthNodeParent = searchQueue_->top();
  double currNodeDepth = currDepthNodeParent.first;
  SimpleVertex currNodeId = currDepthNodeParent.second.child;
  searchQueue_->pop();

  const SimpleGraph::SimpleNode &currNode = graph_->getNode(currNodeId);

  // For each adjacent node
  for (auto adjIter = currNode.getAdjacent().begin();
       adjIter != currNode.getAdjacent().end(); ++adjIter) {
    // Get child id and make a new edge
    SimpleVertex childId = *adjIter;
    SimpleEdge currEdge = SimpleGraph::getEdge(currNode.getId(), childId);

    if (!mask_->operator[](currEdge) || !mask_->operator[](childId)) {
      continue;
    }

    // Calculate depth to visit node
    double newChildDepth = currNodeDepth + weight_->operator[](currEdge);

    // Check if we have already visited this node
    auto childDepthIter = nodeDepths_.find(childId);
    if (childDepthIter != nodeDepths_.end()) {
      // Double check that recorded depth is indeed less than or equal to
      // proposed depth
      if (checkCosts_ && (childDepthIter->second > newChildDepth)) {
        LOG(FATAL) << "[SimpleGraphIterator][incrementInternal_] found a "
                      "shorter path...";
      }
      continue;
    }

    // If visiting node is less than max depth, add to queue and add to graph
    if (maxDepth_ == 0.0 || newChildDepth <= maxDepth_) {
      searchQueue_->push({newChildDepth, NodeParent(childId, currNodeId)});
    }
  }

  while (!searchQueue_->empty() && !checkQueue_()) {
    searchQueue_->pop();
  }
}

bool SimpleGraphIterator::checkQueue_() {
  // Extract all the things from the pairs
  const DepthNodeParent &currDepthNodeParent = searchQueue_->top();
  double currNodeDepth = currDepthNodeParent.first;
  SimpleVertex currNodeId = currDepthNodeParent.second.child;

  auto depthPair =
      nodeDepths_.insert(std::make_pair(currNodeId, currNodeDepth));
  if (!depthPair.second) {
    // Double check that recorded depth is indeed less than or equal to proposed
    // depth
    if (checkCosts_ && (*depthPair.first).second > currNodeDepth) {
      LOG(FATAL)
          << "[SimpleGraphIterator][checkQueue_] found a shorter path...";
    }
    return false;
  }

  return true;
}

}  // namespace simple
}  // namespace pose_graph
}  // namespace vtr