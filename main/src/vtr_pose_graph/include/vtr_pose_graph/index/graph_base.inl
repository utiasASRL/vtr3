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
 * \file graph_base.inl
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/index/graph_base.hpp>

namespace vtr {
namespace pose_graph {

template <class V, class E, class R>
GraphBase<V, E, R>::GraphBase()
    : id_(-1),
      runs_(new RunMap()),
      vertices_(new VertexMap()),
      edges_(new EdgeMap()){};

template <class V, class E, class R>
GraphBase<V, E, R>::GraphBase(const IdType& id)
    : id_(id),
      runs_(new RunMap()),
      vertices_(new VertexMap()),
      edges_(new EdgeMap()){};

template <class V, class E, class R>
GraphBase<V, E, R>::GraphBase(const GraphBase& other, const SimpleGraph& graph)
    : id_(other.id_),
      graph_(graph),
      runs_(other.runs_),
      vertices_(other.vertices_),
      edges_(other.edges_){};

template <class V, class E, class R>
GraphBase<V, E, R>::GraphBase(const GraphBase& other, SimpleGraph&& graph)
    : id_(other.id_),
      graph_(graph),
      runs_(other.runs_),
      vertices_(other.vertices_),
      edges_(other.edges_){};

template <class V, class E, class R>
typename GraphBase<V, E, R>::VertexPtrSet GraphBase<V, E, R>::neighbors(
    const VertexPtr& v) const {
  VertexPtrSet rval;
  auto neighbours = v->neighbours();

  for (auto it = neighbours.begin(), ite = neighbours.end(); it != ite; ++it) {
    if (graph_.hasVertex(*it)) {
      rval.insert(vertices_->at(*it));
    }
  }

  return rval;
}

template <class V, class E, class R>
typename GraphBase<V, E, R>::EdgePtrSet GraphBase<V, E, R>::incident(
    const VertexPtr& v) const {
  EdgePtrSet rval;
  auto neighbours = v->neighbours();

  for (auto it = neighbours.begin(), ite = neighbours.end(); it != ite; ++it) {
    if (graph_.hasVertex(*it)) {
      rval.insert(edges_->at(simple::SimpleGraph::getEdge(v->id(), *it)));
    }
  }

  return rval;
}

#if 0
template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::getRunSubgraph(
    const RunIdType& runId) const {
  SimpleGraph newGraph;

  for (auto&& it : runs_->at(runId)->edges(Temporal)) {
    if (graph_.hasEdge(it.first)) {
      newGraph.addEdge(it.first);
    }
  }

  return MakeShared(*this, newGraph);
}

template <class V, class E, class R>
auto GraphBase<V, E, R>::autonomousRuns() const -> std::map<RunIdType, Ptr> {
  std::map<RunIdType, Ptr> rmap;

  for (auto&& it : *runs_) {
    if (!it.second->isManual()) {
      rmap.emplace(it.first, getRunSubgraph(it.first));
    }
  }

  return rmap;
}
#endif

template <class V, class E, class R>
auto GraphBase<V, E, R>::pathDecomposition(ComponentList* paths,
                                           ComponentList* cycles) const ->
    typename VertexIdType::UnorderedSet {
  SimpleGraph::ComponentList simplePaths, simpleCycles;
  std::unordered_set<SimpleVertexId> simpleJunctions =
      graph_.pathDecomposition(&simplePaths, &simpleCycles);

  for (auto&& it : simplePaths) paths->push_back(GraphComponent(it));
  for (auto&& it : simpleCycles) cycles->push_back(GraphComponent(it));

  typename VertexIdType::UnorderedSet junctions;
  for (auto&& it : simpleJunctions) junctions.insert(VertexId(it));

  return junctions;
}

}  // namespace pose_graph
}  // namespace vtr
