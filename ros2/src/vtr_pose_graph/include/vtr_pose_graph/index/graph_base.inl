#pragma once

#include <vtr_pose_graph/index/graph_base.hpp>
//#include <asrl/pose_graph/evaluator/Common.hpp>
//#include <asrl/pose_graph/index/Graph.hpp>

namespace vtr {
namespace pose_graph {

template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::MakeShared() {
  return Ptr(new GraphBase());
}

template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::MakeShared(
    const IdType& id) {
  return Ptr(new GraphBase(id));
}

template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::MakeShared(
    const GraphBase& other, const SimpleGraph& graph) {
  return Ptr(new GraphBase(other, graph));
}

template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::MakeShared(
    const GraphBase& other, SimpleGraph&& graph) {
  return Ptr(new GraphBase(other, graph));
}

template <class V, class E, class R>
GraphBase<V, E, R>::GraphBase()
    : id_(-1),
      runs_(new RunMap()),
      vertices_(new VertexMap()),
      edges_(new EdgeMap()) {}

template <class V, class E, class R>
GraphBase<V, E, R>::GraphBase(const IdType& id)
    : id_(id),
      runs_(new RunMap()),
      vertices_(new VertexMap()),
      edges_(new EdgeMap()) {}

template <class V, class E, class R>
GraphBase<V, E, R>::GraphBase(const GraphBase& other, const SimpleGraph& graph)
    : id_(other.id_),
      graph_(graph),
      runs_(other.runs_),
      vertices_(other.vertices_),
      edges_(other.edges_) {}

template <class V, class E, class R>
GraphBase<V, E, R>::GraphBase(const GraphBase& other, SimpleGraph&& graph)
    : id_(other.id_),
      graph_(graph),
      runs_(other.runs_),
      vertices_(other.vertices_),
      edges_(other.edges_) {}

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

template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::getSubgraph(
    const typename VertexIdType::Vector& nodes) const {
  return MakeShared(*this, graph_.getSubgraph(makeSimple(nodes)));
}

template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::getSubgraph(
    const VertexIdType& rootId, const eval::Mask::Ptr& mask) const {
  return MakeShared(*this, graph_.getSubgraph(rootId, mask));
}

template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::getSubgraph(
    const VertexIdType& rootId, double maxDepth,
    const eval::Mask::Ptr& mask) const {
  return MakeShared(*this, graph_.getSubgraph(rootId, maxDepth, mask));
}

template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::getSubgraph(
    const eval::Mask::Ptr& mask) const {
  for (auto it = this->beginVertex(); it != this->endVertex(); ++it) {
    if (mask->operator[](it->id())) return this->getSubgraph(it->id(), mask);
  }
  return MakeShared(*this, SimpleGraph());
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
#endif

// template<class V, class E, class R>
// typename GraphBase<V,E,R>::Ptr GraphBase<V,E,R>::getManualSubgraph() const {
//  std::vector<SimpleVertexId> tmp;
//
//  // Grab all manual runs
//  for (auto &&it: *runs_) {
//    if (it.second->isManual()) {
//      tmp.reserve(tmp.size() + it.second->vertices().size());
//
//      // Filter by the simple graph, as this might be a subgraph already
//      for (auto &&jt: it.second->vertices()) {
//        if (graph_.hasVertex(jt.first)) {
//          tmp.push_back(jt.first);
//        }
//      }
//    }
//  }
//
//  if (tmp.size() == 0) {
//    return MakeShared(*this, SimpleGraph());
//  }
//
//  // Use a privileged mask on the reduced graph just in case
//  typedef typename eval::Mask::Privileged<SelfType>::Caching PrivEvalType;
//  typename PrivEvalType::Ptr manualMask(new PrivEvalType());
//  manualMask->setGraph((void*)this);
//
//  return MakeShared(*this, graph_.getSubgraph(tmp, manualMask));
//}
#if 0
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
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::dijkstraTraverseToDepth(
    const VertexIdType& rootId, double maxDepth,
    const eval::Weight::Ptr& weights, const eval::Mask::Ptr& mask) const {
  return MakeShared(
      *this, graph_.dijkstraTraverseToDepth(rootId, maxDepth, weights, mask));
}

template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::dijkstraSearch(
    const VertexIdType& rootId, VertexIdType searchId,
    const eval::Weight::Ptr& weights, const eval::Mask::Ptr& mask) const {
  return MakeShared(*this,
                    graph_.dijkstraSearch(rootId, searchId, weights, mask));
}

template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::dijkstraMultiSearch(
    const VertexIdType& rootId, const typename VertexIdType::Vector& searchIds,
    const eval::Weight::Ptr& weights, const eval::Mask::Ptr& mask) const {
  return MakeShared(*this, graph_.dijkstraMultiSearch(
                               rootId, makeSimple(searchIds), weights, mask));
}

template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::breadthFirstTraversal(
    const VertexIdType& rootId, double maxDepth) const {
  return MakeShared(*this, graph_.breadthFirstTraversal(rootId, maxDepth));
}

template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::breadthFirstSearch(
    const VertexIdType& rootId, VertexIdType searchId) const {
  return MakeShared(*this, graph_.breadthFirstSearch(rootId, searchId));
}

template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::breadthFirstMultiSearch(
    const VertexIdType& rootId,
    const typename VertexIdType::Vector& searchIds) const {
  return MakeShared(
      *this, graph_.breadthFirstMultiSearch(rootId, makeSimple(searchIds)));
}
#if 0
template <class V, class E, class R>
typename GraphBase<V, E, R>::Ptr GraphBase<V, E, R>::getMinimalSpanningTree(
    const eval::Weight::Ptr& weights, const eval::Mask::Ptr& mask) const {
  return MakeShared(*this, graph_.getMinimalSpanningTree(weights, mask));
}

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
#endif
}  // namespace pose_graph
}  // namespace vtr
