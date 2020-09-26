#pragma once

#include <vtr_pose_graph/index/composite_graph.hpp>

namespace vtr {
namespace pose_graph {

template <class G>
CompositeGraph<G>::CompositeGraph(const GraphPtr& graph)
    : Base(graph->id_), denseGraph_(graph) {
  typename G::ComponentList paths, cycles;
  auto junctions = graph->pathDecomposition(&paths, &cycles);

  // Split cycles into two paths, because that is easier to manage
  for (auto&& it : cycles) {
    for (auto&& jt : it.split()) {
      (void)junctions.insert(jt.from());
      (void)junctions.insert(jt.to());
      paths.push_back(jt);
    }
  }

  // Iterate through the paths and count duplicate edges between vertex pairs.
  std::map<typename VertexIdType::Pair, int> edgeCount;
  std::set<typename G::RunIdType> runIds;
  for (auto&& it : paths) {
    typename VertexIdType::Pair idx(
        {std::min(it.from(), it.to()), std::max(it.to(), it.from())});
    edgeCount[idx] += 1;

    runIds.insert(it.from().majorId());
    runIds.insert(it.to().majorId());
  }

  typename G::ComponentList cEdges;

  // A simple graph cannot have duplicate edges, so we split them and add extra
  // "junctions" to the vertex set
  for (auto&& it : paths) {
    typename VertexIdType::Pair idx(
        {std::min(it.from(), it.to()), std::max(it.to(), it.from())});
    if (edgeCount[idx] > 1) {
      for (auto&& jt : it.split()) {
        (void)junctions.insert(jt.from());
        (void)junctions.insert(jt.to());
        cEdges.push_back(jt);
      }
    } else {
      cEdges.push_back(it);
    }
  }

  // Instantiate all of the runs that we need
  for (auto&& it : runIds)
    runs_->insert({it, RunType::MakeShared(it, this->id_)});

  // Add vertices before edges; addEdge_ attempts to look up vertices and fill
  // out their connectivity
  for (auto&& it : junctions) addVertex_(it);

  // Add edges
  for (auto&& it : cEdges) addEdge_(it.elements());
}

template <class G>
void CompositeGraph<G>::addEdge_(const std::list<VertexIdType>& seq) {
  auto mfrom = seq.front().majorId(), mto = seq.back().majorId();
  auto runId = std::max(mfrom, mto);
  // \todo (yuchen) edges can only be connected from higher runs to lower runs,
  // why isn't this a bug in vtr2?.
  // typename VertexIdType::Vector tmpvec(seq.begin(), seq.end());
  typename VertexIdType::Vector tmpvec = ([&]() {
    return (seq.front() > seq.back())
               ? typename VertexIdType::Vector{seq.begin(), seq.end()}
               : typename VertexIdType::Vector{seq.rbegin(), seq.rend()};
  })();

  typename CompositeEdge<G>::Ptr tmp(new CompositeEdge<G>(denseGraph_, tmpvec));
  runs_->at(runId)->addEdge(tmp);
  runs_->at(mfrom)->at(seq.front())->addEdge(tmp->id());
  runs_->at(mto)->at(seq.back())->addEdge(tmp->id());

  graph_.addEdge(tmp->simpleId());
  edges_->insert({tmp->simpleId(), tmp});
  tmp->setManual(true);
}

template <class G>
void CompositeGraph<G>::addVertex_(const VertexIdType& v) {
  auto vertex = runs_->at(v.majorId())->addVertex(v);
  vertices_->insert({vertex->simpleId(), vertex});
  graph_.addVertex(vertex->simpleId());
}

template <class G>
auto CompositeGraph<G>::flatten() const -> SubgraphPtr {
  typename VertexIdType::Vector tmp;
  for (auto it = graph_.beginEdge(); it != graph_.endEdge(); ++it) {
    tmp.insert(tmp.end(), edges_->at(*it)->sequence().begin(),
               edges_->at(*it)->sequence().end());
  }
  return denseGraph_->getSubgraph(tmp);
}

}  // namespace pose_graph
}  // namespace vtr
