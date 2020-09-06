#if 0
#define RCGRAPH_BASE_NO_EXTERN
#endif

#include <vtr_pose_graph/index/rc_graph/rc_graph_base.hpp>

#if 0
#include <vtr_pose_graph/evaluator/Common.hpp>
#endif

namespace vtr {
namespace pose_graph {

RCGraphBase::Ptr RCGraphBase::MakeShared() { return Ptr(new RCGraphBase()); }
#if 0
RCGraphBase::Ptr RCGraphBase::MakeShared(const IdType& id) {
  return Ptr(new RCGraphBase(id));
}

RCGraphBase::Ptr RCGraphBase::MakeShared(const RCGraphBase& other,
                                         const SimpleGraph& graph) {
  return Ptr(new RCGraphBase(other, graph));
}

RCGraphBase::Ptr RCGraphBase::MakeShared(const RCGraphBase& other,
                                         SimpleGraph&& graph) {
  return Ptr(new RCGraphBase(other, std::move(graph)));
}
#endif
RCGraphBase::RCGraphBase() : GraphBase<RCVertex, RCEdge, RCRun>() {}
RCGraphBase::RCGraphBase(const IdType& id)
    : GraphBase<RCVertex, RCEdge, RCRun>(id) {}
#if 0
RCGraphBase::RCGraphBase(const RCGraphBase& other, const SimpleGraph& graph)
    : GraphBase<RCVertex, RCEdge, RCRun>(other, graph) {}

RCGraphBase::RCGraphBase(const RCGraphBase& other, SimpleGraph&& graph)
    : GraphBase<RCVertex, RCEdge, RCRun>(other, std::move(graph)) {}

RCGraphBase::RCGraphBase(RCGraphBase&& other) : Base(std::move(other)) {}
#endif
RCGraphBase& RCGraphBase::operator=(RCGraphBase&& other) {
  Base::operator=(std::move(other));
  return *this;
}
#if 0
graph_msgs::PersistentId RCGraphBase::toPersistent(
    const VertexIdType& vid) const {
  return at(vid)->persistentId();
}

auto RCGraphBase::fromPersistent(const graph_msgs::PersistentId& pid) const
    -> VertexIdType {
  try {
    return persistent_map_.locked().get().at(pid);
  } catch (...) {
    LOG(ERROR) << "Could not find " << pid.DebugString() << ".\n"
               << el::base::debug::StackTrace();
    throw;
  }
  return VertexIdType::Invalid();  // Should not get here
}

void RCGraphBase::loadVertexStream(const std::string& streamName,
                                   uint64_t start, uint64_t end) {
  for (auto&& it : *runs_) {
    if (it.second->hasVertexStream(streamName)) {
      it.second->loadVertexStream(streamName, start, end);
    }
  }
}

void RCGraphBase::unloadVertexStream(const std::string& streamName,
                                     uint64_t start, uint64_t end) {
  for (auto&& it : *runs_) {
    if (it.second->hasVertexStream(streamName)) {
      it.second->unloadVertexStream(streamName, start, end);
    }
  }
}

void RCGraphBase::loadVertexPoint(const std::string& pointName, uint64_t idx1,
                                  uint64_t idx2) {
  for (auto it = graph_.beginVertex(); it != graph_.endVertex(); ++it) {
    vertices_->at(it->first)->loadPoint(pointName, idx1, idx2);
  }
}

void RCGraphBase::unloadVertexPoint(const std::string& pointName, uint64_t idx1,
                                    uint64_t idx2) {
  for (auto it = graph_.beginVertex(); it != graph_.endVertex(); ++it) {
    vertices_->at(it->first)->unloadPoint(pointName, idx1, idx2);
  }
}

void RCGraphBase::unloadVertexData() {
  for (auto it = graph_.beginVertex(); it != graph_.endVertex(); ++it) {
    vertices_->at(it->first)->unload();
    vertices_->at(it->first)->unloadPointData();
  }
}

void RCGraphBase::loadEdgePoint(const std::string& pointName, uint64_t idx1,
                                uint64_t idx2) {
  for (auto it = graph_.beginEdge(); it != graph_.endEdge(); ++it) {
    edges_->at(*it)->loadPoint(pointName, idx1, idx2);
  }
}

void RCGraphBase::unloadEdgePoint(const std::string& pointName, uint64_t idx1,
                                  uint64_t idx2) {
  for (auto it = graph_.beginEdge(); it != graph_.endEdge(); ++it) {
    edges_->at(*it)->unloadPoint(pointName, idx1, idx2);
  }
}

void RCGraphBase::unloadEdgeData() {
  for (auto it = graph_.beginEdge(); it != graph_.endEdge(); ++it) {
    edges_->at(*it)->unloadPointData();
  }
}

void RCGraphBase::unloadData() {
  unloadEdgeData();
  unloadVertexData();
}

/// \brief Get subgraph including all the specified nodes (and all

RCGraphBase::Ptr RCGraphBase::getSubgraph(
    const typename VertexIdType::Vector& nodes) const {
  return MakeShared(*this, graph_.getSubgraph(makeSimple(nodes)));
}
/// \brief Get subgraph including all the specified nodes (and all

RCGraphBase::Ptr RCGraphBase::getSubgraph(const VertexIdType& rootId,
                                          const Eval::Mask::Ptr& mask) const {
  return MakeShared(*this, graph_.getSubgraph(rootId, mask));
}
/// \brief Get subgraph including all the specified nodes (and all

RCGraphBase::Ptr RCGraphBase::getSubgraph(const VertexIdType& rootId,
                                          double maxDepth,
                                          const Eval::Mask::Ptr& mask) const {
  return MakeShared(*this, graph_.getSubgraph(rootId, maxDepth, mask));
}
/// \brief Get subgraph including all the specified nodes (and all

RCGraphBase::Ptr RCGraphBase::getSubgraph(const Eval::Mask::Ptr& mask) const {
  for (auto it = this->beginVertex(); it != this->endVertex(); ++it) {
    if (mask->operator[](it->id())) {
      return this->getSubgraph(it->id(), mask);
    }
  }

  return MakeShared(*this, SimpleGraph());
}

RCGraphBase::Ptr RCGraphBase::getManualSubgraph() {
  typedef typename Eval::Mask::Privileged<RCGraphBase>::Caching PrivEvalType;
  typename PrivEvalType::Ptr manualMask(new PrivEvalType());
  manualMask->setGraph(this);
  return this->getSubgraph(manualMask);
}

RCGraphBase::Ptr RCGraphBase::dijkstraTraverseToDepth(
    const VertexIdType& rootId, double maxDepth,
    const Eval::Weight::Ptr& weights, const Eval::Mask::Ptr& mask) const {
  return MakeShared(
      *this, graph_.dijkstraTraverseToDepth(rootId, maxDepth, weights, mask));
}

RCGraphBase::Ptr RCGraphBase::dijkstraSearch(
    const VertexIdType& rootId, VertexIdType searchId,
    const Eval::Weight::Ptr& weights, const Eval::Mask::Ptr& mask) const {
  return MakeShared(*this,
                    graph_.dijkstraSearch(rootId, searchId, weights, mask));
}

RCGraphBase::Ptr RCGraphBase::dijkstraMultiSearch(
    const VertexIdType& rootId, const typename VertexIdType::Vector& searchIds,
    const Eval::Weight::Ptr& weights, const Eval::Mask::Ptr& mask) const {
  return MakeShared(*this, graph_.dijkstraMultiSearch(
                               rootId, makeSimple(searchIds), weights, mask));
}

RCGraphBase::Ptr RCGraphBase::breadthFirstTraversal(const VertexIdType& rootId,
                                                    double maxDepth) const {
  return MakeShared(*this, graph_.breadthFirstTraversal(rootId, maxDepth));
}

RCGraphBase::Ptr RCGraphBase::breadthFirstSearch(const VertexIdType& rootId,
                                                 VertexIdType searchId) const {
  return MakeShared(*this, graph_.breadthFirstSearch(rootId, searchId));
}

RCGraphBase::Ptr RCGraphBase::breadthFirstMultiSearch(
    const VertexIdType& rootId,
    const typename VertexIdType::Vector& searchIds) const {
  return MakeShared(
      *this, graph_.breadthFirstMultiSearch(rootId, makeSimple(searchIds)));
}

RCGraphBase::Ptr RCGraphBase::getMinimalSpanningTree(
    const Eval::Weight::Ptr& weights, const Eval::Mask::Ptr& mask) const {
  return MakeShared(*this, graph_.getMinimalSpanningTree(weights, mask));
}

template class RunBase<RCVertex, RCEdge>;
template class GraphBase<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>>;

typedef GraphBase<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>> __GraphBaseRC;
EVAL_TYPED_EXPLICIT_INSTANTIATE(double, __GraphBaseRC)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, __GraphBaseRC)

EVAL_TYPED_EXPLICIT_INSTANTIATE(double, RCGraphBase)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, RCGraphBase)
#endif
}  // namespace pose_graph
}  // namespace vtr
