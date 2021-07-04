#if 0
#define RCGRAPH_BASE_NO_EXTERN
#endif

#include <vtr_pose_graph/evaluator/common.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph_base.hpp>

namespace vtr {
namespace pose_graph {

#if 0
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
#endif

auto RCGraphBase::toPersistent(const VertexIdType& vid) const
    -> GraphPersistentIdMsg {
  return at(vid)->persistentId();
}

auto RCGraphBase::fromPersistent(const GraphPersistentIdMsg& pid) const
    -> VertexIdType {
  try {
    return persistent_map_->locked().get().at(pid);
  } catch (...) {
    LOG(ERROR) << "Could not find persistent id: stamp: " << pid.stamp << ".\n"
               << el::base::debug::StackTrace();
    throw;
  }
  return VertexIdType::Invalid();  // Should not get here
}

RCGraphBase::Ptr RCGraphBase::getManualSubgraph() {
  using PrivEvalType = typename eval::Mask::Privileged<RCGraphBase>::Caching;
  typename PrivEvalType::Ptr manualMask(new PrivEvalType());
  manualMask->setGraph(this);
  return getSubgraph(manualMask);
}

#if 0
template class RunBase<RCVertex, RCEdge>;
template class GraphBase<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>>;

using __GraphBaseRC = GraphBase<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>>;
EVAL_TYPED_EXPLICIT_INSTANTIATE(double, __GraphBaseRC)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, __GraphBaseRC)

EVAL_TYPED_EXPLICIT_INSTANTIATE(double, RCGraphBase)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, RCGraphBase)
#endif
}  // namespace pose_graph
}  // namespace vtr
