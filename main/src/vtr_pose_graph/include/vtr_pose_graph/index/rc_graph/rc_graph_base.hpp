#pragma once

#include <vtr_common/utils/lockable.hpp>
#include <vtr_pose_graph/index/graph_base.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_edge.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_run.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_vertex.hpp>

namespace vtr {
namespace pose_graph {

class RCGraphBase : public virtual GraphBase<RCVertex, RCEdge, RCRun> {
 public:
  using Base = GraphBase<RCVertex, RCEdge, RCRun>;
  using RType = RCGraphBase;

  using Base::edges_;
  using Base::graph_;
  using Base::id_;
  using Base::runs_;
  using Base::vertices_;

  using GraphPersistentIdMsg = vtr_messages::msg::GraphPersistentId;

  /** \brief Shared pointer type definitions for this class */
  PTR_TYPEDEFS(RCGraphBase)

  /** \brief Inherited copy/move operators (this class has no new data) */
  /**
   * \brief Interface to downcast base class pointers
   * \details This allows us to do DerivedPtrType = Type::Cast(BasePtrType)
   */
  //  PTR_DOWNCAST_OPS(RCGraphBase, GraphBase<RCVertex, RCEdge, RCRun>)

  /** \brief Pseudo-constructor to make shared pointers */
  static Ptr MakeShared() { return Ptr(new RCGraphBase()); }
  static Ptr MakeShared(const IdType& id) { return Ptr(new RCGraphBase(id)); }
  static Ptr MakeShared(const RCGraphBase& other, const SimpleGraph& graph) {
    return Ptr(new RCGraphBase(other, graph));
  }
  static Ptr MakeShared(const RCGraphBase& other, SimpleGraph&& graph) {
    return Ptr(new RCGraphBase(other, std::move(graph)));
  }

  RCGraphBase() : GraphBase<RCVertex, RCEdge, RCRun>() {}
  RCGraphBase(const IdType& id) : GraphBase<RCVertex, RCEdge, RCRun>(id) {}

  RCGraphBase(const RCGraphBase&) = default;
  RCGraphBase(RCGraphBase&& other)
      : Base(std::move(other)),
        persistent_map_(std::move(other.persistent_map_)) {}

  RCGraphBase& operator=(const RCGraphBase&) = default;
  RCGraphBase& operator=(RCGraphBase&& other) {
    Base::operator=(std::move(other));
    this->persistent_map_ = std::move(other.persistent_map_);
    return *this;
  }

  RCGraphBase(const RCGraphBase& other, const SimpleGraph& graph)
      : GraphBase<RCVertex, RCEdge, RCRun>(other, graph),
        persistent_map_(other.persistent_map_) {}
  RCGraphBase(const RCGraphBase& other, SimpleGraph&& graph)
      : GraphBase<RCVertex, RCEdge, RCRun>(other, std::move(graph)),
        persistent_map_(std::move(other.persistent_map_)) {}

#if 0
  /** \brief Load a stream of data for all vertices */
  void loadVertexStream(const std::string& streamName, uint64_t start = 0,
                        uint64_t end = 0);

  /** \brief Unload a stream of data for all vertices */
  void unloadVertexStream(const std::string& streamName, uint64_t start = 0,
                          uint64_t end = 0);

  /** \brief Load a piece of point data for all vertices */
  void loadVertexPoint(const std::string& pointName, uint64_t idx1 = 0,
                       uint64_t idx2 = 0);

  /** \brief Unload a piece of point data for all vertices */
  void unloadVertexPoint(const std::string& pointName, uint64_t idx1 = 0,
                         uint64_t idx2 = 0);

  /** \brief Unload all data for all vertices */
  void unloadVertexData();

  /** \brief Load a piece of point data for all edges */
  void loadEdgePoint(const std::string& pointName, uint64_t idx1 = 0,
                     uint64_t idx2 = 0);

  /** \brief Unload a piece of point data for all edges */
  void unloadEdgePoint(const std::string& pointName, uint64_t idx1 = 0,
                       uint64_t idx2 = 0);

  /** \brief Unload all data for all edges */
  void unloadEdgeData();

  /** \brief Unload all Robochunk data in this graph */
  void unloadData();

  /** \brief Check if any run has a given stream */
  inline bool hasVertexStream(const std::string& stream_name) const {
    for (auto&& it : *runs_) {
      if (it.second->hasVertexStream(stream_name)) {
        return true;
      }
    }
    return false;
  }

  /** \brief Raw stream read access for data that isn't vertex-indexed */
  inline const StreamPtr& readStream(const RunIdType& run_id,
                                     const std::string& streamName) const {
    if (runs_ != nullptr && runs_->find(run_id) != runs_->end()) {
      return runs_->at(run_id)->readStream(streamName);
    } else {
      LOG(ERROR) << "[RCGraphBase::readStream] Run " << run_id
                 << " was not in the run map.";
    }
  }
#endif

  // Get the persistent id from this vertex id (unchanged on graph refactor)
  GraphPersistentIdMsg toPersistent(const VertexIdType& vid) const;

  // Get the vertex id from persistent id (unchanged on graph refactor)
  VertexIdType fromPersistent(const GraphPersistentIdMsg& pid) const;

  /**
   * The graph functions are given thin re-implementations to handle casting
   */

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const typename VertexIdType::Vector& nodes) const {
    return MakeShared(*this, graph_.getSubgraph(makeSimple(nodes)));
  }

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const VertexIdType& rootId,
                  const eval::Mask::Ptr& mask) const {
    return MakeShared(*this, graph_.getSubgraph(rootId, mask));
  }

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const VertexIdType& rootId, double maxDepth,
                  const eval::Mask::Ptr& mask) const {
    return MakeShared(*this, graph_.getSubgraph(rootId, maxDepth, mask));
  }

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const eval::Mask::Ptr& mask) const {
    for (auto it = this->beginVertex(); it != this->endVertex(); ++it) {
      if (mask->operator[](it->id())) return this->getSubgraph(it->id(), mask);
    }
    return MakeShared(*this, SimpleGraph());
  }

  /** \brief Convenience function to get a manual subgraph */
  Ptr getManualSubgraph();

  /** \brief Get the induced subgraph of another subgraph */
  inline Ptr induced(const RCGraphBase& subgraph) const {
    return Ptr(new RCGraphBase(*this, graph_.induced(subgraph.graph_)));
  }

  /** \brief Merge two graphs in place, as a set union */
  RCGraphBase& operator+=(const RCGraphBase& other) {
    graph_ += other.graph_;
    return *this;
  }

  /** \brief Merge two graphs, as a set union */
  friend RCGraphBase operator+(RCGraphBase lhs, const RCGraphBase& rhs) {
    lhs += rhs;
    return lhs;
  }

  /** \brief Merge two graphs, as a set union */
  Ptr setUnion(const Ptr& other) const {
    return Ptr(new RCGraphBase(*this, graph_ + other->graph_));
  }

  /** \brief Use dijkstra's algorithm to traverse up to a depth (weighted edges)
   */
  Ptr dijkstraTraverseToDepth(
      const VertexIdType& rootId, double maxDepth,
      const eval::Weight::Ptr& weights = eval::Weight::Const::MakeShared(1, 1),
      const eval::Mask::Ptr& mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const {
    return MakeShared(
        *this, graph_.dijkstraTraverseToDepth(rootId, maxDepth, weights, mask));
  }

  /** \brief Use dijkstra's algorithm to search for an id (weighted edges) */
  Ptr dijkstraSearch(
      const VertexIdType& rootId, VertexIdType searchId,
      const eval::Weight::Ptr& weights = eval::Weight::Const::MakeShared(1, 1),
      const eval::Mask::Ptr& mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const {
    return MakeShared(*this,
                      graph_.dijkstraSearch(rootId, searchId, weights, mask));
  }

  /**
   * \brief Use dijkstra's algorithm to search for multiple ids (weighted
   * edges)
   */
  Ptr dijkstraMultiSearch(
      const VertexIdType& rootId,
      const typename VertexIdType::Vector& searchIds,
      const eval::Weight::Ptr& weights = eval::Weight::Const::MakeShared(1, 1),
      const eval::Mask::Ptr& mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const {
    return MakeShared(*this, graph_.dijkstraMultiSearch(
                                 rootId, makeSimple(searchIds), weights, mask));
  }

  /** \brief Use breadth first traversal up to a depth */
  Ptr breadthFirstTraversal(const VertexIdType& rootId, double maxDepth) const {
    return MakeShared(*this, graph_.breadthFirstTraversal(rootId, maxDepth));
  }

  /** \brief Use breadth first search for an id */
  Ptr breadthFirstSearch(const VertexIdType& rootId,
                         VertexIdType searchId) const {
    return MakeShared(*this, graph_.breadthFirstSearch(rootId, searchId));
  }

  /** \brief Use breadth first search for multiple ids */
  Ptr breadthFirstMultiSearch(
      const VertexIdType& rootId,
      const typename VertexIdType::Vector& searchIds) const {
    return MakeShared(
        *this, graph_.breadthFirstMultiSearch(rootId, makeSimple(searchIds)));
  }

  /** \brief Get minimal spanning tree */
  Ptr getMinimalSpanningTree(
      const eval::Weight::Ptr& weights,
      const eval::Mask::Ptr& mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const {
    return MakeShared(*this, graph_.getMinimalSpanningTree(weights, mask));
  }

 protected:
  using PersistentMap = std::unordered_map<GraphPersistentIdMsg, VertexIdType,
                                           PersistentIdHasher>;
  // A map from persistent id to vertex id for long-lasting streams indexing
  // into a changing graph
  std::shared_ptr<common::Lockable<PersistentMap>> persistent_map_ =
      std::make_shared<common::Lockable<PersistentMap>>();
};

#if 0
// TODO: Find a way to make explicit instantiation work in debug mode
#if !defined(RCGRAPH_BASE_NO_EXTERN) && defined(NDEBUG)
extern template class RunBase<RCVertex, RCEdge>;
extern template class GraphBase<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>>;

typedef GraphBase<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>> __GraphBaseRC;
EVAL_TYPED_DECLARE_EXTERN(double, __GraphBaseRC)
EVAL_TYPED_DECLARE_EXTERN(bool, __GraphBaseRC)

EVAL_TYPED_DECLARE_EXTERN(double, RCGraphBase)
EVAL_TYPED_DECLARE_EXTERN(bool, RCGraphBase)
#endif
#endif
}  // namespace pose_graph
}  // namespace vtr
