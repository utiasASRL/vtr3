#pragma once

#include <mutex>
#include <vtr_pose_graph/index/callback_interface.hpp>
#include <vtr_pose_graph/index/graph_base.hpp>

namespace vtr {
namespace pose_graph {

template <class V, class E, class R>
class Graph : public virtual GraphBase<V, E, R> {
 public:
  using Base = GraphBase<V, E, R>;
  using RType = GraphBase<V, E, R>;

  using Base::edges_;
  using Base::graph_;
  using Base::id_;
  using Base::run;
  using Base::runs_;
  using Base::vertices_;
  using IdType = typename Base::IdType;

  // We have to manually import the typedefs, as they exist in dependent scope
  // and the compiler cannot find them by default
  using RunType = typename Base::RunType;
  using VertexType = typename Base::VertexType;
  using EdgeType = typename Base::EdgeType;

  using VertexPtr = typename Base::VertexPtr;
  using EdgePtr = typename Base::EdgePtr;
  using RunPtr = typename Base::RunPtr;

  using RunIdType = typename Base::RunIdType;
  using VertexIdType = typename Base::VertexIdType;
  using EdgeIdType = typename Base::EdgeIdType;
  using EdgeTypeEnum = typename Base::EdgeTypeEnum;
  using SimpleVertexId = typename Base::SimpleVertexId;
  using SimpleEdgeId = typename Base::SimpleEdgeId;

  using RunMap = typename Base::RunMap;
  using VertexMap = typename Base::VertexMap;
  using EdgeMap = typename Base::EdgeMap;
  using TransformType = typename EdgeType::TransformType;
  using CallbackPtr = typename CallbackInterface<V, E, R>::Ptr;

  using UniqueLock = std::unique_lock<std::recursive_mutex>;
  using LockGuard = std::lock_guard<std::recursive_mutex>;

  PTR_TYPEDEFS(Graph)

  /** \brief Pseudo-constructor to make shared pointers */
  static Ptr MakeShared();
  static Ptr MakeShared(const IdType& id);

  Graph();
  /** \brief Construct an empty graph with an id */
  Graph(const IdType& id);

#if true
  /// Yuchen: we used to allow copying and moving, but I don't think it is
  /// needed.
  Graph(const Graph&) = delete;
  Graph(Graph&& other) = delete;
  Graph& operator=(const Graph&) = delete;
  Graph& operator=(Graph&& other) = delete;
#else
  Graph(const Graph&) = default;
  Graph(Graph&& other)
      : Base(std::move(other)),
        currentRun_(std::move(other.currentRun_)),
        lastRunIdx_(std::move(other.lastRunIdx_)),
        callback_(std::move(other.callback_)) {}

  Graph& operator=(const Graph&) = default;
  Graph& operator=(Graph&& other) {
    Base::operator=(std::move(other));
    this->currentRun_ = std::move(other.currentRun_);
    this->lastRunIdx_ = std::move(other.lastRunIdx_);
    this->callback_ = std::move(other.callback_);
    return *this;
  }
#endif
  /** \brief Set the callback handling procedure */
  void setCallbackMode(const CallbackPtr& callback =
                           CallbackPtr(new IgnoreCallbacks<V, E, R>())) {
    callback_ = callback;
  }

  /** \brief Get a pointer to the callback manager */
  const CallbackPtr& callbacks() const { return callback_; }

  /** \brief Add a new run an increment the run id */
  virtual RunIdType addRun();

  /** \brief Return a blank vertex (current run) with the next available Id */
  virtual VertexPtr addVertex();

  /** \brief Return a blank vertex with the next available Id */
  virtual VertexPtr addVertex(const RunIdType& runId);

  /** \brief Return a blank edge with the next available Id */
  virtual EdgePtr addEdge(const VertexIdType& from, const VertexIdType& to,
                          const EdgeTypeEnum& type = EdgeTypeEnum::Temporal,
                          bool manual = false);

  /** \brief Return a blank edge with the next available Id */
  virtual EdgePtr addEdge(const VertexIdType& from, const VertexIdType& to,
                          const TransformType& T_to_from,
                          const EdgeTypeEnum& type = EdgeTypeEnum::Temporal,
                          bool manual = false);

  /** \brief Acquire a lock object that blocks modifications */
  UniqueLock guard() const { return UniqueLock(mtx_); }
  /** \brief Manually lock the graph, preventing modifications */
  void lock() const { mtx_.lock(); }
  /** \brief Manually unlock the graph, allowing modifications */
  void unlock() const { mtx_.unlock(); }
  /** \brief Get a reference to the mutex */
  std::recursive_mutex& mutex() { return mtx_; }

 protected:
  /** \brief The current run */
  RunPtr currentRun_;

  /** \brief The current maximum run index */
  RunIdType lastRunIdx_;

  /** \brief The current maximum run index */
  CallbackPtr callback_;

  /** \brief Used to lock changes to the graph during long-running operations */
  mutable std::recursive_mutex mtx_;
};

using BasicGraph = Graph<VertexBase, EdgeBase, RunBase<VertexBase, EdgeBase>>;

}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/index/graph.inl>

#if 0
#if !defined(BASIC_GRAPH_NO_EXTERN) && defined(NDEBUG)
namespace asrl {
namespace pose_graph {

extern template class Graph<VertexBase, EdgeBase,
                            RunBase<VertexBase, EdgeBase>>;

EVAL_TYPED_DECLARE_EXTERN(double, BasicGraph)
EVAL_TYPED_DECLARE_EXTERN(bool, BasicGraph)

}  // namespace pose_graph
}  // namespace asrl
#endif
#endif