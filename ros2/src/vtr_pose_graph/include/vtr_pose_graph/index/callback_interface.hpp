#pragma once

#include <mutex>
#include <vtr_common/utils/macros.hpp>

namespace vtr {

namespace path_planning {
class PlanningInterface;
}

namespace pose_graph {

template <class V, class E, class R>
class CallbackInterface {
 public:
  using RunPtr = typename R::Ptr;
  using EdgePtr = typename E::Ptr;
  using VertexPtr = typename V::Ptr;
  using MutexPtr = std::shared_ptr<std::mutex>;

  PTR_TYPEDEFS(CallbackInterface)

  virtual void runAdded(const RunPtr&) = 0;
  virtual void vertexAdded(const VertexPtr&) = 0;
  virtual void edgeAdded(const EdgePtr&) = 0;

  virtual void updateRelaxation(const MutexPtr& mutex = nullptr) = 0;

  virtual void setPlanner(
      const std::shared_ptr<path_planning::PlanningInterface>&){};
};

template <class V, class E, class R>
class IgnoreCallbacks : public virtual CallbackInterface<V, E, R> {
 public:
  using Base = CallbackInterface<V, E, R>;
  using RunPtr = typename Base::RunPtr;
  using EdgePtr = typename Base::EdgePtr;
  using VertexPtr = typename Base::VertexPtr;
  using MutexPtr = std::shared_ptr<std::mutex>;

  PTR_TYPEDEFS(IgnoreCallbacks)

  IgnoreCallbacks() = default;
  IgnoreCallbacks(const IgnoreCallbacks&) = default;
  IgnoreCallbacks(IgnoreCallbacks&&) = default;

  IgnoreCallbacks& operator=(const IgnoreCallbacks&) = default;
  IgnoreCallbacks& operator=(IgnoreCallbacks&&) = default;

  void runAdded(const RunPtr&) override {
  }
  void vertexAdded(const VertexPtr&) override {
  }
  void edgeAdded(const EdgePtr&) override {
  }

  void updateRelaxation(const MutexPtr& mutex = nullptr) override {
    (void)mutex;
  }
};

}  // namespace pose_graph
}  // namespace vtr
