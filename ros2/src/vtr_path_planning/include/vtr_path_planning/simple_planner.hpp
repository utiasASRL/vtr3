#pragma once

#include <vtr_path_planning/planning_interface.hpp>

namespace vtr {
namespace path_planning {

template <class GRAPH>
class SimplePlanner : public PlanningInterface {
 public:
  PTR_TYPEDEFS(SimplePlanner)
  using GraphBasePtr = decltype(std::declval<GRAPH>().getManualSubgraph());

  SimplePlanner(const typename GRAPH::Ptr &graph) : graph_(graph) {
    updatePrivileged();
  }

  virtual PathType path(const VertexId &from, const VertexId &to);
  virtual PathType path(const VertexId &from, const VertexId::List &to,
                        std::list<uint64_t> *idx = nullptr);

  virtual void updatePrivileged();

  virtual double cost(const VertexId &) { return 1; }
  virtual double cost(const EdgeId &) { return 1; }

  virtual EvalPtr cost() {
    return pose_graph::eval::Weight::Const::MakeShared(1, 1);
  }

 private:
  PathType _path(const VertexId &from, const VertexId &to);

  typename GRAPH::Ptr graph_;

  GraphBasePtr privileged_;
};

}  // namespace path_planning
}  // namespace vtr

#include <vtr_path_planning/simple_planner.inl>
