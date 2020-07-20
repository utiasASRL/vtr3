#pragma once

#include <asrl/pose_graph/evaluator/WeightEvaluator.hpp>

namespace vtr {
namespace path_planning {

using asrl::pose_graph::EdgeId;
using asrl::pose_graph::VertexId;
using PathType = asrl::pose_graph::VertexId::Vector;

class PlanningInterface {
 public:
  using EvalPtr = asrl::pose_graph::Eval::Weight::Ptr;
  PTR_TYPEDEFS(PlanningInterface)

  virtual PathType path(const VertexId& from, const VertexId& to) = 0;
  virtual PathType path(const VertexId& from, const VertexId::List& to,
                        std::list<uint64_t>* idx) = 0;

  virtual void updatePrivileged() = 0;

  virtual double cost(const VertexId&) = 0;
  virtual double cost(const EdgeId&) = 0;

  virtual EvalPtr cost() = 0;
};

}  // namespace path_planning
}  // namespace vtr
