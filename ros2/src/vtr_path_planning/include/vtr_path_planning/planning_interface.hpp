#pragma once

#include <vtr_pose_graph/evaluator/weight_evaluator.hpp>

namespace vtr {
namespace path_planning {

using vtr::pose_graph::EdgeId;
using vtr::pose_graph::VertexId;
using PathType = vtr::pose_graph::VertexId::Vector;

class PlanningInterface {
 public:
  using EvalPtr = vtr::pose_graph::eval::Weight::Ptr;
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
