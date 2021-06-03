#pragma once

#include <vtr_pose_graph/evaluator/eval_ops.hpp>
#include <vtr_pose_graph/evaluator/evaluator_base.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {

NEW_EVALUATOR_TYPE(Weight, double);

EVALUATOR_SCALAR_INTERFACE_HPP(double, double);

EVAL_BASE_DECLARE_EXTERN(double);

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
