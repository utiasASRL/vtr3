#pragma once

#include <vtr_pose_graph/evaluator/eval_ops.hpp>
#include <vtr_pose_graph/evaluator/evaluator_base.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {

NEW_EVALUATOR_TYPE(Mask, bool);

EVALUATOR_BOOLEAN_INTERFACE_HPP(bool, bool);
EVALUATOR_COMPARISON_INTERFACE_HPP(bool);

EVAL_BASE_DECLARE_EXTERN(bool);

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
