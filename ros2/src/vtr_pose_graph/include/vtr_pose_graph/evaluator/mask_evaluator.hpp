#pragma once

#include <vtr_pose_graph/evaluator/eval_ops.hpp>
#include <vtr_pose_graph/evaluator/evaluator_base.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {

NEW_EVALUATOR_TYPE(Mask, bool)

EVALUATOR_BOOLEAN_INTERFACE_HPP(bool, bool)

EVALUATOR_COMPARISON_INTERFACE_HPP(bool)

#if 0
/// \todo (yuchen) Find a way to make explicit instantiation work in debug mode
#if !defined(Mask_EVAL_NO_EXTERN) && defined(NDEBUG)
EVAL_BASE_DECLARE_EXTERN(bool)
#endif
#endif

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
