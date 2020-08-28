#if 0
#define Mask_EVAL_NO_EXTERN
#endif
#include <vtr_pose_graph/evaluator/mask_evaluator.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {

EVALUATOR_BOOLEAN_INTERFACE_CPP(bool, bool);
EVALUATOR_COMPARISON_INTERFACE_CPP(bool);

EVAL_BASE_EXPLICIT_INSTANTIATE(bool);

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr