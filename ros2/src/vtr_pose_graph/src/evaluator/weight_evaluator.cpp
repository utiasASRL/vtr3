#include <vtr_pose_graph/evaluator/weight_evaluator.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {

EVALUATOR_SCALAR_INTERFACE_CPP(double, double);

EVAL_BASE_EXPLICIT_INSTANTIATE(double);

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
