#include <vtr_pose_graph/evaluator/common.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {

namespace Weight {

EVAL_EXPLICIT_INSTANTIATE(Distance, RCGraph);
EVAL_EXPLICIT_INSTANTIATE(Distance, RCGraphBase);
EVAL_EXPLICIT_INSTANTIATE(Distance, BasicGraph);
EVAL_EXPLICIT_INSTANTIATE(Distance, BasicGraphBase);

EVAL_EXPLICIT_INSTANTIATE(Temporal, RCGraph);
EVAL_EXPLICIT_INSTANTIATE(Temporal, RCGraphBase);
EVAL_EXPLICIT_INSTANTIATE(Temporal, BasicGraph);
EVAL_EXPLICIT_INSTANTIATE(Temporal, BasicGraphBase);

EVAL_EXPLICIT_INSTANTIATE(Angle, RCGraph);
EVAL_EXPLICIT_INSTANTIATE(Angle, RCGraphBase);
EVAL_EXPLICIT_INSTANTIATE(Angle, BasicGraph);
EVAL_EXPLICIT_INSTANTIATE(Angle, BasicGraphBase);

EVAL_EXPLICIT_INSTANTIATE(DistAngle, RCGraph);
EVAL_EXPLICIT_INSTANTIATE(DistAngle, RCGraphBase);
EVAL_EXPLICIT_INSTANTIATE(DistAngle, BasicGraph);
EVAL_EXPLICIT_INSTANTIATE(DistAngle, BasicGraphBase);

}  // namespace Weight

namespace Mask {

EVAL_EXPLICIT_INSTANTIATE(Privileged, RCGraph);
EVAL_EXPLICIT_INSTANTIATE(Privileged, RCGraphBase);
EVAL_EXPLICIT_INSTANTIATE(Privileged, BasicGraph);
EVAL_EXPLICIT_INSTANTIATE(Privileged, BasicGraphBase);

EVAL_EXPLICIT_INSTANTIATE(Spatial, RCGraph);
EVAL_EXPLICIT_INSTANTIATE(Spatial, RCGraphBase);
EVAL_EXPLICIT_INSTANTIATE(Spatial, BasicGraph);
EVAL_EXPLICIT_INSTANTIATE(Spatial, BasicGraphBase);

EVAL_EXPLICIT_INSTANTIATE(SimpleTemporal, RCGraph);
EVAL_EXPLICIT_INSTANTIATE(SimpleTemporal, RCGraphBase);
EVAL_EXPLICIT_INSTANTIATE(SimpleTemporal, BasicGraph);
EVAL_EXPLICIT_INSTANTIATE(SimpleTemporal, BasicGraphBase);

// DistanceFromVertex is a special flower, because it doesn't have Caching or
// Windowed variants
template class DirectionFromVertexDirect<RCGraph>;
template class DirectionFromVertexDirect<RCGraphBase>;
template class DirectionFromVertexDirect<BasicGraph>;
template class DirectionFromVertexDirect<BasicGraphBase>;

}  // namespace Mask

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
