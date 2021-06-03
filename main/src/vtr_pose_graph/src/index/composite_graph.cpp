#include <vtr_pose_graph/index/composite_graph.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

namespace vtr {
namespace pose_graph {

// Explicit template instantiation so that these are actually compiled into the
// library
template class CompositeGraph<RCGraph>;
template class CompositeGraph<BasicGraph>;

}  // namespace pose_graph
}  // namespace vtr
