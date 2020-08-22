#if 0
#define BASIC_GRAPH_NO_EXTERN

#include <vtr_pose_graph/index/graph.hpp>

namespace vtr {
namespace pose_graph {

template class RunBase<VertexBase, EdgeBase>;
template class GraphBase<VertexBase, EdgeBase, RunBase<VertexBase, EdgeBase>>;
template class Graph<VertexBase, EdgeBase, RunBase<VertexBase, EdgeBase>>;

EVAL_TYPED_EXPLICIT_INSTANTIATE(double, BasicGraphBase)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, BasicGraphBase)

EVAL_TYPED_EXPLICIT_INSTANTIATE(double, BasicGraph)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, BasicGraph)

}  // namespace pose_graph
}  // namespace vtr
#endif