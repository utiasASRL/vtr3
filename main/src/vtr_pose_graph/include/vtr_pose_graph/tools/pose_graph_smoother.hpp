#include "vtr_pose_graph/index/graph.hpp"
#include "vtr_pose_graph/evaluator/evaluators.hpp"

namespace vtr::pose_graph {
template <class V, class E>
class GraphSmoother {
 public:
  PTR_TYPEDEFS(GraphSmoother);
  using Base = GraphBase<V, E>;
  using GraphPtr = typename Graph<V, E>::Ptr;
  using Smoother = GraphSmoother<V, E>;
  using PrivEval = eval::mask::privileged::Eval<typename GraphSmoother<V, E>::Base>;


  GraphSmoother(const GraphPtr graph) : graph_{graph}, priv_eval_{std::make_shared<PrivEval>(*graph)} 
  {
    priv_graph_ = graph_->getSubgraph(priv_eval_);
  }

  std::set<EdgeId> findBranchEdges() const;

  void runBranchSmoothing() const;

 private:
  GraphPtr graph_;
  typename Base::Ptr priv_graph_;
  typename PrivEval::Ptr priv_eval_;

  void smoothBranch(const EdgeId) const;

};
}  // namespace vtr::pose_graph

#include "pose_graph_smoother.inl"