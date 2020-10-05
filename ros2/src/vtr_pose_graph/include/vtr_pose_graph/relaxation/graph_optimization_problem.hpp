#pragma once
#if 0
#include <unordered_map>
#endif
#include <lgmath/se3/TransformationWithCovariance.hpp>
#if 0
#include <steam.hpp>
#endif

#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

namespace vtr {
namespace pose_graph {
#if 0
template <class G>
class OptimizationTypeBase {
 public:
  typedef typename G::Ptr GraphPtr;

  typedef typename G::VertexPtr VertexPtr;
  typedef typename G::EdgePtr EdgePtr;
  typedef typename G::VertexIdType VertexIdType;
  typedef typename G::EdgeType::TransformType TransformType;

  typedef std::unordered_map<VertexId, TransformType> TfMapType;
  typedef std::unordered_map<VertexIdType, steam::se3::TransformStateVar::Ptr>
      StateMapType;
  typedef steam::ParallelizedCostTermCollection::Ptr CostTermPtr;

  PTR_TYPEDEFS(OptimizationTypeBase)
  DEFAULT_COPY_MOVE(OptimizationTypeBase)

  OptimizationTypeBase() {}

  virtual ~OptimizationTypeBase() {}

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Build state variables/cost terms and add them to the problem
  /////////////////////////////////////////////////////////////////////////////
  virtual void addCostTerms(const GraphPtr& graph, const VertexIdType& root,
                            StateMapType& stateMap, CostTermPtr& costTerms,
                            const Eval::Mask::Ptr& mask) = 0;
};

template <class G>
class GraphOptimizationProblem {
 public:
  typedef typename G::Ptr GraphPtr;

  typedef typename G::VertexPtr VertexPtr;
  typedef typename G::EdgePtr EdgePtr;
  typedef typename G::VertexIdType VertexIdType;
  typedef typename G::EdgeType::TransformType TransformType;

  typedef std::unordered_map<VertexId, TransformType> TfMapType;
  typedef std::unordered_map<VertexIdType, steam::se3::TransformStateVar::Ptr>
      StateMapType;
  typedef steam::ParallelizedCostTermCollection::Ptr CostTermPtr;
  typedef __shared_ptr<steam::OptimizationProblem> ProblemPtr;

  PTR_TYPEDEFS(GraphOptimizationProblem)
  DEFAULT_COPY_MOVE(GraphOptimizationProblem)

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Constructor; automatically initializes vertices to tree expansion
  /////////////////////////////////////////////////////////////////////////////
  GraphOptimizationProblem(
      const GraphPtr& graph, const VertexIdType& root,
      const TfMapType& init = TfMapType(),
      const Eval::Mask::Ptr& mask = Eval::Mask::Const::MakeShared(true, true));

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Lock a state variable
  /////////////////////////////////////////////////////////////////////////////
  void setLock(const VertexIdType& v, bool locked = true);

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Lock a vector of state variables
  /////////////////////////////////////////////////////////////////////////////
  void setLock(const std::vector<VertexIdType>& v, bool locked = true);

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Visitor function to build a set of cost terms
  /////////////////////////////////////////////////////////////////////////////
  inline void registerComponent(
      typename OptimizationTypeBase<G>::Ptr component) {
    component->addCostTerms(graph_, root_, stateMap_, costTerms_, mask_);
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Solve the optimization problem using a given solver
  /////////////////////////////////////////////////////////////////////////////
  template <class Solver>
  void optimize(
      const typename Solver::Params& params = typename Solver::Params());

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Get a transform by vertex ID
  /////////////////////////////////////////////////////////////////////////////
  const lgmath::se3::Transformation& at(const VertexIdType& v) const;

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Get a transform by vertex ID
  /////////////////////////////////////////////////////////////////////////////
  inline typename StateMapType::const_iterator begin() const {
    return stateMap_.begin();
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Get a transform by vertex ID
  /////////////////////////////////////////////////////////////////////////////
  inline typename StateMapType::const_iterator end() const {
    return stateMap_.end();
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Get the transform between two vertex IDs
  /////////////////////////////////////////////////////////////////////////////
  lgmath::se3::Transformation T_ab(const VertexIdType& v_a,
                                   const VertexIdType& v_b) const;

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Apply the optimization to the graph
  /////////////////////////////////////////////////////////////////////////////
  void apply() const;

 protected:
  GraphPtr graph_;

  VertexIdType root_;

  StateMapType stateMap_;

  CostTermPtr costTerms_;

  Eval::Mask::Ptr mask_;

  // ProblemPtr problem_;
};
#endif
}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/relaxation/graph_optimization_problem.inl>

namespace vtr {
namespace pose_graph {
#if 0
#ifndef GRAPH_OPTIMIZATION_NO_EXTERN
extern template class GraphOptimizationProblem<BasicGraph>;
extern template class GraphOptimizationProblem<RCGraph>;
#endif
#endif

}  // namespace pose_graph
}  // namespace vtr