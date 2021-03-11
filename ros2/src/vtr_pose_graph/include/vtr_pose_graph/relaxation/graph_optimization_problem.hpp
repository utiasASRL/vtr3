#pragma once
#if 0
#include <unordered_map>
#endif
#include <lgmath/se3/TransformationWithCovariance.hpp>
#include <steam.hpp>

#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

namespace vtr {
namespace pose_graph {

template <class G>
class OptimizationTypeBase {
 public:
  using GraphPtr = typename G::Ptr;

  using VertexPtr = typename G::VertexPtr;
  using EdgePtr = typename G::EdgePtr;
  using VertexIdType = typename G::VertexIdType;
  using TransformType = typename G::EdgeType::TransformType;

  using TfMapType = std::unordered_map<VertexId, TransformType>;
  using StateMapType =
      std::unordered_map<VertexIdType, steam::se3::TransformStateVar::Ptr>;
  using CostTermPtr = steam::ParallelizedCostTermCollection::Ptr;

  PTR_TYPEDEFS(OptimizationTypeBase)

  OptimizationTypeBase() {
  }
  OptimizationTypeBase(const OptimizationTypeBase&) = default;
  OptimizationTypeBase(OptimizationTypeBase&&) = default;

  OptimizationTypeBase& operator=(const OptimizationTypeBase&) = default;
  OptimizationTypeBase& operator=(OptimizationTypeBase&&) = default;

  virtual ~OptimizationTypeBase() {
  }

  /** \brief Build state variables/cost terms and add them to the problem */
  virtual void addCostTerms(const GraphPtr& graph, const VertexIdType& root,
                            StateMapType& stateMap, CostTermPtr& costTerms,
                            const eval::Mask::Ptr& mask) = 0;
};

template <class G>
class GraphOptimizationProblem {
 public:
  using GraphPtr = typename G::Ptr;

  using VertexPtr = typename G::VertexPtr;
  using EdgePtr = typename G::EdgePtr;
  using VertexIdType = typename G::VertexIdType;
  using TransformType = typename G::EdgeType::TransformType;

  using TfMapType = std::unordered_map<VertexId, TransformType>;
  using StateMapType =
      std::unordered_map<VertexIdType, steam::se3::TransformStateVar::Ptr>;
  using CostTermPtr = steam::ParallelizedCostTermCollection::Ptr;
  using ProblemPtr = std::shared_ptr<steam::OptimizationProblem>;

  PTR_TYPEDEFS(GraphOptimizationProblem)

  /** \brief automatically initializes vertices to tree expansion */
  GraphOptimizationProblem(
      const GraphPtr& graph, const VertexIdType& root,
      const TfMapType& init = TfMapType(),
      const eval::Mask::Ptr& mask = eval::Mask::Const::MakeShared(true, true));

  GraphOptimizationProblem(const GraphOptimizationProblem&) = default;
  GraphOptimizationProblem(GraphOptimizationProblem&&) = default;

  GraphOptimizationProblem& operator=(const GraphOptimizationProblem&) =
      default;
  GraphOptimizationProblem& operator=(GraphOptimizationProblem&&) = default;

  /** \brief Lock a state variable */
  void setLock(const VertexIdType& v, bool locked = true) {
    stateMap_.at(v)->setLock(locked);
  }

  /** \brief Lock a vector of state variables */
  void setLock(const std::vector<VertexIdType>& v, bool locked = true) {
    for (auto&& it : v) stateMap_.at(it)->setLock(locked);
  }

  /** \brief Visitor function to build a set of cost terms */
  inline void registerComponent(
      typename OptimizationTypeBase<G>::Ptr component) {
    component->addCostTerms(graph_, root_, stateMap_, costTerms_, mask_);
  }

  /** \brief Solve the optimization problem using a given solver */
  template <class Solver>
  void optimize(
      const typename Solver::Params& params = typename Solver::Params());

  /** \brief Get a transform by vertex ID */
  const lgmath::se3::Transformation& at(const VertexIdType& v) const {
    return stateMap_.at(v)->getValue();
  };

  /** \brief Get a transform by vertex ID */
  inline typename StateMapType::const_iterator begin() const {
    return stateMap_.begin();
  }

  /** \brief Get a transform by vertex ID */
  inline typename StateMapType::const_iterator end() const {
    return stateMap_.end();
  }

  /** \brief Get the transform between two vertex IDs */
  lgmath::se3::Transformation T_ab(const VertexIdType& v_a,
                                   const VertexIdType& v_b) const {
    return stateMap_.at(v_a)->getValue() / stateMap_.at(v_b)->getValue();
  }

  /** \brief Apply the optimization to the graph */
  void apply() const {
    for (auto it = graph_->beginEdge(), ite = graph_->endEdge(); it != ite;
         ++it)
      graph_->at(it->id())->setTransform(this->T_ab(it->from(), it->to()));
  }

 protected:
  GraphPtr graph_;

  VertexIdType root_;

  StateMapType stateMap_;

  CostTermPtr costTerms_;

  eval::Mask::Ptr mask_;

  // ProblemPtr problem_;
};

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