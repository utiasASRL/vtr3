#pragma once

#include <vtr_logging/logging.hpp>
#include <vtr_pose_graph/relaxation/graph_optimization_problem.hpp>

namespace vtr {
namespace pose_graph {

// Base class for noise model generators
template <class T, size_t DIM>
class NoiseModelGeneratorBase {
 public:
  using MatType = Eigen::Matrix<double, DIM, DIM>;
  using ModelType = steam::StaticNoiseModel<DIM>;
  using ModelPtr = typename steam::BaseNoiseModel<DIM>::Ptr;

  NoiseModelGeneratorBase(const MatType& cov = MatType::Identity())
      : defaultModel_(new ModelType(cov)) {}

  NoiseModelGeneratorBase(const ModelPtr& defaultModel)
      : defaultModel_(defaultModel) {}

  // Return the default model
  virtual inline ModelPtr operator()(const T&) const = 0;

 protected:
  ModelPtr defaultModel_;
};

// Template class that generates noise models for a type, or returns a default
// model. The generic template always returns the default model
template <class T, size_t DIM>
class NoiseModelGenerator : public NoiseModelGeneratorBase<T, DIM> {
 public:
  using Base = NoiseModelGeneratorBase<T, DIM>;
  using ModelPtr = typename Base::ModelPtr;
  using Base::defaultModel_;
  using Base::NoiseModelGeneratorBase;

  // Return the default model
  virtual inline ModelPtr operator()(const T&) const { return defaultModel_; }
};

// Template specialization for transofrms that have a covariance
template <>
class NoiseModelGenerator<lgmath::se3::TransformationWithCovariance, 6>
    : public NoiseModelGeneratorBase<lgmath::se3::TransformationWithCovariance,
                                     6> {
 public:
  typedef NoiseModelGeneratorBase<lgmath::se3::TransformationWithCovariance, 6>
      Base;
  typedef typename Base::ModelPtr ModelPtr;
  using Base::defaultModel_;
  using Base::NoiseModelGeneratorBase;

  // Make a new model and return it
  virtual inline ModelPtr operator()(
      const lgmath::se3::TransformationWithCovariance& T) const {
    /// \todo (yuchen) The extra copy op is due to a weird alignment issue in
    /// Eigen. Old code as reference.
    // if (T.covarianceSet() && T.cov().norm() > 0)
    //   return ModelPtr(new steam::StaticNoiseModel<6>(T.cov()));
    // else
    //   return defaultModel_;
    if (T.covarianceSet()) {
      auto cov = T.cov();
      if (cov.norm() > 0) return ModelPtr(new steam::StaticNoiseModel<6>(cov));
    }
    return defaultModel_;
  }
};

template <class G>
GraphOptimizationProblem<G>::GraphOptimizationProblem(
    const GraphPtr& graph, const VertexIdType& root, const TfMapType& init,
    const eval::Mask::Ptr& mask)
    : graph_(graph),
      root_(root),
      stateMap_(StateMapType()),
      costTerms_(new steam::ParallelizedCostTermCollection()),
      mask_(mask) {
  mask_->setGraph(graph.get());

  // Initialize the root to the identity transform and lock it
  steam::se3::TransformStateVar::Ptr tmpRoot(
      new steam::se3::TransformStateVar());
  tmpRoot->setLock(true);

  // Get a graph iterator in default search-order
  auto it = graph_->begin(root_, 0, mask_);
  stateMap_[root_] = tmpRoot;
  ++it;

  // Loop through all vertex-ancestor pairs, and get T_{vertex}_{from}
  lgmath::se3::Transformation T_ab;
  for (auto ite = graph_->end(); it != ite; ++it) {
    VertexId vid = it->v()->id();
    if (init.find(vid) != init.end()) {
      stateMap_[vid] = steam::se3::TransformStateVar::Ptr(
          new steam::se3::TransformStateVar(init.at(vid)));
    } else {
      const EdgePtr& e = it->e();

      // Check if we traversed the edge "backwards", and invert if necessary
      if (e->from() != it->from())
        T_ab = e->T().inverse();
      else
        T_ab = e->T();

      // T_{vertex}_{root} = T_{vertex}_{from} * T_{from}_{root}
      // We know that the "from" state variable already exists, because we are
      // expanding in search-order
      T_ab *= stateMap_.at(it->from())->getValue();

      // Add the new vertex state variable to the map
      stateMap_[vid] = steam::se3::TransformStateVar::Ptr(
          new steam::se3::TransformStateVar(T_ab));
    }
  }
}

template <class G>
template <class Solver>
void GraphOptimizationProblem<G>::optimize(
    const typename Solver::Params& params) {
  if (params.maxIterations == 0) {
    LOG(INFO) << "Graph optimization skipped since maxIterations==0.";
    return;
  }

  steam::OptimizationProblem problem;

  for (auto&& it : stateMap_) {
    if (!it.second->isLocked())
      problem.addStateVariable(it.second);
    else
      LOG(INFO) << "Skipping locked pose " << it.first;
  }
  problem.addCostTerm(costTerms_);

  if (problem.getStateVariables().size() == 0 ||
      problem.getNumberOfCostTerms() == 0) {
    LOG(INFO) << "Attempted relaxation on an empty/locked problem... Result: 0 "
                 "cost; perfect 5/7!";
    return;
  } else if (problem.cost() < 1) {
    LOG(INFO) << "Skipping relaxation because the cost was too low";
    return;
  }

  try {
    Solver s(&problem, params);
    s.optimize();
  } catch (...) {
    LOG(INFO) << "Graph optimization failed";
  }
}

}  // namespace pose_graph
}  // namespace vtr
