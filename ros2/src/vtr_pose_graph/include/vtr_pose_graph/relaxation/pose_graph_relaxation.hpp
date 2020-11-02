#pragma once
#if 0
#include <Eigen/Core>
#endif
#include <vtr_pose_graph/relaxation/graph_optimization_problem.hpp>

namespace vtr {
namespace pose_graph {
#if 0
template <class G>
class PoseGraphRelaxation : public OptimizationTypeBase<G> {
 public:
  typedef OptimizationTypeBase<G> Base;

  // Explicity import things from the dependent scope
  typedef typename Base::GraphPtr GraphPtr;
  typedef typename Base::VertexIdType VertexIdType;
  typedef typename Base::TransformType TransformType;
  typedef typename Base::TfMapType TfMapType;
  typedef typename Base::StateMapType StateMapType;
  typedef typename Base::CostTermPtr CostTermPtr;

  typedef NoiseModelGenerator<TransformType, 6> ModelGen;

  typedef steam::LossFunctionBase::Ptr LossFuncPtr;
  typedef steam::BaseNoiseModel<6>::Ptr NoiseModelPtr;
  typedef steam::StaticNoiseModel<6> NoiseModel;

  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  PTR_TYPEDEFS(PoseGraphRelaxation)
  DEFAULT_COPY_MOVE(PoseGraphRelaxation)

  /////////////////////////////////////////////////////////////////////////////
  /// @brief MakeShared forwarding template to catch all constructors
  /////////////////////////////////////////////////////////////////////////////
  template <typename... Args>
  static Ptr MakeShared(Args&&... args) {
    return Ptr(new PoseGraphRelaxation(std::forward<Args>(args)...));
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Build state variables/cost terms and add them to the problem
  /////////////////////////////////////////////////////////////////////////////
  PoseGraphRelaxation(
      const Matrix6d& cov = Matrix6d::Identity(6, 6),
      const LossFuncPtr& lossFunc = LossFuncPtr(new steam::L2LossFunc()));

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Build state variables/cost terms and add them to the problem
  /////////////////////////////////////////////////////////////////////////////
  PoseGraphRelaxation(
      const Matrix6d& covTemporal, const Matrix6d& covSpatial,
      const LossFuncPtr& lossFunc = LossFuncPtr(new steam::L2LossFunc()));

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Build state variables/cost terms and add them to the problem
  /////////////////////////////////////////////////////////////////////////////
  PoseGraphRelaxation(
      const NoiseModelPtr& noiseModel,
      const LossFuncPtr& lossFunc = LossFuncPtr(new steam::L2LossFunc()));

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Build state variables/cost terms and add them to the problem
  /////////////////////////////////////////////////////////////////////////////
  PoseGraphRelaxation(
      const NoiseModelPtr& noiseModelTemporal,
      const NoiseModelPtr& noiseModelSpatial,
      const LossFuncPtr& lossFunc = LossFuncPtr(new steam::L2LossFunc()));

  virtual ~PoseGraphRelaxation() {}

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Build state variables/cost terms and add them to the problem
  /////////////////////////////////////////////////////////////////////////////
  virtual void addCostTerms(const GraphPtr& graph, const VertexIdType& root,
                            StateMapType& stateMap, CostTermPtr& costTerms,
                            const Eval::Mask::Ptr& mask);

 private:
  std::array<ModelGen, 2> noiseModels_;

  LossFuncPtr lossFunc_;
};

#ifndef POSE_GRAPH_RELAXATION_NO_EXTERN
extern template class PoseGraphRelaxation<BasicGraph>;
extern template class PoseGraphRelaxation<RCGraph>;
#endif
#endif
}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/relaxation/pose_graph_relaxation.inl>
