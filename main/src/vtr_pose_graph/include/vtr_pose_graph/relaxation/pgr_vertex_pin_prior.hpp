#pragma once

#include <vtr_pose_graph/relaxation/graph_optimization_problem.hpp>

namespace vtr {
namespace pose_graph {

template <class G>
class PGRVertexPinPrior : public OptimizationTypeBase<G> {
 public:
  using Base = OptimizationTypeBase<G>;

  // Explicity import things from the dependent scope
  using GraphPtr = typename Base::GraphPtr;
  using VertexIdType = typename Base::VertexIdType;
  using TransformType = typename Base::TransformType;
  using TfMapType = typename Base::TfMapType;
  using StateMapType = typename Base::StateMapType;
  using CostTermPtr = typename Base::CostTermPtr;

  using ModelGen = NoiseModelGenerator<Eigen::Vector2d, 2>;

  using LossFuncPtr = steam::LossFunctionBase::Ptr;
  using NoiseModelPtr = steam::BaseNoiseModel<2>::Ptr;
  using NoiseModel = steam::StaticNoiseModel<2>;

  PTR_TYPEDEFS(PGRVertexPinPrior);

  /** \brief MakeShared forwarding template to catch all constructors */
  template <typename... Args>
  static Ptr MakeShared(Args&&... args) {
    return Ptr(new PGRVertexPinPrior(std::forward<Args>(args)...));
  }

  /** \brief Build state variables/cost terms and add them to the problem */
  PGRVertexPinPrior(
      VertexId id, Eigen::Vector2d meas, double weight,
      const LossFuncPtr& loss_func = LossFuncPtr(new steam::L2LossFunc()))
      : id_(id),
        meas_(meas),
        noise_model_(
            ModelGen(Eigen::Matrix2d(Eigen::Matrix2d::Identity() / weight))),
        loss_func_(loss_func) {}

  virtual ~PGRVertexPinPrior() {}

  /** \brief Build state variables/cost terms and add them to the problem */
  void addCostTerms(const GraphPtr& graph, const VertexIdType& root,
                    StateMapType& stateMap, CostTermPtr& costTerms,
                    const eval::Mask::Ptr& mask) override {
    LOG(INFO) << "TODO: adding a unary factor.";
  }

 private:
  VertexId id_;

  Eigen::Vector2d meas_;

  ModelGen noise_model_;

  LossFuncPtr loss_func_;
};

}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/relaxation/pose_graph_relaxation.inl>
