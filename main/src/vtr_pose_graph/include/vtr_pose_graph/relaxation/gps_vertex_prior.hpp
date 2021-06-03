#pragma once
#if 0
#include <Eigen/Core>

#include <vtr_pose_graph/index/rc_graph.hpp>
#include <vtr_pose_graph/relaxation/graph_optimization_problem.hpp>
//#include <asrl/messages/GPSMeasurement.pb.h>
#include <babelfish_robochunk_sensor_msgs/NavSatFix.pb.h>
#endif
namespace vtr {
namespace pose_graph {
#if 0
struct GpsPoint {
  GpsPoint(const Eigen::Vector3d& point) : point_(point), covSet_(false) {}
  GpsPoint(const Eigen::Vector3d& point, const Eigen::Matrix3d& cov)
      : point_(point), cov_(cov), covSet_(true) {}

  Eigen::Vector3d point_;
  Eigen::Matrix3d cov_;
  bool covSet_;
};

class GpsVertexPrior : public OptimizationTypeBase<RCGraph> {
 public:
  typedef OptimizationTypeBase<RCGraph> Base;

  // Explicity import things from the dependent scope
  typedef RCGraph::Ptr GraphPtr;
  typedef RCGraph::VertexIdType VertexIdType;
  typedef RCGraph::TransformType TransformType;
  typedef typename Base::TfMapType TfMapType;
  typedef typename Base::StateMapType StateMapType;
  typedef typename Base::CostTermPtr CostTermPtr;

  typedef babelfish::robochunk::sensor_msgs::NavSatFix MsgType;

  typedef steam::LossFunctionBase::Ptr LossFuncPtr;
  typedef steam::StaticNoiseModel<3> NoiseModel;
  typedef steam::BaseNoiseModel<3>::Ptr NoiseModelPtr;

  static constexpr char gpsStream[] = "/DGPS/fix";

  PTR_TYPEDEFS(GpsVertexPrior)
  DEFAULT_COPY_MOVE(GpsVertexPrior)

  /////////////////////////////////////////////////////////////////////////////
  /// @brief MakeShared forwarding template to catch all constructors
  /////////////////////////////////////////////////////////////////////////////
  template <typename... Args>
  static Ptr MakeShared(Args&&... args) {
    return Ptr(new GpsVertexPrior(std::forward<Args>(args)...));
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Build state variables/cost terms and add them to the problem
  /////////////////////////////////////////////////////////////////////////////
  GpsVertexPrior(
      const Eigen::Matrix3d& cov = Eigen::Matrix3d::Identity(),
      const TransformType& T_gps_base = TransformType(), bool useAdj = false,
      const LossFuncPtr& lossFunc = LossFuncPtr(new steam::L2LossFunc()));

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Build state variables/cost terms and add them to the problem
  /////////////////////////////////////////////////////////////////////////////
  GpsVertexPrior(
      const NoiseModelPtr& noiseModel,
      const TransformType& T_gps_base = TransformType(), bool useAdj = false,
      const LossFuncPtr& lossFunc = LossFuncPtr(new steam::L2LossFunc()));

  virtual ~GpsVertexPrior() {}

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Build state variables/cost terms and add them to the problem
  /////////////////////////////////////////////////////////////////////////////
  virtual void addCostTerms(const GraphPtr& graph, const VertexIdType& root,
                            StateMapType& stateMap, CostTermPtr& costTerms,
                            const Eval::Mask::Ptr& mask);

 private:
  NoiseModelPtr noiseModel_;

  TransformType T_gps_base_;

  LossFuncPtr lossFunc_;

  bool useAdj_;
};

#endif
}  // namespace pose_graph
}  // namespace vtr

//#include <asrl/pose_graph/relaxation/implementation/GpsVertexPrior-impl.hpp>