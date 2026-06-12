/**
 * \file reproj_error_eval.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * STEAM-compatible reprojection error evaluator for lidar intensity features.
 *
 * Error model (2-D):
 *   e_m = y_{m,2} − f(T_m · p_{m,1})
 *
 * where T_m is the motion-distorted pose at measurement time t_m
 * (obtained from a trajectory interpolator), p_{m,1} is the 3-D point
 * in the reference (sensor) frame, f(·) is the beam-offset projection
 * (OusterProjector::projectPoint), and y_{m,2} is the observed analytical
 * projection in the current frame.
 *
 * The backward pass chains ∂e/∂T through the STEAM trajectory via the
 * standard odot / point2fs pattern.
 *
 */
#pragma once

#include <Eigen/Core>
#include <memory>

#include <lgmath.hpp>
#include <steam/evaluable/evaluable.hpp>

#include "vtr_lidar/features/ouster_projector.hpp"

namespace vtr {
namespace lidar {

class ReprojErrorEval
    : public steam::Evaluable<Eigen::Matrix<double, 2, 1>> {
 public:
  using Ptr = std::shared_ptr<ReprojErrorEval>;
  using ConstPtr = std::shared_ptr<const ReprojErrorEval>;

  using PoseType = lgmath::se3::Transformation;
  using OutType = Eigen::Matrix<double, 2, 1>;

  /// Factory
  static Ptr MakeShared(
      const steam::Evaluable<PoseType>::ConstPtr& T_m,
      const Eigen::Vector3d& p1,
      const Eigen::Vector2d& y2,
      const std::shared_ptr<const OusterProjector>& projector) {
    return std::make_shared<ReprojErrorEval>(T_m, p1, y2, projector);
  }

  /// Constructor
  /// @param T_m  Pose evaluable at measurement time (maps p1 into obs frame)
  /// @param p1   3-D reference point in frame k (sensor frame)
  /// @param y2   Observed analytical 2-D projection in frame k+1
  /// @param projector  OusterProjector for f(·) and ∂f/∂g
  ReprojErrorEval(
      const steam::Evaluable<PoseType>::ConstPtr& T_m,
      const Eigen::Vector3d& p1,
      const Eigen::Vector2d& y2,
      const std::shared_ptr<const OusterProjector>& projector)
      : T_m_(T_m), p1_(p1), y2_(y2), projector_(projector) {}

  bool active() const override { return T_m_->active(); }

  void getRelatedVarKeys(KeySet& keys) const override {
    T_m_->getRelatedVarKeys(keys);
  }

  OutType value() const override {
    const auto T = T_m_->value();
    const Eigen::Vector4d p1h = (Eigen::Vector4d() << p1_, 1.0).finished();
    const Eigen::Vector3d g = (T * p1h).head<3>();
    Eigen::Vector2d y2_hat;
    projector_->projectPoint(g, y2_hat);
    return y2_ - y2_hat;
  }

  steam::Node<OutType>::Ptr forward() const override {
    const auto child = T_m_->forward();
    const auto T = child->value();

    // g_m = T_m · p1
    const Eigen::Vector4d p1h = (Eigen::Vector4d() << p1_, 1.0).finished();
    const Eigen::Vector3d g = (T * p1h).head<3>();

    // f(g_m)
    Eigen::Vector2d y2_hat;
    projector_->projectPoint(g, y2_hat);

    // error
    OutType error = y2_ - y2_hat;

    const auto node = steam::Node<OutType>::MakeShared(error);
    node->addChild(child);
    return node;
  }

  void backward(const Eigen::MatrixXd& lhs,
                const steam::Node<OutType>::Ptr& node,
                steam::Jacobians& jacs) const override {
    if (!T_m_->active()) return;

    const auto child =
        std::static_pointer_cast<steam::Node<PoseType>>(node->at(0));
    const auto T = child->value();

    // g_m = T_m · p1
    const Eigen::Vector4d p1h = (Eigen::Vector4d() << p1_, 1.0).finished();
    const Eigen::Vector3d g = (T * p1h).head<3>();

    // F_m = ∂f/∂g  (2×3)
    Eigen::Matrix<double, 2, 3> F;
    projector_->projectionJacobian(g, F);

    // g^⊙ top 3 rows (3×6)  — from point2fs which gives 4×6
    const Eigen::Matrix<double, 4, 6> odot_full =
        lgmath::se3::point2fs(g);
    const Eigen::Matrix<double, 3, 6> odot_3x6 = odot_full.topRows<3>();

    // ∂e/∂T = −F · odot  (2×6)
    // (error = y2 - f(T·p1), so ∂e/∂T = -∂f/∂T = -F · odot)
    const Eigen::Matrix<double, 2, 6> de_dT = -F * odot_3x6;

    // Chain: multiply by lhs and pass backward through the trajectory
    T_m_->backward(lhs * de_dT, child, jacs);
  }

 private:
  steam::Evaluable<PoseType>::ConstPtr T_m_;
  Eigen::Vector3d p1_;
  Eigen::Vector2d y2_;
  std::shared_ptr<const OusterProjector> projector_;
};

}  // namespace lidar
}  // namespace vtr
