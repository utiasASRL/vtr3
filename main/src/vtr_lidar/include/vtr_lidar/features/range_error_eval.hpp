/**
 * \file range_error_eval.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * STEAM-compatible scalar range error evaluator for lidar intensity features.
 *
 * Error model (1-D):
 *   e_m = r_{m,2} − ‖T_m · p_{m,1}‖
 *
 * where T_m is the motion-distorted pose at measurement time t_m
 * (obtained from a trajectory interpolator), p_{m,1} is the 3-D point
 * in the reference (sensor) frame, and r_{m,2} is the measured range of
 * the matched feature in the current frame.
 *
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <memory>

#include <lgmath.hpp>
#include <steam/evaluable/evaluable.hpp>

namespace vtr {
namespace lidar {

class RangeErrorEval
    : public steam::Evaluable<Eigen::Matrix<double, 1, 1>> {
 public:
  using Ptr = std::shared_ptr<RangeErrorEval>;
  using ConstPtr = std::shared_ptr<const RangeErrorEval>;

  using PoseType = lgmath::se3::Transformation;
  using OutType = Eigen::Matrix<double, 1, 1>;

  /// Factory
  static Ptr MakeShared(
      const steam::Evaluable<PoseType>::ConstPtr& T_m,
      const Eigen::Vector3d& p1,
      double range2) {
    return std::make_shared<RangeErrorEval>(T_m, p1, range2);
  }

  /// Constructor
  /// @param T_m     Pose evaluable at measurement time (maps p1 into obs frame)
  /// @param p1      3-D reference point in frame k (sensor frame)
  /// @param range2  Measured range of the matched feature in frame k+1
  RangeErrorEval(
      const steam::Evaluable<PoseType>::ConstPtr& T_m,
      const Eigen::Vector3d& p1,
      double range2)
      : T_m_(T_m), p1_(p1), range2_(range2) {}

  bool active() const override { return T_m_->active(); }

  void getRelatedVarKeys(KeySet& keys) const override {
    T_m_->getRelatedVarKeys(keys);
  }

  OutType value() const override {
    const Eigen::Vector3d g = transformPoint(T_m_->value());
    OutType error;
    error(0, 0) = range2_ - std::max(g.norm(), kMinRange);
    return error;
  }

  steam::Node<OutType>::Ptr forward() const override {
    const auto child = T_m_->forward();
    const Eigen::Vector3d g = transformPoint(child->value());

    OutType error;
    error(0, 0) = range2_ - std::max(g.norm(), kMinRange);

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
    const Eigen::Vector3d g = transformPoint(child->value());
    const double range = g.norm();
    if (range < kMinRange) return;

    // ∂r/∂g  (1×3)
    const Eigen::Matrix<double, 1, 3> dr_dg = g.transpose() / range;

    // g^⊙ top 3 rows (3×6) — from point2fs which gives 4×6
    const Eigen::Matrix<double, 4, 6> odot_full = lgmath::se3::point2fs(g);
    const Eigen::Matrix<double, 3, 6> odot_3x6 = odot_full.topRows<3>();

    // ∂e/∂T = −∂r/∂g · odot  (1×6)
    const Eigen::Matrix<double, 1, 6> de_dT = -dr_dg * odot_3x6;

    T_m_->backward(lhs * de_dT, child, jacs);
  }

 private:
  Eigen::Vector3d transformPoint(const PoseType& T) const {
    const Eigen::Vector4d p1h = (Eigen::Vector4d() << p1_, 1.0).finished();
    return (T * p1h).head<3>();
  }

  static constexpr double kMinRange = 1e-6;

  steam::Evaluable<PoseType>::ConstPtr T_m_;
  Eigen::Vector3d p1_;
  double range2_;
};

}  // namespace lidar
}  // namespace vtr
