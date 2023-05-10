#include "vtr_path_planning/mpc/lateral_error_evaluators.hpp"
#include <iostream>

namespace steam {

auto LateralErrorEvaluatorRight::MakeShared(const Evaluable<InType>::ConstPtr& pt,
                                         const InType& meas_pt) -> Ptr {
  return std::make_shared<LateralErrorEvaluatorRight>(pt, meas_pt);
}

LateralErrorEvaluatorRight::LateralErrorEvaluatorRight(
    const Evaluable<InType>::ConstPtr& pt, const InType& meas_pt)
    : pt_(pt), meas_pt_(meas_pt) {
  //D_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  D_(0,1) = 1;

}

bool LateralErrorEvaluatorRight::active() const { return pt_->active(); }

void LateralErrorEvaluatorRight::getRelatedVarKeys(KeySet& keys) const {
  pt_->getRelatedVarKeys(keys);
}

auto LateralErrorEvaluatorRight::value() const -> OutType {
  return D_ * (pt_->value() - meas_pt_);
}

auto LateralErrorEvaluatorRight::forward() const -> Node<OutType>::Ptr {
  const auto child = pt_->forward();
  const auto value = D_ * (child->value() - meas_pt_);
  if (value <= 0.0)
  {
    std::cout << "PROBLEM: The value was: " << value << std::endl;
  }
  const auto node = Node<OutType>::MakeShared(value);
  node->addChild(child);
  return node;
}

void LateralErrorEvaluatorRight::backward(const Eigen::MatrixXd& lhs,
                                       const Node<OutType>::Ptr& node,
                                       Jacobians& jacs) const {
  if (pt_->active()) {
    const auto child = std::static_pointer_cast<Node<InType>>(node->at(0));
    Eigen::MatrixXd new_lhs = lhs * D_;
    pt_->backward(new_lhs, child, jacs);
  }
}

LateralErrorEvaluatorRight::Ptr homo_point_error_right(
    const Evaluable<LateralErrorEvaluatorRight::InType>::ConstPtr& pt,
    const LateralErrorEvaluatorRight::InType& meas_pt) {
  return LateralErrorEvaluatorRight::MakeShared(pt, meas_pt);
}




auto LateralErrorEvaluatorLeft::MakeShared(const Evaluable<InType>::ConstPtr& pt,
                                         const InType& meas_pt) -> Ptr {
  return std::make_shared<LateralErrorEvaluatorLeft>(pt, meas_pt);
}

LateralErrorEvaluatorLeft::LateralErrorEvaluatorLeft(
    const Evaluable<InType>::ConstPtr& pt, const InType& meas_pt)
    : pt_(pt), meas_pt_(meas_pt) {
  //D_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  D_(0,1) = 1;

}

bool LateralErrorEvaluatorLeft::active() const { return pt_->active(); }

void LateralErrorEvaluatorLeft::getRelatedVarKeys(KeySet& keys) const {
  pt_->getRelatedVarKeys(keys);
}

auto LateralErrorEvaluatorLeft::value() const -> OutType {
  return D_ * (meas_pt_ - pt_->value());
}

auto LateralErrorEvaluatorLeft::forward() const -> Node<OutType>::Ptr {
  const auto child = pt_->forward();
  const auto value = D_ * (meas_pt_ - child->value());
  const auto node = Node<OutType>::MakeShared(value);
  node->addChild(child);
  return node;
}

void LateralErrorEvaluatorLeft::backward(const Eigen::MatrixXd& lhs,
                                       const Node<OutType>::Ptr& node,
                                       Jacobians& jacs) const {
  if (pt_->active()) {
    const auto child = std::static_pointer_cast<Node<InType>>(node->at(0));
    Eigen::MatrixXd new_lhs = -lhs * D_;
    pt_->backward(new_lhs, child, jacs);
  }
}

LateralErrorEvaluatorLeft::Ptr homo_point_error_left(
    const Evaluable<LateralErrorEvaluatorLeft::InType>::ConstPtr& pt,
    const LateralErrorEvaluatorLeft::InType& meas_pt) {
  return LateralErrorEvaluatorLeft::MakeShared(pt, meas_pt);
}

}  // namespace steam