#include "vtr_path_planning/mpc/custom_steam_evaluators.hpp"

namespace steam {
namespace stereo {

auto HomoPointErrorEvaluatorRight::MakeShared(const Evaluable<InType>::ConstPtr& pt,
                                         const InType& meas_pt) -> Ptr {
  return std::make_shared<HomoPointErrorEvaluatorRight>(pt, meas_pt);
}

HomoPointErrorEvaluatorRight::HomoPointErrorEvaluatorRight(
    const Evaluable<InType>::ConstPtr& pt, const InType& meas_pt)
    : pt_(pt), meas_pt_(meas_pt) {
  //D_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  D_(0,1) = 1;

}

bool HomoPointErrorEvaluatorRight::active() const { return pt_->active(); }

void HomoPointErrorEvaluatorRight::getRelatedVarKeys(KeySet& keys) const {
  pt_->getRelatedVarKeys(keys);
}

auto HomoPointErrorEvaluatorRight::value() const -> OutType {
  return D_ * (pt_->value() - meas_pt_);
}

auto HomoPointErrorEvaluatorRight::forward() const -> Node<OutType>::Ptr {
  const auto child = pt_->forward();
  const auto value = D_ * (child->value() - meas_pt_);
  const auto node = Node<OutType>::MakeShared(value);
  node->addChild(child);
  return node;
}

void HomoPointErrorEvaluatorRight::backward(const Eigen::MatrixXd& lhs,
                                       const Node<OutType>::Ptr& node,
                                       Jacobians& jacs) const {
  if (pt_->active()) {
    const auto child = std::static_pointer_cast<Node<InType>>(node->at(0));
    Eigen::MatrixXd new_lhs = lhs * D_;
    pt_->backward(new_lhs, child, jacs);
  }
}

HomoPointErrorEvaluatorRight::Ptr homo_point_error_right(
    const Evaluable<HomoPointErrorEvaluatorRight::InType>::ConstPtr& pt,
    const HomoPointErrorEvaluatorRight::InType& meas_pt) {
  return HomoPointErrorEvaluatorRight::MakeShared(pt, meas_pt);
}




auto HomoPointErrorEvaluatorLeft::MakeShared(const Evaluable<InType>::ConstPtr& pt,
                                         const InType& meas_pt) -> Ptr {
  return std::make_shared<HomoPointErrorEvaluatorLeft>(pt, meas_pt);
}

HomoPointErrorEvaluatorLeft::HomoPointErrorEvaluatorLeft(
    const Evaluable<InType>::ConstPtr& pt, const InType& meas_pt)
    : pt_(pt), meas_pt_(meas_pt) {
  //D_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  D_(0,1) = 1;

}

bool HomoPointErrorEvaluatorLeft::active() const { return pt_->active(); }

void HomoPointErrorEvaluatorLeft::getRelatedVarKeys(KeySet& keys) const {
  pt_->getRelatedVarKeys(keys);
}

auto HomoPointErrorEvaluatorLeft::value() const -> OutType {
  return D_ * (meas_pt_ - pt_->value());
}

auto HomoPointErrorEvaluatorLeft::forward() const -> Node<OutType>::Ptr {
  const auto child = pt_->forward();
  const auto value = D_ * (meas_pt_ - child->value());
  const auto node = Node<OutType>::MakeShared(value);
  node->addChild(child);
  return node;
}

void HomoPointErrorEvaluatorLeft::backward(const Eigen::MatrixXd& lhs,
                                       const Node<OutType>::Ptr& node,
                                       Jacobians& jacs) const {
  if (pt_->active()) {
    const auto child = std::static_pointer_cast<Node<InType>>(node->at(0));
    Eigen::MatrixXd new_lhs = -lhs * D_;
    pt_->backward(new_lhs, child, jacs);
  }
}

HomoPointErrorEvaluatorLeft::Ptr homo_point_error_left(
    const Evaluable<HomoPointErrorEvaluatorLeft::InType>::ConstPtr& pt,
    const HomoPointErrorEvaluatorLeft::InType& meas_pt) {
  return HomoPointErrorEvaluatorLeft::MakeShared(pt, meas_pt);
}

}  // namespace stereo
}  // namespace steam