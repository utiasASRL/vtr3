#pragma once

#include <Eigen/Core>

#include "steam/evaluable/evaluable.hpp"

namespace steam {
namespace vspace {

template <int DIM = Eigen::Dynamic>
class ScalarLogBarrierEvaluator : public Evaluable<Eigen::Matrix<double, DIM, 1>> {
 public:
  using Ptr = std::shared_ptr<ScalarLogBarrierEvaluator>;
  using ConstPtr = std::shared_ptr<const ScalarLogBarrierEvaluator>;

  using InType = Eigen::Matrix<double, DIM, 1>;
  using OutType = Eigen::Matrix<double, DIM, 1>;

  static Ptr MakeShared(const typename Evaluable<InType>::ConstPtr& v);
  ScalarLogBarrierEvaluator(const typename Evaluable<InType>::ConstPtr& v);

  bool active() const override;
  using KeySet = typename Evaluable<OutType>::KeySet;
  void getRelatedVarKeys(KeySet& keys) const override;

  OutType value() const override;
  typename Node<OutType>::Ptr forward() const override;
  void backward(const Eigen::MatrixXd& lhs,
                const typename Node<OutType>::Ptr& node,
                Jacobians& jacs) const override;

 private:
  const typename Evaluable<InType>::ConstPtr v_;
};

// clang-format off
template <int DIM>
typename ScalarLogBarrierEvaluator<DIM>::Ptr slogbar(
    const typename Evaluable<typename ScalarLogBarrierEvaluator<DIM>::InType>::ConstPtr& v);
// clang-format on

}  // namespace vspace
}  // namespace steam

namespace steam {
namespace vspace {

template <int DIM>
auto ScalarLogBarrierEvaluator<DIM>::MakeShared(
    const typename Evaluable<InType>::ConstPtr& v) -> Ptr {
  return std::make_shared<ScalarLogBarrierEvaluator>(v);
}

template <int DIM>
ScalarLogBarrierEvaluator<DIM>::ScalarLogBarrierEvaluator(
    const typename Evaluable<InType>::ConstPtr& v)
    : v_(v) {}

template <int DIM>
bool ScalarLogBarrierEvaluator<DIM>::active() const {
  return v_->active();
}

template <int DIM>
void ScalarLogBarrierEvaluator<DIM>::getRelatedVarKeys(KeySet& keys) const {
  v_->getRelatedVarKeys(keys);
}

template <int DIM>
auto ScalarLogBarrierEvaluator<DIM>::value() const -> OutType {
  auto v_intermed = v_->value();
  v_intermed[0] = (v_->value()[0]); // TODO. need to validate this, not 100% sure this is the best/correct way to do this log
  return v_intermed;
}

template <int DIM>
auto ScalarLogBarrierEvaluator<DIM>::forward() const -> typename Node<OutType>::Ptr {
  const auto child = v_->forward();
  auto value = child->value(); 
  value[0] = log((child->value())[0]); // TODO. need to validate this, not 100% sure this is the best/correct way to do this log
  //const auto value = log((child->value()[0]));
  const auto node = Node<OutType>::MakeShared(value);
  node->addChild(child);
  return node;
}

template <int DIM>
void ScalarLogBarrierEvaluator<DIM>::backward(const Eigen::MatrixXd& lhs,
                                        const typename Node<OutType>::Ptr& node,
                                        Jacobians& jacs) const {
  const auto child = std::static_pointer_cast<Node<InType>>(node->at(0));
  if (v_->active()) {
    v_->backward(lhs.inverse(), child, jacs);
  }
}

// clang-format off
template <int DIM>
typename ScalarLogBarrierEvaluator<DIM>::Ptr slogbar(
    const typename Evaluable<typename ScalarLogBarrierEvaluator<DIM>::InType>::ConstPtr& v) {
  return ScalarLogBarrierEvaluator<DIM>::MakeShared(v);
}
// clang-format on







// Inverse barrier term (EXPERIMENTAL)
template <int DIM = Eigen::Dynamic>
class ScalarInverseBarrierEvaluator : public Evaluable<Eigen::Matrix<double, DIM, 1>> {
 public:
  using Ptr = std::shared_ptr<ScalarInverseBarrierEvaluator>;
  using ConstPtr = std::shared_ptr<const ScalarInverseBarrierEvaluator>;

  using InType = Eigen::Matrix<double, DIM, 1>;
  using OutType = Eigen::Matrix<double, DIM, 1>;

  static Ptr MakeShared(const typename Evaluable<InType>::ConstPtr& v);
  ScalarInverseBarrierEvaluator(const typename Evaluable<InType>::ConstPtr& v);

  bool active() const override;
  using KeySet = typename Evaluable<OutType>::KeySet;
  void getRelatedVarKeys(KeySet& keys) const override;

  OutType value() const override;
  typename Node<OutType>::Ptr forward() const override;
  void backward(const Eigen::MatrixXd& lhs,
                const typename Node<OutType>::Ptr& node,
                Jacobians& jacs) const override;

 private:
  const typename Evaluable<InType>::ConstPtr v_;
};

// clang-format off
template <int DIM>
typename ScalarInverseBarrierEvaluator<DIM>::Ptr sinvbar(
    const typename Evaluable<typename ScalarInverseBarrierEvaluator<DIM>::InType>::ConstPtr& v);
// clang-format on

}  // namespace vspace
}  // namespace steam

namespace steam {
namespace vspace {

template <int DIM>
auto ScalarInverseBarrierEvaluator<DIM>::MakeShared(
    const typename Evaluable<InType>::ConstPtr& v) -> Ptr {
  return std::make_shared<ScalarInverseBarrierEvaluator>(v);
}

template <int DIM>
ScalarInverseBarrierEvaluator<DIM>::ScalarInverseBarrierEvaluator(
    const typename Evaluable<InType>::ConstPtr& v)
    : v_(v) {}

template <int DIM>
bool ScalarInverseBarrierEvaluator<DIM>::active() const {
  return v_->active();
}

template <int DIM>
void ScalarInverseBarrierEvaluator<DIM>::getRelatedVarKeys(KeySet& keys) const {
  v_->getRelatedVarKeys(keys);
}

template <int DIM>
auto ScalarInverseBarrierEvaluator<DIM>::value() const -> OutType {
  auto v_intermed = v_->value();
  v_intermed[0] = (v_->value()[0]); // TODO. need to validate this, not 100% sure this is the best/correct way to do this log
  return v_intermed;
}

template <int DIM>
auto ScalarInverseBarrierEvaluator<DIM>::forward() const -> typename Node<OutType>::Ptr {
  const auto child = v_->forward();
  auto value = child->value(); 
  value[0] = 1.0 / (child->value())[0]; // TODO. need to validate this, not 100% sure this is the best/correct way to do this inv
  //const auto value = log((child->value()[0]));
  const auto node = Node<OutType>::MakeShared(value);
  node->addChild(child);
  return node;
}

template <int DIM>
void ScalarInverseBarrierEvaluator<DIM>::backward(const Eigen::MatrixXd& lhs,
                                        const typename Node<OutType>::Ptr& node,
                                        Jacobians& jacs) const {
  const auto child = std::static_pointer_cast<Node<InType>>(node->at(0));
  if (v_->active()) {
    v_->backward(-1.0 * (lhs * lhs).inverse(), child, jacs);
  }
}

// clang-format off
template <int DIM>
typename ScalarInverseBarrierEvaluator<DIM>::Ptr sinvbar(
    const typename Evaluable<typename ScalarInverseBarrierEvaluator<DIM>::InType>::ConstPtr& v) {
  return ScalarInverseBarrierEvaluator<DIM>::MakeShared(v);
}
// clang-format on

}  // namespace vspace
}  // namespace steam