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
  return log(v_->value());
}

template <int DIM>
auto ScalarLogBarrierEvaluator<DIM>::forward() const -> typename Node<OutType>::Ptr {
  const auto child = v_->forward();
  const auto value = log(child->value());
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
    v_->backward(1.0 /(lhs), child, jacs);
  }
}

// clang-format off
template <int DIM>
typename ScalarLogBarrierEvaluator<DIM>::Ptr slogbar(
    const typename Evaluable<typename ScalarLogBarrierEvaluator<DIM>::InType>::ConstPtr& v) {
  return ScalarLogBarrierEvaluator<DIM>::MakeShared(v);
}
// clang-format on

}  // namespace vspace
}  // namespace steam