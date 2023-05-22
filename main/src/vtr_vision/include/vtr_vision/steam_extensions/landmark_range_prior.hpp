#pragma once
#include <iostream>

#include "steam/evaluable/evaluable.hpp"
#include "steam/evaluable/stereo/homo_point_state_var.hpp"

namespace steam {
namespace stereo {


/** \brief Stereo camera error function evaluator */
class LandmarkRangePrior : public Evaluable<Eigen::Matrix<double, 1, 1>> {
 public:
  using Ptr = std::shared_ptr<LandmarkRangePrior>;
  using ConstPtr = std::shared_ptr<const LandmarkRangePrior>;

  using LmInType = Eigen::Vector4d;
  using OutType = Eigen::Matrix<double, 1, 1>;

  static Ptr MakeShared(const Evaluable<LmInType>::ConstPtr landmark);
  LandmarkRangePrior(const Evaluable<LmInType>::ConstPtr landmark);

  bool active() const override;
  void getRelatedVarKeys(KeySet& keys) const override;

  OutType value() const override;
  Node<OutType>::Ptr forward() const override;
  void backward(const Eigen::MatrixXd& lhs, const Node<OutType>::Ptr& node,
                Jacobians& jacs) const override;

 private:
  /** \brief Original z cooridinate from stereo camera */
  Eigen::Matrix<double, 1, 1> meas_;

  /**
   * \brief Point evaluator
   * \details evaluates the current landmark position
   */
  const Evaluable<LmInType>::ConstPtr landmark_;
};

} //namespace stereo
} //namespace steam