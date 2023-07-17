#pragma once

#include "steam/problem/loss_func/base_loss_func.hpp"

namespace steam {

/** \brief 'L2' loss function with different weights: TODO long term make a version of this class which is more generic and can dynamically set the weight */
class L2WeightedLossFunc : public BaseLossFunc {
 private:
    double weight_val = 1.0;

 public:
  /** \brief Convenience typedefs */
  using Ptr = std::shared_ptr<L2WeightedLossFunc>;
  using ConstPtr = std::shared_ptr<const L2WeightedLossFunc>;

  static Ptr MakeShared(double weight_input) { return std::make_shared<L2WeightedLossFunc>(weight_input); }

  /** \brief Constructor */
  //L2WeightedLossFunc() = default;


  L2WeightedLossFunc(double weight_input)
  {
    weight_val = weight_input;
  }

  /** \brief Cost function (basic evaluation of the loss function) */
  double cost(double whitened_error_norm) const override {
    return 0.5 * whitened_error_norm * whitened_error_norm;
  }

  /**
   * \brief Weight for iteratively reweighted least-squares (influence function
   * div. by error)
   */
  double weight(double whitened_error_norm) const override { return weight_val; }

};

}  // namespace steam