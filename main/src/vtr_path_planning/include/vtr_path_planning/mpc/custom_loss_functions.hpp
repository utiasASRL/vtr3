// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file custom_loss_functions.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

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
  double weight(double) const override { return weight_val; }

};

}  // namespace steam