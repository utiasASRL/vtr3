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
 * \file range_conditioning_eval.hpp
 * \brief
 * \details
 *
 * \author Sean Anderson, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <steam/evaluator/ErrorEvaluator.hpp>
#include <steam/state/LandmarkStateVar.hpp>

namespace vtr {
namespace steam_extensions {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief
//////////////////////////////////////////////////////////////////////////////////////////////
class RangeConditioningEval : public steam::ErrorEvaluator<1, 3>::type {
 public:
  /// Convenience typedefs
  typedef boost::shared_ptr<RangeConditioningEval> Ptr;
  typedef boost::shared_ptr<const RangeConditioningEval> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  RangeConditioningEval(const steam::se3::LandmarkStateVar::ConstPtr& landmark);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state
  /// variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Matrix<double, 1, 1> evaluate() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Matrix<double, 1, 1> evaluate(
      const Eigen::Matrix<double, 1, 1>& lhs,
      std::vector<steam::Jacobian<1, 3> >* jacs) const;

 private:
  Eigen::Matrix<double, 1, 1> meas_;

  steam::se3::LandmarkStateVar::ConstPtr landmark_;
};

}  // namespace steam_extensions
}  // namespace vtr
