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
 * \file lateral_error_evaluators.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

#pragma once

#include "steam/evaluable/state_var.hpp"
#include "steam/evaluable/se3/evaluables.hpp"
#include "lgmath.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr::steam_extension {

  using namespace lgmath::se3;
  using namespace steam;

  struct CurvatureInfo
  {
    Eigen::Vector3d center;
    double radius;

    inline double curvature() const {
      return 1 / radius;
    }

    static CurvatureInfo fromTransform(const Transformation &T_12); 
  };
  

  class PathInterpolator : public Evaluable<Transformation> {
  public:
    using Ptr = std::shared_ptr<PathInterpolator>;
    using ConstPtr = std::shared_ptr<const PathInterpolator>;

    using InType = Transformation;
    using OutType = Transformation;

    static Ptr MakeShared(const Evaluable<InType>::ConstPtr& tf,
                          const Transformation seq_start,
                          const Transformation seq_end);

    PathInterpolator(const Evaluable<InType>::ConstPtr& tf,
                     const Transformation seq_start, Transformation seq_end)
        : tf_{tf}, seq_start_{seq_start}, seq_end_{seq_end} {};

    bool active() const override;
    void getRelatedVarKeys(KeySet& keys) const override{};

    OutType value() const override;
    steam::Node<OutType>::Ptr forward() const override;
    void backward(const Eigen::MatrixXd& lhs,
                  const typename steam::Node<OutType>::Ptr& node,
                  Jacobians& jacs) const override{};

  private:
    /** \brief Transform to vec evaluable */
    const Evaluable<InType>::ConstPtr tf_;
    const se3::PoseInterpolator::Ptr path_;

    const se3::ComposeInverseEvaluator::ConstPtr se3_err_;

    /** Return sequence id of path*/
    const Transformation seq_start_;
    const Transformation seq_end_;
  };
}