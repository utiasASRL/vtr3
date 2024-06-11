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
 * \file lateral_error_evaluator.cpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/mpc/lateral_error_evaluators.hpp"
#include <iostream>
#include "math.h"
#include "vtr_path_planning/cbit/utils.hpp"


namespace vtr::steam_extension {

  CurvatureInfo CurvatureInfo::fromTransform(const Transformation& T) {
    // Note that this is only along a relative path with an origin at 0,0
    // Using the base tf is still required to move into the world frame
    auto aang = lgmath::so3::rot2vec(T.inverse().C_ba());
    double roc = T.r_ba_ina().norm() / 2 / (sin(aang(2) / 2) + 1e-6);

    static Eigen::Matrix3d rotm_perp;
    rotm_perp << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    auto dist = T.r_ba_ina().norm();
    auto lin_dir = T.r_ba_ina() / dist;

    Eigen::Vector3d coc = T.r_ba_ina() / 2 + sqrt(roc * roc - dist * dist / 4) *
                                                 sgn(roc) * rotm_perp * lin_dir;
    return {coc, roc};
  }

   PathInterpolator::Ptr PathInterpolator::MakeShared(
      const Evaluable<InType>::ConstPtr& tf, const Transformation seq_start,
      const Transformation seq_end) {
      return std::make_shared<PathInterpolator>(tf, seq_start, seq_end);
   }

  bool PathInterpolator::active() const { return false; }

  PathInterpolator::OutType PathInterpolator::value() const {
      Transformation edge = seq_start_.inverse() * seq_end_;
      const auto& [coc, roc] = CurvatureInfo::fromTransform(edge);
      Eigen::Vector4d coc_h{0, 0, 0, 1};
      coc_h.head<3>() = coc;

      coc_h = seq_start_.inverse().matrix() * coc_h;

      const auto interp_ang =
          acos((tf_->value().r_ab_inb() - coc_h.head<3>())
                   .normalized()
                   .dot((seq_start_.r_ab_inb() - coc_h.head<3>()).normalized()));
      const auto interp_full =
          acos((seq_end_.r_ab_inb() - coc_h.head<3>())
                   .normalized()
                   .dot((seq_start_.r_ab_inb() - coc_h.head<3>()).normalized()));
      const double interp = interp_ang / interp_full;
      const auto val = seq_start_ * Transformation(interp * edge.vec(), 0);
      return val;
  }

  typename steam::Node<PathInterpolator::OutType>::Ptr PathInterpolator::forward()
        const {
      return steam::Node<OutType>::MakeShared(value());
  }

}  // namespace steam