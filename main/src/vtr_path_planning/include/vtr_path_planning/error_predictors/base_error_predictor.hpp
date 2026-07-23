// Copyright 2026, Autonomous Space Robotics Lab (ASRL)
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
 * \file base_error_predictor.hpp
 * \author Luka Antonyshyn Autonomous Space Robotics Lab (ASRL)
 */

#include <array>
#include <vector>

#include <vtr_logging/logging.hpp>
#include <vtr_tactic/types.hpp>
#include <vtr_path_planning/base_path_planner.hpp>
#include <vtr_common/utils/macros.hpp>
#include <lgmath/se3/Transformation.hpp>

#ifndef VTR_PATH_PLANNING_ERROR_PREDICTORS_BASE_ERROR_PREDICTOR_HPP
#define VTR_PATH_PLANNING_ERROR_PREDICTORS_BASE_ERROR_PREDICTOR_HPP

namespace vtr {
namespace path_planning {
using RobotState = tactic::OutputCache;

// Snapshot of the exact numeric tensors a predictError() call built, for
// direct offline logging -- avoids reconstructing (and potentially
// mismatching) these inputs from bag/graph data after the fact. Image
// inputs are intentionally excluded (large, not CSV-friendly); this covers
// only the small numeric/sequence inputs.
struct PredictorInputSnapshot {
  bool valid = false;
  std::array<double, 6> loc_res{};    // [x_in_path, y_in_path, z_log, rx, ry, rz]
  std::array<double, 6> odom_vel{};   // body-frame velocity twist (-w_p_r_in_r)
  std::array<double, 3> grav_vec{};   // gravity rotated into robot body frame
  std::vector<std::array<double, 6>> sequence;  // one 6-vec (poseToVec) per horizon step
};

class BaseErrorPredictor {
public:
  PTR_TYPEDEFS(BaseErrorPredictor);

  virtual ~BaseErrorPredictor() = default;

  // input_snapshot: optional output parameter. If non-null, implementations
  // that build numeric/sequence tensors (ImageErrorPredictorNetwork,
  // NumericalErrorPredictorNetwork) should fill it in with exactly the
  // values passed to the model this call, for direct offline logging.
  // Implementations that don't build these tensors (HistoryLookupErrorPredictor)
  // may ignore it -- it is left with valid=false in that case.
  virtual void predictError(const RobotState& robot_state,
                            const tactic::Timestamp& curr_time,
                            std::vector<lgmath::se3::Transformation>& reference_poses,
                            PredictorInputSnapshot* input_snapshot = nullptr) = 0;

protected:
  struct ErrorSample {
    tactic::Timestamp timestamp;
    Eigen::VectorXd error;
  };

  std::vector<ErrorSample> error_history_;
  Eigen::VectorXd gravity_;
};

}  // namespace path_planning
}  // namespace vtr

#endif  // VTR_PATH_PLANNING_ERROR_PREDICTORS_BASE_ERROR_PREDICTOR_HPP