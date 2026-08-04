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

enum class NNTargetMode {
  Trajectory = 0,
  PathTracking = 1,
  SE2 = 2,
};

struct PredictorInputSnapshot {
  bool valid = false;
  std::array<double, 6> loc_res{};    // [x_in_path, y_in_path, z_log, rx, ry, rz]
  std::array<double, 6> odom_vel{};   // body-frame velocity twist (-w_p_r_in_r)
  std::array<double, 3> grav_vec{};   // gravity rotated into robot body frame
  std::vector<std::array<double, 6>> sequence;  // one 6-vec (poseToVec) per horizon step

  // NN model outputs, one entry per horizon step. raw_* is the model's
  std::vector<std::array<double, 3>> raw_output;
  std::vector<std::array<double, 3>> final_output;

  unsigned sid = 0;
  lgmath::se3::Transformation T_p_r;
  lgmath::se3::Transformation T_p_r_extp;
  lgmath::se3::Transformation T_w_p;
  Eigen::Matrix<double, 6, 1> w_p_r_in_r = Eigen::Matrix<double, 6, 1>::Zero();
  std::vector<lgmath::se3::Transformation> reference_poses_raw;  // pre-correction

  // Timing for T_p_r_extp: dt = extrap_wall_time - lidar_stamp for association, wall time for interpolation
  tactic::Timestamp lidar_stamp = 0;
  tactic::Timestamp extrap_wall_time = 0;
};

class BaseErrorPredictor {
public:
  PTR_TYPEDEFS(BaseErrorPredictor);

  virtual ~BaseErrorPredictor() = default;

  virtual std::vector<std::array<double, 3>> predictError(
      const RobotState& robot_state, const tactic::Timestamp& curr_time,
      std::vector<lgmath::se3::Transformation>& reference_poses,
      const lgmath::se3::Transformation& T_p_r_extp,
      bool apply_correction = true,
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