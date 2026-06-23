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

#include <vtr_logging/logging.hpp>
#include <vtr_tactic/types.hpp>
#include <vtr_path_planning/base_path_planner.hpp>
#include <vtr_common/utils/macros.hpp>

#ifndef VTR_PATH_PLANNING_ERROR_PREDICTORS_BASE_ERROR_PREDICTOR_HPP
#define VTR_PATH_PLANNING_ERROR_PREDICTORS_BASE_ERROR_PREDICTOR_HPP

namespace vtr {
namespace path_planning {
using RobotState = tactic::OutputCache;

class BaseErrorPredictor {
public:
  PTR_TYPEDEFS(BaseErrorPredictor);

  virtual ~BaseErrorPredictor() = default;

  virtual void predictError(const RobotState& robot_state, const tactic::Timestamp& curr_time) = 0;

  virtual void compute_gravity_vector(const RobotState& /* robot_state */) {}

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