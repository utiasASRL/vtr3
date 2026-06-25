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
 * \file history_lookup_error_predictor.hpp
 * \author Luka Antonyshyn Autonomous Space Robotics Lab (ASRL)
 */

#ifndef VTR_PATH_PLANNING_ERROR_PREDICTORS_HISTORY_LOOKUP_ERROR_PREDICTOR_HPP
#define VTR_PATH_PLANNING_ERROR_PREDICTORS_HISTORY_LOOKUP_ERROR_PREDICTOR_HPP

#include "vtr_path_planning/error_predictors/base_error_predictor.hpp"
#include <lgmath/se3/Transformation.hpp>

namespace vtr::path_planning {

class HistoryLookupErrorPredictor : public BaseErrorPredictor {
public:
  HistoryLookupErrorPredictor();
  ~HistoryLookupErrorPredictor();

  void predictError(const RobotState& robot_state,
                    const tactic::Timestamp& curr_time,
                    std::vector<lgmath::se3::Transformation>& reference_poses) override;

private:
};

}  // namespace vtr::path_planning

#endif  // VTR_PATH_PLANNING_ERROR_PREDICTORS_HISTORY_LOOKUP_ERROR_PREDICTOR_HPP