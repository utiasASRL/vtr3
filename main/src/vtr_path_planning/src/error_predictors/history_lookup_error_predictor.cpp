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
 * \file history_lookup_error_predictor.cpp
 * \author Luka Antonyshyn, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/error_predictors/history_lookup_error_predictor.hpp"

namespace vtr::path_planning {

HistoryLookupErrorPredictor::HistoryLookupErrorPredictor() = default;
HistoryLookupErrorPredictor::~HistoryLookupErrorPredictor() = default;

void HistoryLookupErrorPredictor::predictError(
    const RobotState& /*robot_state*/,
    const tactic::Timestamp& /*curr_time*/,
    std::vector<lgmath::se3::Transformation>& /*reference_poses*/) {
  // TODO: implement history-based lookup correction
}

}  // namespace vtr::path_planning
