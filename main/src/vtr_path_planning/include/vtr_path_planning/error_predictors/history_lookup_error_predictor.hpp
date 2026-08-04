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
#include "vtr_path_planning/cbit/utils.hpp"
#include <vtr_path_planning_msgs/msg/accumulated_tracking_error.hpp>
#include <optional>

namespace vtr::path_planning {
  // PreviousRepeat: apply the single most recent repeat's observed error as a correction
  // Accumulating: integral-style accumulation across repeats
  enum class HistoryLookupMode {
    PreviousRepeat,
    Accumulating,
    PathTrackingError,
    KNearestNeighbour
  };

// vtr_tactic/types.hpp
using Graph = pose_graph::RCGraph;
using GraphBase = Graph::Base;
using VertexId = pose_graph::VertexId;
using EdgeId = pose_graph::EdgeId;

class HistoryLookupErrorPredictor : public BaseErrorPredictor {
public:
  HistoryLookupErrorPredictor(HistoryLookupMode mode,
                              const tactic::GraphBase::Ptr& graph,
                              bool invert_correction = false,
                              bool lateral_only_correction = true);
  ~HistoryLookupErrorPredictor();

  std::vector<std::array<double, 3>> predictError(
      const RobotState& robot_state, const tactic::Timestamp& curr_time,
      std::vector<lgmath::se3::Transformation>& reference_poses,
      const lgmath::se3::Transformation& T_p_r_extp,
      bool apply_correction = true,
      PredictorInputSnapshot* input_snapshot = nullptr) override;

private:
  HistoryLookupMode mode_;
  tactic::GraphBase::Ptr graph_;
  bool invert_correction_;
  bool lateral_only_correction_;

  static constexpr char kAccumStreamName[] = "history_lookup_accumulated_error";
  static constexpr char kAccumStreamType[] =
      "vtr_path_planning_msgs/msg/AccumulatedTrackingError";

  std::optional<lgmath::se3::Transformation> lookupLastRepeatError(
      const tactic::VertexId& teach_vid) const;

  std::optional<std::pair<uint32_t, lgmath::se3::Transformation>>
  lookupResidualBeforeRun(const tactic::VertexId& teach_vid,
                          std::optional<uint32_t> before_run) const;

  std::optional<lgmath::se3::Transformation> lookupAccumulatedError(
      const tactic::VertexId& teach_vid, const tactic::Timestamp& write_time);
};

}  // namespace vtr::path_planning

#endif  // VTR_PATH_PLANNING_ERROR_PREDICTORS_HISTORY_LOOKUP_ERROR_PREDICTOR_HPP
