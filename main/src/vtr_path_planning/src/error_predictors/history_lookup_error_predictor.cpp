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
#include "vtr_path_planning/mpc/mpc_common.hpp"

#include <algorithm>
#include <cmath>

namespace vtr::path_planning {

namespace {
double yawOf(const lgmath::se3::Transformation& T) {
  const auto M = T.matrix();
  const double r00 = M(0, 0), r11 = M(1, 1), r22 = M(2, 2);
  const double r10 = M(1, 0), r01 = M(0, 1);
  const double cos_phi = std::clamp((r00 + r11 + r22 - 1.0) / 2.0, -1.0, 1.0);
  const double phi = std::acos(cos_phi);
  const double sinc = (std::abs(phi) < 1e-8) ? 1.0 : std::sin(phi) / phi;
  return 0.5 / sinc * (r10 - r01);
}

lgmath::se3::Transformation applyPoseCorrection(
    const lgmath::se3::Transformation& pose, double dx, double dy, double dyaw) {
  Eigen::Matrix4d M = pose.matrix();
  M(0, 3) -= dx;
  M(1, 3) -= dy;
  Eigen::Matrix4d R_dyaw = Eigen::Matrix4d::Identity();
  R_dyaw(0, 0) = std::cos(-dyaw);  R_dyaw(0, 1) = -std::sin(-dyaw);
  R_dyaw(1, 0) = std::sin(-dyaw);  R_dyaw(1, 1) = std::cos(-dyaw);
  M = M * R_dyaw;
  return lgmath::se3::Transformation(M);
}
}  // namespace

HistoryLookupErrorPredictor::HistoryLookupErrorPredictor(HistoryLookupMode mode,
                                                       const tactic::GraphBase::Ptr& graph)
    : mode_(mode),
      graph_(graph) {}

HistoryLookupErrorPredictor::~HistoryLookupErrorPredictor() = default;

std::optional<lgmath::se3::Transformation>
HistoryLookupErrorPredictor::lookupLastRepeatError(
    const tactic::VertexId& teach_vid) const {
  if (!graph_ || !graph_->contains(teach_vid)) return std::nullopt;

  const auto neighbours = graph_->neighbors(teach_vid);
  for (auto it = neighbours.rbegin(); it != neighbours.rend(); ++it) {
    const tactic::EdgeId edge_id(teach_vid, *it);
    if (!graph_->contains(edge_id)) continue;
    const auto edge = graph_->at(edge_id);
    // Filter teach edges
    if (!edge->isSpatial()) continue;
    return lgmath::se3::Transformation(edge->T().inverse());
  }
  return std::nullopt;
}

void HistoryLookupErrorPredictor::predictError(
    const RobotState& robot_state,
    const tactic::Timestamp& /*curr_time*/,
    std::vector<lgmath::se3::Transformation>& reference_poses,
    PredictorInputSnapshot* /*input_snapshot*/) {
  if (reference_poses.empty() || !graph_) return;

  auto& chain = robot_state.chain.ptr();
  const auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo,
              curr_sid] = getChainInfo(*chain);

  unsigned last_sid = curr_sid;
  for (size_t i = 0; i < reference_poses.size(); ++i) {
    // Find nearest teach vertex, then use that to interpolate between two
    // nearest realized vertices from last run.
    // TODO: add switching based on defined enum modes
    const lgmath::se3::Transformation T_wr = T_w_p * reference_poses[i];
    const auto segment = findClosestSegment(T_wr, chain, last_sid);
    last_sid = segment.start_sid;

    double interp = 0.0;
    interpolatePath(T_wr, chain->pose(segment.start_sid),
                    chain->pose(segment.end_sid), interp);
    interp = std::clamp(interp, 0.0, 1.0);

    const tactic::VertexId start_vid = chain->sequence()[segment.start_sid];
    const tactic::VertexId end_vid = chain->sequence()[segment.end_sid];
    const auto err_start = lookupLastRepeatError(start_vid);
    const auto err_end = lookupLastRepeatError(end_vid);

    std::optional<lgmath::se3::Transformation> err;
    // Interpolate between the two errors to get the final error
    if (err_start.has_value() && err_end.has_value()) {
      const lgmath::se3::Transformation edge = err_start->inverse() * *err_end;
      err = *err_start * lgmath::se3::Transformation(interp * edge.vec(), 0);
    } else if (err_start.has_value()) {
      err = err_start;
    } else if (err_end.has_value()) {
      err = err_end;
    }
    if (!err.has_value()) continue;  // cold start: no prior repeat here yet

    const double dx = err->r_ba_ina()(0);
    const double dy = err->r_ba_ina()(1);
    const double dyaw = yawOf(*err);
    reference_poses[i] = applyPoseCorrection(reference_poses[i], dx, dy, dyaw);
  }
}

}  // namespace vtr::path_planning
