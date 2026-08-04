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
#include "vtr_path_planning/error_predictors/correction_utils.hpp"
#include "vtr_path_planning/mpc/mpc_common.hpp"

#include <vtr_storage/stream/message.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

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

// A flat (dx, dy, dyaw) correction/error, in the teach-path/trunk frame
// Used to store past corrections for accumulator
struct Correction {
  double dx = 0.0;
  double dy = 0.0;
  double dyaw = 0.0;

  static Correction fromTransform(const lgmath::se3::Transformation& T) {
    return {T.r_ba_ina()(0), T.r_ba_ina()(1), yawOf(T)};
  }

  Correction operator+(const Correction& o) const {
    return {dx + o.dx, dy + o.dy, dyaw + o.dyaw};
  }
};

Correction interpolate(const Correction& a, const Correction& b, double interp) {
  return {a.dx + interp * (b.dx - a.dx), a.dy + interp * (b.dy - a.dy),
          a.dyaw + interp * (b.dyaw - a.dyaw)};
}

// Recover TF from correction type above
lgmath::se3::Transformation toTransform(const Correction& c) {
  Eigen::Matrix3d C_ba = Eigen::Matrix3d::Identity();
  C_ba(0, 0) = std::cos(c.dyaw);  C_ba(0, 1) = -std::sin(c.dyaw);
  C_ba(1, 0) = std::sin(c.dyaw);  C_ba(1, 1) = std::cos(c.dyaw);
  const Eigen::Vector3d r_ba_ina(c.dx, c.dy, 0.0);
  return lgmath::se3::Transformation(C_ba, r_ba_ina);
}
}  // namespace

HistoryLookupErrorPredictor::HistoryLookupErrorPredictor(HistoryLookupMode mode,
                                                       const tactic::GraphBase::Ptr& graph,
                                                       bool invert_correction,
                                                       bool lateral_only_correction)
    : mode_(mode),
      graph_(graph),
      invert_correction_(invert_correction),
      lateral_only_correction_(lateral_only_correction) {}

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
    const auto T = edge->T();
    return lgmath::se3::Transformation(invert_correction_ ? T.inverse() : T);
  }
  return std::nullopt;
}

std::optional<std::pair<uint32_t, lgmath::se3::Transformation>>
HistoryLookupErrorPredictor::lookupResidualBeforeRun(
    const tactic::VertexId& teach_vid, std::optional<uint32_t> before_run) const {
  if (!graph_ || !graph_->contains(teach_vid)) return std::nullopt;

  const auto neighbours = graph_->neighbors(teach_vid);
  for (auto it = neighbours.rbegin(); it != neighbours.rend(); ++it) {
    if (before_run.has_value() && it->majorId() >= *before_run) continue;
    const tactic::EdgeId edge_id(teach_vid, *it);
    if (!graph_->contains(edge_id)) continue;
    const auto edge = graph_->at(edge_id);
    if (!edge->isSpatial()) continue;
    const auto T = edge->T();
    return std::make_pair(it->majorId(),
                          lgmath::se3::Transformation(invert_correction_ ? T.inverse() : T));
  }
  return std::nullopt;
}

std::optional<lgmath::se3::Transformation>
HistoryLookupErrorPredictor::lookupAccumulatedError(
    const tactic::VertexId& teach_vid, const tactic::Timestamp& write_time) {
  if (!graph_ || !graph_->contains(teach_vid)) return std::nullopt;

  using AccumMsg = vtr_path_planning_msgs::msg::AccumulatedTrackingError;
  const auto vertex = graph_->at(teach_vid);
  const auto records = vertex->retrieve<AccumMsg>(
      kAccumStreamName, kAccumStreamType,
      std::numeric_limits<storage::Timestamp>::min(),
      std::numeric_limits<storage::Timestamp>::max());

  // Latest stored running total (if any)
  std::optional<uint32_t> total_run;
  Correction total;
  if (!records.empty()) {
    // Get last error for accumulation
    const auto& msg = records.back()->locked().get().getData();
    total_run = msg.run_id;
    total = {msg.dx, msg.dy, msg.dyaw};
  }

  // Lazily catch up: fold in the most recent repeat's residual that hasn't accumulated
  const auto newer = lookupResidualBeforeRun(teach_vid, std::nullopt);
  if (newer.has_value() &&
      (!total_run.has_value() || newer->first > *total_run)) {
    const auto& [run_id, residual_tf] = *newer;
    const Correction residual = Correction::fromTransform(residual_tf);
    const Correction new_total = total_run.has_value() ? (total + residual) : residual;

    auto msg = std::make_shared<AccumMsg>();
    msg->run_id = run_id;
    msg->timestamp = static_cast<int64_t>(write_time);
    msg->dx = new_total.dx;
    msg->dy = new_total.dy;
    msg->dyaw = new_total.dyaw;

    auto locked_msg = std::make_shared<storage::LockableMessage<AccumMsg>>(
        msg, static_cast<storage::Timestamp>(write_time));

    vertex->insert<AccumMsg>(kAccumStreamName, kAccumStreamType, locked_msg);

    total = new_total;
    total_run = run_id;
  }

  if (!total_run.has_value()) return std::nullopt;
  return toTransform(total);
}

std::vector<std::array<double, 3>> HistoryLookupErrorPredictor::predictError(
    const RobotState& robot_state,
    const tactic::Timestamp& curr_time,
    std::vector<lgmath::se3::Transformation>& reference_poses,
    const lgmath::se3::Transformation& T_p_r_extp,
    bool apply_correction,
    PredictorInputSnapshot* /*input_snapshot*/) {
  std::vector<std::array<double, 3>> corrections(reference_poses.size(), {0.0, 0.0, 0.0});
  if (reference_poses.empty() || !graph_) return corrections;

  auto& chain = robot_state.chain.ptr();
  const auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo,
              curr_sid] = getChainInfo(*chain);

  const bool accumulating = (mode_ == HistoryLookupMode::Accumulating);

  unsigned last_sid = curr_sid;
  for (size_t i = 0; i < reference_poses.size(); ++i) {
    const lgmath::se3::Transformation T_wr = T_w_p * reference_poses[i];
    const auto segment = findClosestSegment(T_wr, chain, last_sid);
    last_sid = segment.start_sid;

    double interp = 0.0;
    interpolatePath(T_wr, chain->pose(segment.start_sid),
                    chain->pose(segment.end_sid), interp);
    interp = std::clamp(interp, 0.0, 1.0);

    const tactic::VertexId start_vid = chain->sequence()[segment.start_sid];
    const tactic::VertexId end_vid = chain->sequence()[segment.end_sid];

    const auto err_start = accumulating ? lookupAccumulatedError(start_vid, curr_time)
                                        : lookupLastRepeatError(start_vid);
    const auto err_end = accumulating ? lookupAccumulatedError(end_vid, curr_time)
                                      : lookupLastRepeatError(end_vid);

    std::optional<Correction> err;
    if (err_start.has_value() && err_end.has_value()) {
      err = interpolate(Correction::fromTransform(*err_start),
                        Correction::fromTransform(*err_end), interp);
    } else if (err_start.has_value()) {
      err = Correction::fromTransform(*err_start);
    } else if (err_end.has_value()) {
      err = Correction::fromTransform(*err_end);
    }
    if (!err.has_value()) continue;  // cold start: no prior repeat here yet

    corrections[i] = {err->dx, err->dy, err->dyaw};
    if (apply_correction) {
      reference_poses[i] = lateral_only_correction_
          ? applyLateralPoseCorrection(reference_poses[i], err->dx, err->dy, err->dyaw)
          : applyPoseCorrection(reference_poses[i], err->dx, err->dy, err->dyaw);
    }
  }
  return corrections;
}

}  // namespace vtr::path_planning
