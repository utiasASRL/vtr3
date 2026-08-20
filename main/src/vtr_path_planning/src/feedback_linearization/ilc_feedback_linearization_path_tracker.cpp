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
 * \file ilc_feedback_linearization_path_tracker.cpp
 * \author Luka Antonyshyn, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_path_planning/feedback_linearization/ilc_feedback_linearization_path_tracker.hpp"
#include <vtr_path_planning/mpc/mpc_common.hpp>
#include <vtr_path_planning/mpc/speed_scheduler.hpp>

namespace vtr::path_planning {

namespace {

double clampAbs(double val, double limit) {
  return std::clamp(val, -limit, limit);
}

double wrapToPi(double angle) {
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle <= 0.0) angle += 2.0 * M_PI;
  return angle - M_PI;
}

}  // namespace

auto ILCFeedbackLinearizationPathTracker::Config::loadConfig(
    Config::Ptr config, const rclcpp::Node::SharedPtr& node,
    const std::string& prefix) -> void {
  config->learning_gain_lateral = node->declare_parameter<double>(
      prefix + ".ilc.learning_gain_lateral", config->learning_gain_lateral);
  config->learning_gain_heading = node->declare_parameter<double>(
      prefix + ".ilc.learning_gain_heading", config->learning_gain_heading);
  config->forgetting_factor = node->declare_parameter<double>(
      prefix + ".ilc.forgetting_factor", config->forgetting_factor);
  config->feedforward_max = node->declare_parameter<double>(
      prefix + ".ilc.feedforward_max", config->feedforward_max);
  config->lookahead_window = node->declare_parameter<int>(
      prefix + ".ilc.lookahead_window", config->lookahead_window);
}

auto ILCFeedbackLinearizationPathTracker::Config::fromROS(
    const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();
  auto base_config =
      std::static_pointer_cast<FeedbackLinearizationPathTracker::Config>(
          config);
  *base_config = *FeedbackLinearizationPathTracker::Config::fromROS(node, prefix);
  loadConfig(config, node, prefix);
  return config;
}

ILCFeedbackLinearizationPathTracker::ILCFeedbackLinearizationPathTracker(
    const Config::ConstPtr& config, const RobotState::Ptr& robot_state,
    const tactic::GraphBase::Ptr& graph, const Callback::Ptr& callback)
    : FeedbackLinearizationPathTracker(config, robot_state, graph, callback),
      config_(config),
      current_accum_vid_(0) {}

ILCFeedbackLinearizationPathTracker::~ILCFeedbackLinearizationPathTracker() {
  stop();
}

void ILCFeedbackLinearizationPathTracker::setRunning(const bool running) {
  CLOG(DEBUG, "feedback_linearization.ilc") << "Setting running state to " << running;
  if (!running && lateral_error_mean_.size() > 0) {
    updateBookkeeping();
    updateFeedForwardCorrections();
    // Clear these - set running is set to false twice for some reason?
    // TODO: dig into this later maybe
    lateral_error_mean_.clear();
    heading_error_mean_.clear();
    linear_velocity_mean_.clear();
    CLOG(DEBUG, "feedback_linearization.ilc") << "Finalized bookkeeping and feedforward corrections";
  }
  else{
    CLOG(DEBUG, "feedback_linearization.ilc") << "Not updating bookkeeping or feedforward corrections";
  }
  BasePathPlanner::setRunning(running);
}

std::pair<double, double> ILCFeedbackLinearizationPathTracker::calculateErrors(
    const unsigned sid, RobotState& robot_state) {

  auto& chain = robot_state.chain.ptr();
  const auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] =
      getChainInfo(*chain);

  const auto T_w_r = T_w_p * T_p_r;

  const auto segment = findClosestSegment(T_w_r, chain, chain->trunkSequenceId());
  const auto& seq_start = chain->pose(segment.start_sid);
  const auto& seq_end = chain->pose(segment.end_sid);

  double interp = 0.0;
  const auto T_w_ref = interpolatePath(T_w_r, seq_start, seq_end, interp);
  interp = std::clamp(interp, 0.0, 1.0);

  // Rear-axle state: world-frame position and heading.
  const double x = T_w_r.r_ab_inb()(0);
  const double y = T_w_r.r_ab_inb()(1);
  const double theta = lgmath::so3::rot2vec(T_w_r.C_ba())(2);

  const double x_ref = T_w_ref.r_ab_inb()(0);
  const double y_ref = T_w_ref.r_ab_inb()(1);
  const double theta_ref = lgmath::so3::rot2vec(T_w_ref.C_ba())(2);

  // Signed lateral (cross-track) error assuming locally linear-ish
  const double dx = x - x_ref;
  const double dy = y - y_ref;
  const double lateral_error = -std::sin(theta_ref) * dx + std::cos(theta_ref) * dy;

  // Heading error, wrapped to (-pi, pi]
  const double heading_error = wrapToPi(theta - theta_ref);

  return {lateral_error, heading_error};
}

auto ILCFeedbackLinearizationPathTracker::computeCommand(RobotState& robot_state)
    -> Command {
  // Get parent command, then offset
  auto base_command = FeedbackLinearizationPathTracker::computeCommand(robot_state);
  auto v_k_i = base_command.linear.x;
  auto psi_k_i_fb = base_command.angular.z;
  //Handle bookkeeping
  auto& chain = robot_state.chain.ptr();
  const auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] =
      getChainInfo(*chain);

  auto [lateral_error, heading_error] = calculateErrors(curr_sid, robot_state);

  if (curr_sid != current_accum_vid_) {
    CLOG(DEBUG, "feedback_linearization.ilc")
        << "Advancing accumulation vertex from " << current_accum_vid_
        << " to " << curr_sid << ", finalizing bookkeeping for "
        << lateral_error_mean_.size() << " vertices seen so far";
    updateBookkeeping();
    current_accum_vid_ = curr_sid;
  }

  // Temporary hack to see if the vehicle is active
  // Don't want to bias errors with dead-man switched items
  double psi_k_i_ff = 0.0d;
  if (v_k_i >= 0.1) {
    lateral_error_samples_.push_back(lateral_error);
    heading_error_samples_.push_back(heading_error);
    linear_velocity_samples_.push_back(w_p_r_in_r.head(3).norm());
    psi_k_i_ff = feedforwardCorrection(curr_sid);
  } else {
    CLOG(DEBUG, "feedback_linearization.ilc")
        << "Skipping ILC sample accumulation at sid " << curr_sid
        << ", v_k_i = " << v_k_i << " below activity threshold (0.1)";
  }

  CLOG(DEBUG, "feedback_linearization.ilc")
      << "sid: " << curr_sid << " | lateral_error: " << lateral_error
      << " heading_error: " << heading_error
      << " | psi_fb: " << psi_k_i_fb << " psi_ff: " << psi_k_i_ff
      << " | psi_total: " << (psi_k_i_fb + psi_k_i_ff)
      << " v: " << v_k_i;

  base_command.angular.z += psi_k_i_ff;
  return base_command;
}


double ILCFeedbackLinearizationPathTracker::feedforwardCorrection(
      const unsigned sid){
    if (psi_ff_correction_.find(sid) != psi_ff_correction_.end()) {
      return psi_ff_correction_.at(sid);
    }
    return 0.0;
}

void ILCFeedbackLinearizationPathTracker::updateBookkeeping(){
  if (lateral_error_samples_.empty()) {
    CLOG(DEBUG, "feedback_linearization.ilc")
        << "updateBookkeeping called for vertex " << current_accum_vid_
        << " with no samples accumulated, skipping";
    return;
  }

  // Update cached feedforward corrections
  double lateral_error_accum = std::accumulate(lateral_error_samples_.begin(), lateral_error_samples_.end(), 0.0);
  double lat_err_mean = lateral_error_accum / lateral_error_samples_.size();
  lateral_error_mean_[current_accum_vid_] = lat_err_mean;
  lateral_error_samples_.clear();

  double heading_error_accum = std::accumulate(heading_error_samples_.begin(), heading_error_samples_.end(), 0.0);
  double heading_err_mean = heading_error_accum / heading_error_samples_.size();
  heading_error_mean_[current_accum_vid_] = heading_err_mean;
  heading_error_samples_.clear();

  double linear_velocity_accum = std::accumulate(linear_velocity_samples_.begin(), linear_velocity_samples_.end(), 0.0);
  double lin_vel_mean = linear_velocity_accum / linear_velocity_samples_.size();
  linear_velocity_mean_[current_accum_vid_] = lin_vel_mean;
  linear_velocity_samples_.clear();

  CLOG(DEBUG, "feedback_linearization.ilc")
      << "Vertex " << current_accum_vid_ << " bookkeeping: mean lateral_error="
      << lat_err_mean << " mean heading_error=" << heading_err_mean
      << " mean v=" << lin_vel_mean;

  return;
}

void ILCFeedbackLinearizationPathTracker::updateFeedForwardCorrections(){
  unsigned num_verts = lateral_error_mean_.size();
  auto config_lookahead_window = config_->lookahead_window;
  CLOG(DEBUG, "feedback_linearization.ilc")
      << "updateFeedForwardCorrections: " << num_verts
      << " vertices with error data, lookahead_window="
      << config_lookahead_window;
  unsigned num_updated = 0;
  // Iterate only over SIDs that actually have accumulated error data
  for (const auto& [i, lat_err_i] : lateral_error_mean_) {
    // Average over next N SIDs, following Ostafew
    double avg_err_lat = 0.0, avg_err_yaw = 0.0;
    unsigned lookahead_window = 0;
    for (unsigned j = i + 2; j < i + 2 + config_lookahead_window; ++j) {
      const auto lat_it = lateral_error_mean_.find(j);
      const auto yaw_it = heading_error_mean_.find(j);
      if (lat_it == lateral_error_mean_.end() ||
          yaw_it == heading_error_mean_.end())
        continue;
      avg_err_lat += lat_it->second;
      avg_err_yaw += yaw_it->second;
      ++lookahead_window;
    }
    if (lookahead_window == 0) {
      CLOG(DEBUG, "feedback_linearization.ilc")
          << "Vertex " << i << " has no lookahead window, skipping "
          << "feedforward update";
      continue;
    }
    const auto lin_vel_it = linear_velocity_mean_.find(i);
    if (lin_vel_it == linear_velocity_mean_.end()) {
      CLOG(DEBUG, "feedback_linearization.ilc")
          << "Vertex " << i << " has no linear velocity data, skipping "
          << "feedforward update";
      continue;
    }
    const double lin_vel_i = lin_vel_it->second;

    avg_err_lat /= lookahead_window;
    avg_err_yaw /= lookahead_window;

    // Update the feedforward correction
    auto last_iter_ff_psi = config_->forgetting_factor * (psi_ff_correction_.find(i) != psi_ff_correction_.end() ? psi_ff_correction_.at(i) : 0.0);
    auto num = -config_->wheelbase * (config_->learning_gain_lateral * avg_err_lat + \
      config_->learning_gain_heading * lin_vel_i * sin(avg_err_yaw));
    auto denom = lin_vel_i * lin_vel_i * cos(avg_err_yaw);

    auto psi_curr_iter_ff = last_iter_ff_psi + std::atan2(num, denom);
    CLOG(DEBUG, "feedback_linearization.ilc") << "avg_err_lat: " << avg_err_lat
        << " avg_err_yaw: " << avg_err_yaw
        << " lin_vel_i: " << lin_vel_i
        << " psi_curr_iter_ff: " << psi_curr_iter_ff;
    CLOG(DEBUG, "feedback_linearization.ilc") << "Numerator: " << num << " Denominator: " << denom;

    const double clamped = clampAbs(psi_curr_iter_ff, config_->feedforward_max);
    if (std::abs(clamped - psi_curr_iter_ff) > 1e-9) {
      CLOG(DEBUG, "feedback_linearization.ilc")
          << "Vertex " << i << " ff correction clamped: raw="
          << psi_curr_iter_ff << " limit=" << config_->feedforward_max;
    }
    psi_ff_correction_[i] = clamped;
    ++num_updated;
  }
  CLOG(DEBUG, "feedback_linearization.ilc")
      << "updateFeedForwardCorrections: updated " << num_updated << "/"
      << num_verts << " vertex corrections";
}

}  // namespace vtr::path_planning
