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
 * \file feedback_linearization_path_tracker.cpp
 * \author Luka Antonyshyn, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_path_planning/feedback_linearization/feedback_linearization_path_tracker.hpp"

#include <vtr_path_planning/mpc/mpc_common.hpp>

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

auto FeedbackLinearizationPathTracker::Config::loadConfig(
    Config::Ptr config, const rclcpp::Node::SharedPtr& node,
    const std::string& prefix) -> void {
  config->wheelbase = node->declare_parameter<double>(
      prefix + ".feedback_linearization.wheelbase", config->wheelbase);
  config->forward_vel = node->declare_parameter<double>(
      prefix + ".feedback_linearization.forward_vel", config->forward_vel);
  config->max_lin_vel = node->declare_parameter<double>(
      prefix + ".feedback_linearization.max_lin_vel", config->max_lin_vel);
  config->max_ang_vel = node->declare_parameter<double>(
      prefix + ".feedback_linearization.max_ang_vel", config->max_ang_vel);
  config->max_steering_angle = node->declare_parameter<double>(
      prefix + ".feedback_linearization.max_steering_angle",
      config->max_steering_angle);
  config->gamma1 = node->declare_parameter<double>(
      prefix + ".feedback_linearization.gamma1", config->gamma1);
  config->gamma2 = node->declare_parameter<double>(
      prefix + ".feedback_linearization.gamma2", config->gamma2);
  config->min_linearizing_speed = node->declare_parameter<double>(
      prefix + ".feedback_linearization.min_linearizing_speed",
      config->min_linearizing_speed);
  config->creep_vel = node->declare_parameter<double>(
      prefix + ".feedback_linearization.creep_vel", config->creep_vel);
  config->robot_linear_velocity_scale = node->declare_parameter<double>(
      prefix + ".feedback_linearization.robot_linear_velocity_scale",
      config->robot_linear_velocity_scale);
  config->robot_angular_velocity_scale = node->declare_parameter<double>(
      prefix + ".feedback_linearization.robot_angular_velocity_scale",
      config->robot_angular_velocity_scale);
  config->extrapolate_robot_pose = node->declare_parameter<bool>(
      prefix + ".feedback_linearization.extrapolate_robot_pose",
      config->extrapolate_robot_pose);
}

auto FeedbackLinearizationPathTracker::Config::fromROS(
    const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();
  auto base_config = std::static_pointer_cast<BasePathPlanner::Config>(config);
  *base_config = *BasePathPlanner::Config::fromROS(node, prefix);
  loadConfig(config, node, prefix);
  return config;
}

FeedbackLinearizationPathTracker::FeedbackLinearizationPathTracker(
    const Config::ConstPtr& config, const RobotState::Ptr& robot_state,
    const tactic::GraphBase::Ptr& graph, const Callback::Ptr& callback)
    : BasePathPlanner(config, robot_state, graph, callback), config_(config) {}

FeedbackLinearizationPathTracker::~FeedbackLinearizationPathTracker() { stop(); }

// Exact feedback linearization for steering angle commands 
// Dynamics
//[Delta L[k+1], Delta H[k+1]]^T = [Delta L[k], Delta H[k]\^T +  dt [v[k] sin \Delta H[k] v[k] / L tan psi[K]]
// Select a coordinate change to linearize the dynamics 
// z_1 [k] = \Delta L[k], z_2 [k] = v[k] sin Delta_H[k]
// Virtual Input is nu
// nu [k] = v[k] cos Delta H[k] v[k]/L tan \psi[k] [1]
// Coordinate change dynamics:
// [z_1[k+1], z_2[k+1]]^T = [1, dt \\ 0, 1] [z_1[k], z_2[k]]^T + dt [0, nu[k]]^T
// We can express nu as
// nu[k] = -gamma_1 * Delta_L[k] - gamma_2 v[k] sin Delta_H[k]
// and then use [1] to Solve for psi using
// psi [k] = atan2( (-L (gamma_1 Delta_L[k] + gamma_2 v[k] sin Delta_H[k])) / (v[k]^2 cos Delta_H[k]))

auto FeedbackLinearizationPathTracker::computeCommand(RobotState& robot_state)
    -> Command {
  auto& chain = robot_state.chain.ptr();
  if (!chain->isLocalized()) {
    CLOG(WARNING, "feedback_linearization.control")
        << "Robot is not localized, commanding the robot to stop";
    return Command();
  }

  const auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] =
      getChainInfo(*chain);

  // Extrapolate the rear-axle pose to the current time to compensate for
  // pipeline delay, exactly as the MPC trackers do.
  auto T_p_r_extp = T_p_r;
  auto curr_time = stamp;
  if (config_->extrapolate_robot_pose) {
    curr_time = robot_state.node->get_clock()->now().nanoseconds();
    const double dt = static_cast<double>(curr_time - stamp) * 1e-9;
    Eigen::Matrix<double, 6, 1> xi_p_r_in_r(-dt * w_p_r_in_r);
    T_p_r_extp = T_p_r * tactic::EdgeTransform(xi_p_r_in_r);
    CLOG(DEBUG, "feedback_linearization.control")
        << "Extrapolated robot pose from " << stamp << " to " << curr_time
        << " (dt = " << dt << " s)";
  }

  const auto T_w_r = T_w_p * T_p_r_extp;

  // Rear-axle state: world-frame position and heading.
  const double x = T_w_r.r_ab_inb()(0);
  const double y = T_w_r.r_ab_inb()(1);
  const double theta = lgmath::so3::rot2vec(T_w_r.C_ba())(2);

  // Locate the closest path segment and interpolate the reference pose and
  // tangent heading at that point
  // We use this as our error for the exact feedback linearization
  const auto segment = findClosestSegment(T_w_r, chain, chain->trunkSequenceId());
  const auto& seq_start = chain->pose(segment.start_sid);
  const auto& seq_end = chain->pose(segment.end_sid);

  double interp = 0.0;
  const auto T_w_ref = interpolatePath(T_w_r, seq_start, seq_end, interp);
  interp = std::clamp(interp, 0.0, 1.0);

  const double x_ref = T_w_ref.r_ab_inb()(0);
  const double y_ref = T_w_ref.r_ab_inb()(1);
  const double theta_ref = lgmath::so3::rot2vec(T_w_ref.C_ba())(2);

  // Signed lateral (cross-track) error assuming locally linear-ish
  const double dx = x - x_ref;
  const double dy = y - y_ref;
  const double delta_l = -std::sin(theta_ref) * dx + std::cos(theta_ref) * dy;

  // Heading error, wrapped to (-pi, pi]
  const double delta_h = wrapToPi(theta - theta_ref);

  const bool is_reverse = (segment.dir == tactic::Direction::Backward);
  const double v = is_reverse ? -config_->forward_vel : config_->forward_vel;

  double psi_cmd = 0.0;
  double v_cmd = v;
  v_cmd = clampAbs(v_cmd, config_->max_lin_vel);
  if (std::abs(v) < config_->min_linearizing_speed) {
    // The linearization is singular at v == 0
    // so fall back if we are near that velocity
    v_cmd = is_reverse ? -config_->creep_vel : config_->creep_vel;
    psi_cmd = clampAbs(std::atan2(-config_->gamma1 * delta_l, v_cmd),
                          config_->max_steering_angle);
  } else {
    // psi [k] = atan2( (-L (gamma_1 Delta_L[k] + gamma_2 v[k] sin Delta_H[k])) / (v[k]^2 cos Delta_H[k]))
    const double denom = v * v * std::cos(delta_h);
    const double numer =
        -config_->wheelbase *
        (config_->gamma1 * delta_l + config_->gamma2 * v * std::sin(delta_h));

    psi_cmd = clampAbs(std::atan2(numer, denom), config_->max_steering_angle);
  }



  Command command;
  command.linear.x = v_cmd * config_->robot_linear_velocity_scale;
  command.angular.z = psi_cmd * config_->robot_angular_velocity_scale;

  CLOG(DEBUG, "feedback_linearization.control")
      << "x: " << x << " y: " << y << " theta: " << theta
      << " | ref x: " << x_ref << " y: " << y_ref << " theta: " << theta_ref
      << " | delta_L: " << delta_l << " delta_H: " << delta_h
      << " | v: " << command.linear.x << " omega: " << command.angular.z
      << " psi_fb: " << psi_cmd;

  return command;
}

}  // namespace vtr::path_planning
