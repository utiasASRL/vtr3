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
 * \file ilc_feedback_linearization_path_tracker.hpp
 * \author Luka Antonyshyn, Autonomous Space Robotics Lab (ASRL)
 *
 
 */
#pragma once

#include <vtr_path_planning/feedback_linearization/feedback_linearization_path_tracker.hpp>

#include <unordered_map>

namespace vtr {
namespace path_planning {

/*
 * Iterative learning controller (ILC) built on top of
 * FeedbackLinearizationPathTracker. Currently only works on consecutive repeats,
 * adding memory is a todo. 
 * Follows Ostafew's work nearly exactly, aside from the mapping through nu, the exact feedback
 * linearization, as that varies between the bicycle and MPC
 * */
class ILCFeedbackLinearizationPathTracker
    : public FeedbackLinearizationPathTracker {
 public:
  PTR_TYPEDEFS(ILCFeedbackLinearizationPathTracker);

  static constexpr auto static_name = "ilc_feedback_linearization";

  struct Config : public FeedbackLinearizationPathTracker::Config {
    PTR_TYPEDEFS(Config);

    // Learning gain applied to the update law (fraction of this trial's
    // observed error folded into next trial's feedforward correction)
    double learning_gain_lateral = 0.0;
    double learning_gain_heading = 0.0;

    // Forgetting factor on the stored correction each trial, in [0, 1]
    // 1.0 = no forgetting (pure integration across repeats)
    double forgetting_factor = 1.0;

    // Max radians for the ff steering angle correction
    double feedforward_max = 0.2;

    // Number of vertices to average over into the future
    // for the heading and lateral errors
    unsigned lookahead_window = 2;

    static void loadConfig(Config::Ptr config,
                            const rclcpp::Node::SharedPtr& node,
                            const std::string& prefix = "path_planning");

    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                        const std::string& prefix = "path_planning");
  };

  ILCFeedbackLinearizationPathTracker(const Config::ConstPtr& config,
                                       const RobotState::Ptr& robot_state,
                                       const tactic::GraphBase::Ptr& graph,
                                       const Callback::Ptr& callback);
  ~ILCFeedbackLinearizationPathTracker() override;
  void setRunning(const bool running) override;

 protected:
  Command computeCommand(RobotState& robot_state) override;


  // Feedforward correction for the reference trajectory
  // Purely affects the steering angle psi, as we assume an exogenous linear velocity
  double feedforwardCorrection(
      const unsigned sid);

 private:
  const Config::ConstPtr config_;

  // Bookkeeping for per-vertex error terms
  // One per teach vertex, averaged over N localization vertices per
  // teach vertex for Repeat K
  std::vector<double> lateral_error_samples_;
  std::unordered_map<unsigned, double> lateral_error_mean_;

  std::vector<double> heading_error_samples_;
  std::unordered_map<unsigned, double> heading_error_mean_;

  // Bookkeeping for per-vertex lin vel
  std::vector<double> linear_velocity_samples_;
  std::unordered_map<unsigned, double> linear_velocity_mean_;

  // Bookkeeping for per-vertex feedforward terms
  std::unordered_map<unsigned, double> psi_feedback_mean_;
  std::vector<double> psi_feedback_samples_;


  // Keeping track of ff corrections
  std::unordered_map<unsigned, double> psi_ff_correction_;
  // Two validity check - existence of a ff, and whether it has been visited this repeat (should be updated)
  std::unordered_map<unsigned, bool> psi_ff_exist_;
  std::unordered_map<unsigned, bool> psi_ff_updated_;

  // Update feedforward terms at the end of a repeat
  void updateFeedForwardCorrections();
  void updateBookkeeping();

  std::pair<double, double> calculateErrors(
      const unsigned sid, RobotState& robot_state);


  
  unsigned current_accum_vid_;

  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(ILCFeedbackLinearizationPathTracker);
};

}  // namespace path_planning
}  // namespace vtr
