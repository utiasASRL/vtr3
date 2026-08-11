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
 * \file feedback_linearization_path_tracker.hpp
 * \author Luka Antonyshyn, Autonomous Space Robotics Lab (ASRL)
 *
 * Exact feedback linearization controller for bicycle kinematics, mainly built as a base
 * for ILC baseline, from Ostafew 2013
 */
#pragma once

#include <vtr_logging/logging.hpp>
#include <vtr_tactic/types.hpp>
#include <vtr_path_planning/base_path_planner.hpp>
#include <vtr_path_planning/cbit/utils.hpp>

namespace vtr {
namespace path_planning {

class FeedbackLinearizationPathTracker : public BasePathPlanner {
 public:
  PTR_TYPEDEFS(FeedbackLinearizationPathTracker);

  static constexpr auto static_name = "feedback_linearization";

  struct Config : public BasePathPlanner::Config {
    PTR_TYPEDEFS(Config);

    // Vehicle
    double wheelbase = 0.55;

    // Desired forward speed along the path (m/s). Sign flips automatically
    // when the localization chain indicates a reverse segment.
    double forward_vel = 0.75;

    // Speed scheduler config vals 
    double planar_curv_weight = 2.50;
    double profile_curv_weight = 0.5;
    double eop_weight = 1.0;
    double min_vel = 0.5;
    double eop_min_vel = 0.5;

    // Output limits
    double max_lin_vel = 1.25;
    double max_steering_angle = 0.6;  // [rad]

    // Feedback gains on the linearized (Delta_L, z_2 = v*sin(Delta_H))
    // error integrator dynamics
    //   gamma1 > 0   and   0 < gamma2 - dt*gamma1 < 2/dt
    double gamma1 = 1.0;
    double gamma2 = 2.0;

    // Below this speed the linearization is ill-conditioned 
    // So we set a separate minimum
    double min_linearizing_speed = 0.05;
    double creep_vel = 0.15;

    // Calibration scale factors applied to the final command.
    double robot_linear_velocity_scale = 1.0;
    double robot_angular_velocity_scale = 1.0;

    bool extrapolate_robot_pose = true;

    static void loadConfig(Config::Ptr config,
                            const rclcpp::Node::SharedPtr& node,
                            const std::string& prefix = "path_planning");

    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                        const std::string& prefix = "path_planning");
  };

  FeedbackLinearizationPathTracker(const Config::ConstPtr& config,
                                    const RobotState::Ptr& robot_state,
                                    const tactic::GraphBase::Ptr& graph,
                                    const Callback::Ptr& callback);
  ~FeedbackLinearizationPathTracker() override;

 protected:
  Command computeCommand(RobotState& robot_state) override;

  // Feedforward correction for the reference trajectory
  virtual float feedforwardCorrection(
      const tactic::VertexId& /* teach_vid */, const double /* x_ref */,
      const double /* y_ref */) {
    return 0.0f;
  }

 private:
  const Config::ConstPtr config_;

  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(FeedbackLinearizationPathTracker);
};

}  // namespace path_planning
}  // namespace vtr
