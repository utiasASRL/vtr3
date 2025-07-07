// Copyright 2025, Autonomous Space Robotics Lab (ASRL)
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
 * \file unicycle_mpc_path_tracker.hpp
 * \author Alec Krawciw, Luka Antonyshyn Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_logging/logging.hpp>
#include <vtr_tactic/types.hpp>
#include <vtr_path_planning/mpc/base_mpc_path_tracker.hpp>
#include <vtr_path_planning/mpc/casadi_path_planners.hpp>
#include <vtr_path_planning/mpc/speed_scheduler.hpp>

#include <vtr_path_planning/cbit/visualization_utils.hpp>

namespace vtr {
namespace path_planning {

class BicycleMPCPathTracker : public BaseMPCPathTracker {
 public:
  PTR_TYPEDEFS(BicycleMPCPathTracker);

  static constexpr auto static_name = "bicycle_mpc";

  // Note all rosparams that are in the config yaml file need to be declared here first, though they can be then changes using the declareparam function for ros in the cpp file
  struct Config : public BaseMPCPathTracker::Config {
    PTR_TYPEDEFS(Config);

    // MPC Costs
    double q_x = 0.0;
    double q_y = 0.0;
    double q_th = 0.0;
    double r1 = 0.0;
    double r2 = 0.0;
    double racc2 = 0.0;
    double racc1 = 0.0;
    double q_f = 0.0;

    bool repeat_flipped = false;

    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning");
  };

  BicycleMPCPathTracker(const Config::ConstPtr& config,
                 const RobotState::Ptr& robot_state,
                 const tactic::GraphBase::Ptr& graph,
                 const Callback::Ptr& callback);

  ~BicycleMPCPathTracker() override;

 protected:
  virtual CasadiMPC::Config::Ptr loadMPCConfig(
      const bool isReversing,   Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel) override;

  virtual casadi::DMDict callSolver(CasadiMPC::Config::Ptr config) override;

 private: 
  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(BicycleMPCPathTracker);

  Config::ConstPtr config_;
  CasadiBicycleMPC solver_;
};

}  // namespace path_planning
}  // namespace vtr