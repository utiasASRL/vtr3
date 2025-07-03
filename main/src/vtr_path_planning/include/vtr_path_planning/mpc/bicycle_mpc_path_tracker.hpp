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
#include <vtr_path_planning/base_path_planner.hpp>
#include <vtr_path_planning/mpc/casadi_path_planners.hpp>
#include <vtr_path_planning/mpc/speed_scheduler.hpp>

#include <vtr_path_planning/cbit/visualization_utils.hpp>

namespace vtr {
namespace path_planning {

class BicycleMPCPathTracker : public BasePathPlanner {
 public:
  PTR_TYPEDEFS(BicycleMPCPathTracker);

  static constexpr auto static_name = "bicycle_mpc";

  // Note all rosparams that are in the config yaml file need to be declared here first, though they can be then changes using the declareparam function for ros in the cpp file
  struct Config : public BasePathPlanner::Config {
    PTR_TYPEDEFS(Config);

    // Speed Scheduler
    double planar_curv_weight = 2.50;
    double profile_curv_weight = 0.5; 
    double eop_weight = 1.0;
    double min_vel = 0.5;  

    // MPC Configs
    bool extrapolate_robot_pose = true;
    bool mpc_verbosity = false;
    double forward_vel = 0.75;
    double max_lin_vel = 1.25;
    double max_ang_vel = 0.75;
    double max_lin_acc = 10.0;
    double max_ang_acc = 10.0;
    double robot_linear_velocity_scale = 1.0;
    double robot_angular_velocity_scale = 1.0;
    double turning_radius = 1.0;
    double wheelbase = 0.5;

    // MPC Costs
    double q_x = 0.0;
    double q_y = 0.0;
    double q_th = 0.0;
    double r1 = 0.0;
    double r2 = 0.0;
    double racc2 = 0.0;
    double racc1 = 0.0;
    double q_f = 0.0;

    // Misc
    int command_history_length = 100;


    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning");
  };

  BicycleMPCPathTracker(const Config::ConstPtr& config,
                 const RobotState::Ptr& robot_state,
                 const tactic::GraphBase::Ptr& graph,
                 const Callback::Ptr& callback);
  ~BicycleMPCPathTracker() override;

 protected:
  void initializeRoute(RobotState& robot_state);
  Command computeCommand(RobotState& robot_state) override;

 private: 
  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(BicycleMPCPathTracker);

  Config::ConstPtr config_;
  CasadiBicycleMPC solver_;

  // Store the previously applied velocity and a sliding window history of MPC results
  Eigen::Vector2d applied_vel_;
  std::vector<Eigen::Vector2d> vel_history;
  tactic::Timestamp prev_vel_stamp_;
  RobotState::Ptr robot_state_;
  Command computeCommand_(RobotState& robot_state);

  VisualizationUtils::Ptr vis_;  


};

}  // namespace path_planning
}  // namespace vtr