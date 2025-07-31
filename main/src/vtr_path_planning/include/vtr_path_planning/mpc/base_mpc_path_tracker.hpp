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
 * \file base_mpc_path_tracker.hpp
 * \author Luka Antonyshyn Autonomous Space Robotics Lab (ASRL)
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

class BaseMPCPathTracker : public BasePathPlanner {
 public:
  PTR_TYPEDEFS(BaseMPCPathTracker);

  static constexpr auto static_name = "base_mpc";

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
    bool repeat_flipped = false;

    // Detection threshold to set MPC weights to zero
    double end_of_path_distance_threshold = 0.01;

    // Misc
    int command_history_length = 100;

    static void loadConfig(Config::Ptr config,  
		           const rclcpp::Node::SharedPtr& node,
                           const std::string& prefix = "path_planning");


    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning");
  };

  BaseMPCPathTracker(const Config::ConstPtr& config,
                 const RobotState::Ptr& robot_state,
                 const tactic::GraphBase::Ptr& graph,
                 const Callback::Ptr& callback);
  ~BaseMPCPathTracker() override;

 protected:
  void initializeRoute(RobotState& robot_state);
  virtual Command computeCommand(RobotState& robot_state) override;
  virtual bool isMPCStateValid(CasadiMPC::Config::Ptr mpcConfig, const tactic::Timestamp& curr_time) = 0;
  virtual CasadiMPC::Config::Ptr getMPCConfig(const bool isReversing,  Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel)=0;
  virtual std::map<std::string, casadi::DM> callSolver(CasadiMPC::Config::Ptr config) = 0;
  virtual void loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                            const lgmath::se3::Transformation& T_p_r_extp,
                           const double state_p,
                           RobotState& robot_state, 
                           const tactic::Timestamp& curr_time);


 private: 

  const Config::ConstPtr base_config_;
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
