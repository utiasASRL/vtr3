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
    double f_q_lat = 0.0;
    double f_q_lon = 0.0;
    double f_q_th = 0.0;
    double f_r1 = 0.0;
    double f_r2 = 0.0;
    double f_racc2 = 0.0;
    double f_racc1 = 0.0;
    double f_q_f = 0.0;

    // MPC Costs
    double r_q_lat = 0.0;
    double r_q_lon = 0.0;
    double r_q_th = 0.0;
    double r_r1 = 0.0;
    double r_r2 = 0.0;
    double r_racc2 = 0.0;
    double r_racc1 = 0.0;
    double r_q_f = 0.0;

    double alpha = 0.6;

    int failure_threshold = 5;
    int recovery_steps = 15;

    double wheelbase = 0.55;

    static void loadConfig(Config::Ptr config,  
		           const rclcpp::Node::SharedPtr& node,
                           const std::string& prefix = "path_planning");

    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning");
  };

  BicycleMPCPathTracker(const Config::ConstPtr& config,
                 const RobotState::Ptr& robot_state,
                 const tactic::GraphBase::Ptr& graph,
                 const Callback::Ptr& callback);

  ~BicycleMPCPathTracker() override;

 protected:
  virtual std::map<std::string, casadi::DM> callSolver(CasadiMPC::Config::Ptr config) override;
  void loadMPCConfig(
      CasadiBicycleMPC::Config::Ptr mpc_config, const bool isReversing,   Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel);
  virtual bool isMPCStateValid(CasadiMPC::Config::Ptr mpcConfig, const tactic::Timestamp& curr_time) override;
  virtual CasadiMPC::Config::Ptr getMPCConfig(
      const bool isReversing,   Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel) override;
  void loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                            const lgmath::se3::Transformation& T_p_r_extp,
                           const double state_p,
                           RobotState& robot_state, 
                           const tactic::Timestamp& curr_time) override;

 private: 
  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(BicycleMPCPathTracker);

  int failure_count = 0;
  int success_count = 0;
  Config::ConstPtr config_;
  CasadiBicycleMPC solver_;
};

}  // namespace path_planning
}  // namespace vtr
