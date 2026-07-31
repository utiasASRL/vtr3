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
 * \author Jordy Sehn, Alec Krawciw Autonomous Space Robotics Lab (ASRL)
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

class UnicycleMPCPathTracker : public BaseMPCPathTracker {
 public:
  PTR_TYPEDEFS(UnicycleMPCPathTracker);

  static constexpr auto static_name = "unicycle_mpc";

  // Note all rosparams that are in the config yaml file need to be declared here first, though they can be then changes using the declareparam function for ros in the cpp file
  struct Config : public BaseMPCPathTracker::Config {
    PTR_TYPEDEFS(Config);

    // Add unicycle model param
    double q_x = 0.0;
    double q_y = 0.0;
    double q_th = 0.0;
    double r1 = 0.0;
    double r2 = 0.0;
    double racc1 = 0.0;
    double racc2 = 0.0;

    static void loadConfig(Config::Ptr config,  
		           const rclcpp::Node::SharedPtr& node,
                           const std::string& prefix = "path_planning");

    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning");
  };

  UnicycleMPCPathTracker(const Config::ConstPtr& config,
                 const RobotState::Ptr& robot_state,
                 const tactic::GraphBase::Ptr& graph,
                 const Callback::Ptr& callback);
  ~UnicycleMPCPathTracker() override;

 protected:
 
  void loadMPCConfig(
      CasadiUnicycleMPC::Config::Ptr mpc_config, Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel);
  virtual CasadiMPC::Config::Ptr getMPCConfig(Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel, const bool /*isReversing*/) override;

  virtual bool isMPCStateValid(CasadiMPC::Config::Ptr mpcConfig, const tactic::Timestamp& curr_time) override;
  
  void loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                            const lgmath::se3::Transformation& T_p_r_extp,
                            const double state_p,
                            RobotState& robot_state, 
                            const tactic::Timestamp& curr_time) override;

  virtual std::map<std::string, casadi::DM> callSolver(CasadiMPC::Config::Ptr config) override;

 private: 
  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(UnicycleMPCPathTracker);

  Config::ConstPtr config_;
  CasadiUnicycleMPC solver_;

};

}  // namespace path_planning
}  // namespace vtr