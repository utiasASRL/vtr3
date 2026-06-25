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
#include <vtr_path_planning/error_predictors/image_error_predictor_network.hpp>
#include <vtr_path_planning/error_predictors/numerical_error_predictor_network.hpp>
#include <vtr_path_planning/error_predictors/history_lookup_error_predictor.hpp>

#include <vtr_path_planning/cbit/visualization_utils.hpp>
#include <vtr_path_planning/mpc/base_mpc_path_tracker.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vtr_path_planning_msgs/msg/path_info_for_external_navigation.hpp>

namespace vtr {
namespace path_planning {

using ImageMsg = sensor_msgs::msg::Image;
using Transformation = lgmath::se3::Transformation;
using ImuMsg = sensor_msgs::msg::Imu;
using PathInfoMsg = vtr_path_planning_msgs::msg::PathInfoForExternalNavigation;

enum class ReferenceAdjustmentMode {
  None = 0,
  NumericalNeuralNetwork = 1,
  ImageNeuralNetwork = 2,
  HistoryBased = 3,
};

class BaseReferenceAdjustmentMPCPathTracker : public BaseMPCPathTracker {
 public:
  PTR_TYPEDEFS(BaseReferenceAdjustmentMPCPathTracker);

  static constexpr auto static_name = "base_mpc";

  // Note all rosparams that are in the config yaml file need to be declared here first, though they can be then changes using the declareparam function for ros in the cpp file
  struct Config : public BaseMPCPathTracker::Config {
    PTR_TYPEDEFS(Config);

    // Topics
    std::string sig_img_topic = "signal";
    std::string dpt_img_topic = "depth";
    std::string imu_topic = "imu";
    
    // Reference Pose Adjustment Modes and Info
    ReferenceAdjustmentMode reference_adjustment_mode = ReferenceAdjustmentMode::None;
    std::string reference_adjustment_model_path = "";

    static void loadConfig(Config::Ptr config,  
		           const rclcpp::Node::SharedPtr& node,
                           const std::string& prefix = "path_planning");


    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning");
  };

  BaseReferenceAdjustmentMPCPathTracker(const Config::ConstPtr& config,
                 const RobotState::Ptr& robot_state,
                 const tactic::GraphBase::Ptr& graph,
                 const Callback::Ptr& callback);
  ~BaseReferenceAdjustmentMPCPathTracker() override;

 protected:
  virtual void loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                            const lgmath::se3::Transformation& T_p_r_extp,
                            const double state_p,
                            RobotState& robot_state,
                            const tactic::Timestamp& curr_time) override;

 private:
  const Config::ConstPtr base_config_;
  RobotState::Ptr robot_state_;

  rclcpp::Publisher<Command>::SharedPtr command_pub_;
  rclcpp::Publisher<PathInfoMsg>::SharedPtr path_info_pub_;

  BaseErrorPredictor::Ptr error_predictor_ = nullptr;

};

}  // namespace path_planning
}  // namespace vtr
