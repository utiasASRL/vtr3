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
#include <deque>
#include <fstream>
#include <vector>

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

  struct Config : public BaseMPCPathTracker::Config {
    PTR_TYPEDEFS(Config);

    // Topics
    std::string sig_img_topic = "signal";
    std::string dpt_img_topic = "depth";
    std::string imu_topic = "imu";
    

    // Reference Pose Adjustment Modes and Info
    ReferenceAdjustmentMode reference_adjustment_mode = ReferenceAdjustmentMode::None;
    std::string reference_adjustment_model_path = "";
    std::string prediction_log_path = "";
    bool use_gpu = false;

    // Sub-mode used when reference_adjustment_mode == HistoryBased.
    HistoryLookupMode history_lookup_mode = HistoryLookupMode::PreviousRepeat;
    // A/B testing knobs -- see each predictor's constructor doc comment.
    bool history_lookup_invert_correction = false;
    bool nn_invert_correction = false;
    bool apply_corrections = true;
    bool lateral_only_correction = true;

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
  const tactic::GraphBase::Ptr graph_;

  rclcpp::Publisher<Command>::SharedPtr command_pub_;
  rclcpp::Publisher<PathInfoMsg>::SharedPtr path_info_pub_;

  BaseErrorPredictor::Ptr error_predictor_ = nullptr;

  // Logging: raw streams written to CSV, matched post-hoc by process_prediction_log.py.
  static constexpr int64_t kPredictionStepNs = 250000000LL;  // 0.25s in nanoseconds

  // One row per control cycle: robot world pose for post-hoc interpolation.
  struct PoseLogEntry {
    int64_t timestamp_ns;
    lgmath::se3::Transformation T_w_r;
    lgmath::se3::Transformation T_w_r_extp;
  };


  // One row per prediction: all world-frame context needed to compute actual error offline.
  struct PredLogEntry {
    int64_t  pred_timestamp_ns;
    int64_t  target_timestamp_ns;
    unsigned sid;
    int      step;
    double   pred_dx, pred_dy, pred_dyaw;
    lgmath::se3::Transformation T_w_ref;
    lgmath::se3::Transformation T_w_p_pred;
  };

  // One row per cycle: the exact live numeric inputs predictError() built and
  // fed to the model, for direct comparison against offline reconstructions.
  struct InputLogEntry {
    int64_t  timestamp_ns;
    unsigned sid;
    std::array<double, 6> loc_res;
    std::array<double, 6> odom_vel;
    std::array<double, 3> grav_vec;
    std::vector<std::array<double, 6>> sequence;
  };

  std::vector<PoseLogEntry> pose_log_;
  std::vector<PredLogEntry> pred_log_;
  std::vector<InputLogEntry> input_log_;
  std::string log_path_;

};

}  // namespace path_planning
}  // namespace vtr
