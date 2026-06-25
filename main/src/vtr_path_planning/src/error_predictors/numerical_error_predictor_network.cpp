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
 * \file numerical_error_predictor_network.cpp
 * \author Luka Antonyshyn, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/error_predictors/numerical_error_predictor_network.hpp"

namespace vtr {
namespace path_planning {

struct NumericalErrorPredictorNetwork::Impl {};

NumericalErrorPredictorNetwork::NumericalErrorPredictorNetwork(
    rclcpp::Node::SharedPtr node,
    const std::string& imu_topic,
    const std::string& /*model_path*/,
    bool /*use_gpu*/)
    : impl_(std::make_unique<Impl>()) {
  imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::QoS(1),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { cur_imu_msg_ = msg; });
}

NumericalErrorPredictorNetwork::~NumericalErrorPredictorNetwork() = default;

void NumericalErrorPredictorNetwork::predictError(
    const RobotState& /*robot_state*/,
    const tactic::Timestamp& /*curr_time*/,
    std::vector<lgmath::se3::Transformation>& /*reference_poses*/) {
  // TODO: implement numerical network inference
}

}  // namespace path_planning
}  // namespace vtr
