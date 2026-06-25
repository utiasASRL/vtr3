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
 * \file numerical_error_predictor_network.hpp
 * \author Luka Antonyshyn Autonomous Space Robotics Lab (ASRL)
 */

#ifndef VTR_PATH_PLANNING_ERROR_PREDICTORS_NUMERICAL_ERROR_PREDICTOR_NETWORK_HPP
#define VTR_PATH_PLANNING_ERROR_PREDICTORS_NUMERICAL_ERROR_PREDICTOR_NETWORK_HPP

#include "vtr_path_planning/error_predictors/base_error_predictor.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vtr {
namespace path_planning {

class NumericalErrorPredictorNetwork : public BaseErrorPredictor {
public:
  PTR_TYPEDEFS(NumericalErrorPredictorNetwork);

  NumericalErrorPredictorNetwork(rclcpp::Node::SharedPtr node,
                                 const std::string& imu_topic,
                                 const std::string& model_path,
                                 bool use_gpu = false);
  ~NumericalErrorPredictorNetwork();

  void predictError(const RobotState& robot_state,
                    const tactic::Timestamp& curr_time,
                    std::vector<lgmath::se3::Transformation>& reference_poses) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::shared_ptr<sensor_msgs::msg::Imu> cur_imu_msg_ = nullptr;
};

}  // namespace path_planning
}  // namespace vtr

#endif  // VTR_PATH_PLANNING_ERROR_PREDICTORS_NUMERICAL_ERROR_PREDICTOR_NETWORK_HPP
