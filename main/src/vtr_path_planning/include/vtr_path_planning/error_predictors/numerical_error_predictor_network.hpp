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
#include <lgmath/se3/Transformation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <mutex>

namespace vtr {
namespace path_planning {

using ImuMsg = sensor_msgs::msg::Imu;

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

  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
  std::shared_ptr<ImuMsg> cur_imu_msg_ = nullptr;

  // Rolling buffer of world-frame gravity samples for averaging.
  // Each entry is (timestamp_ns, accel_world_frame).
  static constexpr double kGravWindowSeconds = 1.0;
  struct ImuSample {
    int64_t stamp_ns;
    std::array<double, 3> accel_world;
  };
  std::deque<ImuSample> imu_buffer_;
  std::mutex imu_mutex_;
};

}  // namespace path_planning
}  // namespace vtr

#endif  // VTR_PATH_PLANNING_ERROR_PREDICTORS_NUMERICAL_ERROR_PREDICTOR_NETWORK_HPP
