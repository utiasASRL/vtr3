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
 * \file image_error_predictor_network.hpp
 * \author Luka Antonyshyn Autonomous Space Robotics Lab (ASRL)
 */

#ifndef VTR_PATH_PLANNING_ERROR_PREDICTORS_IMAGE_ERROR_PREDICTOR_NETWORK_HPP
#define VTR_PATH_PLANNING_ERROR_PREDICTORS_IMAGE_ERROR_PREDICTOR_NETWORK_HPP

#include "vtr_path_planning/error_predictors/base_error_predictor.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <lgmath/se3/Transformation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>

namespace vtr {
namespace path_planning {

using ImageMsg = sensor_msgs::msg::Image;
using ImuMsg = sensor_msgs::msg::Imu;

class ImageErrorPredictorNetwork : public BaseErrorPredictor {
public:
  PTR_TYPEDEFS(ImageErrorPredictorNetwork);

  ImageErrorPredictorNetwork(rclcpp::Node::SharedPtr node,
                             const std::string& sig_img_topic,
                             const std::string& dpt_img_topic,
                             const std::string& imu_topic,
                             const std::string& model_path,
                             bool use_gpu = false,
                             bool invert_correction = false,
                             bool lateral_only_correction = true,
                             NNTargetMode nn_target_mode = NNTargetMode::Trajectory);
  ~ImageErrorPredictorNetwork();

  std::vector<std::array<double, 3>> predictError(
      const RobotState& robot_state, const tactic::Timestamp& curr_time,
      std::vector<lgmath::se3::Transformation>& reference_poses,
      const lgmath::se3::Transformation& T_p_r_extp,
      bool apply_correction = true,
      PredictorInputSnapshot* input_snapshot = nullptr) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  bool invert_correction_;
  bool lateral_only_correction_;
  NNTargetMode nn_target_mode_;

  rclcpp::Subscription<ImageMsg>::SharedPtr sig_img_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr dpt_img_sub_;
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;

  std::shared_ptr<ImageMsg> cur_sig_img_msg_ = nullptr;
  std::shared_ptr<ImageMsg> cur_dpt_img_msg_ = nullptr;

  // Max age of the last received signal/range image relative to the current
  // planning cycle before we warn that it's stale
  static constexpr double kMaxImageStalenessSeconds = 0.05;

  // Num. Secs to average IMU measurements to get gravity estimate
  static constexpr double kGravWindowSeconds = 1.0;
  static constexpr int kGravMaxSamples = 500;
  bool gravity_locked_ = false;
  int64_t gravity_first_stamp_ns_ = -1;
  Eigen::Vector3d gravity_world_sum_ = Eigen::Vector3d::Zero();
  int gravity_sample_count_ = 0;
  Eigen::Vector3d gravity_world_ = Eigen::Vector3d::Zero();
  std::mutex imu_mutex_;
};

}  // namespace path_planning
}  // namespace vtr

#endif  // VTR_PATH_PLANNING_ERROR_PREDICTORS_IMAGE_ERROR_PREDICTOR_NETWORK_HPP
