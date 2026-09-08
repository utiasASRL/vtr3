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
 * \file gyro_bias_estimation_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */


#include "vtr_navigation/modules/gyro_bias_estimation_module.hpp"
#include <typeinfo> // Required for typeid


namespace vtr {
namespace navigation {

auto GyroBiasEstimationModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  const int window_size = node->declare_parameter<int>(param_prefix + ".window_size", int(1/config->alpha));
  config->alpha = 1 - 1 / double(window_size);
  config->max_vel = node->declare_parameter<double>(param_prefix + ".max_velocity", config->max_vel);
  config->min_bias_time = node->declare_parameter<double>(param_prefix + ".min_bias_time", config->min_bias_time);

  // clang-format on
  return config;
}

void GyroBiasEstimationModule::run_(tactic::QueryCache &qdata0, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) {
  try {
    CacheType& qdata = dynamic_cast<CacheType &>(qdata0);

    const bool pre_odometry_pass = awaiting_odometry_;
    awaiting_odometry_ = !awaiting_odometry_;

    if (pre_odometry_pass) {
      // Pre-odometry pass: subtract the previous bias estimate and cache the raw readings for the update step below.
      raw_gyro_cache_.clear();
      raw_gyro_cache_.reserve(qdata.gyro_msgs->size());

      // Avoid dividing by zero before any bias update has ever happened.
      const double correction = count_ > 0 ? (1 - pow(config_->alpha, count_)) : 1.0;

      for (auto& gyro_msg : *qdata.gyro_msgs) {
        raw_gyro_cache_.emplace_back(gyro_msg.angular_velocity.x,
                                      gyro_msg.angular_velocity.y,
                                      gyro_msg.angular_velocity.z);

        const Eigen::Vector3d subtraction = gyro_bias_ / correction;
        gyro_msg.angular_velocity.x -= subtraction(0);
        gyro_msg.angular_velocity.y -= subtraction(1);
        gyro_msg.angular_velocity.z -= subtraction(2);

        total_bias_correction_ += subtraction;
      }

      CLOG_EVERY_N(10, DEBUG, static_name) << "Total gyro bias subtracted so far: " << total_bias_correction_.transpose();
    } else {
      // Post-odometry pass: update the bias estimate once the velocity has been below max_vel for min_bias_time.
      const auto& est_vel = *qdata.w_v_r_in_r_odo;
      if (*qdata.odo_success && est_vel.head<3>().norm() < config_->max_vel) {
        if (low_vel_start_stamp_ < 0) low_vel_start_stamp_ = *qdata.stamp;
        const double low_vel_duration = static_cast<double>(*qdata.stamp - low_vel_start_stamp_) * 1e-9;

        if (low_vel_duration >= config_->min_bias_time) {
          for (const auto& raw_gyro : raw_gyro_cache_) {
            gyro_bias_ = config_->alpha * gyro_bias_ + (1 - config_->alpha) * raw_gyro;
          }
          if (count_ < 10 / (1 - config_->alpha)) {
            count_ += raw_gyro_cache_.size();
          }

          CLOG_EVERY_N(10, DEBUG, static_name) << "Gyro bias is now: " << gyro_bias_ / (1 - pow(config_->alpha, count_ ));
        }
      } else {
        low_vel_start_stamp_ = -1;
      }
    }
  } catch(std::bad_cast& b) {
    CLOG(ERROR, static_name) << "Requested gyro bias removal for a pipeline that does not support imu messages!";
    return;
  }

}

}  // namespace navigation
}  // namespace vtr