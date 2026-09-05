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

  // clang-format on
  return config;
}

void GyroBiasEstimationModule::run_(tactic::QueryCache &qdata0, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) {
  const auto& last_vel = *qdata0.w_v_r_in_r_odo;

  try {
    CacheType& qdata = dynamic_cast<CacheType &>(qdata0);
    if (last_vel.head<3>().norm() < config_->max_vel) {
      for(const auto& gyro_msg : *qdata.gyro_msgs) {
        gyro_bias_ = config_->alpha * gyro_bias_ + (1 - config_->alpha) * Eigen::Vector3d(gyro_msg.angular_velocity.x, gyro_msg.angular_velocity.y, gyro_msg.angular_velocity.z);
      }
      if (count_ < 10 / (1 - config_->alpha)) {
        count_ += qdata.gyro_msgs->size();
      }
      
      CLOG_EVERY_N(10, DEBUG, static_name) << "Gyro bias is now: " << gyro_bias_ / (1 - pow(config_->alpha, count_ ));
    }

    for(auto& gyro_msg : *qdata.gyro_msgs) {
      const double correction = (1 - pow(config_->alpha, count_ ));
      gyro_msg.angular_velocity.x -= gyro_bias_(0) / correction;
      gyro_msg.angular_velocity.y -= gyro_bias_(1) / correction;
      gyro_msg.angular_velocity.z -= gyro_bias_(2) / correction;
    }
  } catch(std::bad_cast& b) {
    CLOG(ERROR, static_name) << "Requested gyro bias removal for a pipeline that does not support imu messages!";
    return;
  }

}

}  // namespace navigation
}  // namespace vtr