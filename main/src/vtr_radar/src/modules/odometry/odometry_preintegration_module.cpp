// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
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
 * \file odometry_preintegration_module.cpp
 * \author Sven Lilge, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/odometry_preintegration_module.hpp"

#include "vtr_radar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace radar {

using namespace tactic;
using namespace steam;
using namespace steam::se3;
using namespace steam::traj;
using namespace steam::vspace;

auto OdometryPreintegrationModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();// clang-format on
  config->bias = node->declare_parameter<double>(param_prefix + ".bias", config->bias);
  return config;
}

void OdometryPreintegrationModule::run_(QueryCache &qdata0, OutputCache &,
                             const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  // Skip frame and reset preint if query is for radar scan or if odometry has not been initialized (we will wait until radar did this)
  if (qdata.radar_data || !qdata.sliding_map_odo) {
    reset_preint_ = true;
    return;
  }

  // Handle first-ever seen gyro message
  if(!qdata.prev_gyro_msg) {
    CLOG(DEBUG, "radar.odometry_preintegration") << "First gyro message received. Cannot preintegrate yet.";
    // This is the first time we are every receiving gyro, we cannot preintegrate
    // Save current gyro message and update preintegration terms
    qdata.prev_gyro_msg.emplace(*qdata.gyro_msg);
    qdata.stamp_start_pre_integration.emplace(*qdata.stamp);
    qdata.stamp_end_pre_integration.emplace(*qdata.stamp);
    qdata.preintegrated_delta_yaw = 0.0;
    reset_preint_ = false;

    return;
  }

  if (reset_preint_) {
    qdata.preintegrated_delta_yaw = 0.0;
    // Set start time to the previous end time and end time to current stamp
    *qdata.stamp_start_pre_integration = *qdata.stamp_end_pre_integration;
    *qdata.stamp_end_pre_integration = *qdata.stamp;
    reset_preint_ = false;
  }

  CLOG(DEBUG, "radar.odometry_preintegration") << "Retrieve input data and setup evaluators.";
  CLOG(DEBUG, "radar.odometry_preintegration") << "stamp_start_pre_integration: " << *qdata.stamp_start_pre_integration;
  CLOG(DEBUG, "radar.odometry_preintegration") << "stamp_end_pre_integration: " << *qdata.stamp_end_pre_integration;

  // Inputs
  const auto &current_time_stamp = *qdata.stamp;
  const auto &prev_time_stamp = *qdata.stamp_end_pre_integration;
  const auto &prev_gyro_msg = *qdata.prev_gyro_msg;

  // Distance between current and last time
  Time prev_time(static_cast<int64_t>(prev_time_stamp)); // get previous odometry timestamp
  Time cur_time(static_cast<int64_t>(current_time_stamp));
  
  // Integrate last gyro measurement (we don't multiply by -1 here due to the frame convention in the icp module)

  Eigen::Vector3d angular_velocity;
  angular_velocity << prev_gyro_msg.angular_velocity.x, prev_gyro_msg.angular_velocity.y, prev_gyro_msg.angular_velocity.z;


  CLOG(DEBUG, "radar.odometry_preintegration") << "Ang Before " << angular_velocity;
  angular_velocity = C_s_r * angular_velocity;
  CLOG(DEBUG, "radar.odometry_preintegration") << "Ang After " << angular_velocity;

  double delta_yaw = (cur_time - prev_time).seconds() * (angular_velocity(2, 0) - config_->bias);

  // Set preintegrated value
  double value = (qdata.preintegrated_delta_yaw) ? *qdata.preintegrated_delta_yaw : 0.0;

  CLOG(DEBUG, "radar.odometry_preintegration") << "Current delta yaw value: " << delta_yaw;
  CLOG(DEBUG, "radar.odometry_preintegration") << "Old preintegrated yaw value: " << value;

  value += delta_yaw;

  CLOG(DEBUG, "radar.odometry_preintegration") << "New preintegrated yaw value: " << value;
   
  if(!qdata.preintegrated_delta_yaw)
    qdata.preintegrated_delta_yaw.emplace(value);
  else
    *qdata.preintegrated_delta_yaw = value;

  // Set preintegration end time to current time
  *qdata.stamp_end_pre_integration = current_time_stamp;

  // Set prev gyro msg to the current message for the next preintegration
  *qdata.prev_gyro_msg = *qdata.gyro_msg;

  }
  // clang-format on
}  // namespace radar
}  // namespace vtr