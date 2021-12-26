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
 * \file keyframe_test_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/keyframe_test_module_v2.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto KeyframeTestModuleV2::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                           const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->max_translation = node->declare_parameter<float>(param_prefix + ".max_translation", config->max_translation);
  config->max_rotation = node->declare_parameter<float>(param_prefix + ".max_rotation", config->max_rotation);
  // clang-format on
  return config;
}

void KeyframeTestModuleV2::runImpl(QueryCache &qdata0, OutputCache &,
                                   const Graph::Ptr &,
                                   const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  // default to
  qdata.keyframe_test_result = KeyframeTestResult::DO_NOTHING;

  // input
  const auto &first_frame = *qdata.first_frame;
  const auto &T_r_m = *qdata.T_r_m_odo;
  const auto &success = *qdata.odo_success;
  // output
  auto &result = *qdata.keyframe_test_result;

  // check first frame
  if (first_frame) result = KeyframeTestResult::CREATE_VERTEX;

  // check if we successfully register this frame
  if (!success) {
    result = KeyframeTestResult::FAILURE;
    return;
  }

  auto se3vec = T_r_m.vec();
  auto translation_distance = se3vec.head<3>().norm();
  auto rotation_distance = se3vec.tail<3>().norm() * 57.29577;  // 180/pi
  CLOG(DEBUG, "lidar.keyframe_test")
      << "Total translation: " << translation_distance
      << ", total rotation: " << rotation_distance;

  if (translation_distance >= config_->max_translation ||
      rotation_distance >= config_->max_rotation) {
    result = KeyframeTestResult::CREATE_VERTEX;
  }
}

}  // namespace lidar
}  // namespace vtr