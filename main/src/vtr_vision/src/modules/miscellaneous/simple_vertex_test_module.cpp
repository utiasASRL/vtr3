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
 * \file simple_vertex_test_module.cpp
 * \brief SimpleVertexTestModule class method definitions
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_vision/modules/miscellaneous/simple_vertex_test_module.hpp>

namespace vtr {
namespace vision {

using namespace tactic;

void SimpleVertexTestModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                           const std::string param_prefix) {
  VertexCreationModule::configFromROS(node, param_prefix);
  simple_config_ = std::make_shared<Config>();
  auto casted_config =
      std::static_pointer_cast<VertexCreationModule::Config>(simple_config_);
  *casted_config = *config_;  // copy over base config
  // clang-format off
  simple_config_->min_creation_distance = node->declare_parameter<double>(param_prefix + ".min_creation_distance", simple_config_->min_creation_distance);
  simple_config_->max_creation_distance = node->declare_parameter<double>(param_prefix + ".max_creation_distance", simple_config_->max_creation_distance);
  simple_config_->min_distance = node->declare_parameter<double>(param_prefix + ".min_distance", simple_config_->min_distance);
  simple_config_->rotation_threshold_min = node->declare_parameter<double>(param_prefix + ".rotation_threshold_min", simple_config_->rotation_threshold_min);
  simple_config_->rotation_threshold_max = node->declare_parameter<double>(param_prefix + ".rotation_threshold_max", simple_config_->rotation_threshold_max);
  simple_config_->match_threshold_min_count = node->declare_parameter<int>(param_prefix + ".match_threshold_min_count", simple_config_->match_threshold_min_count);
  simple_config_->match_threshold_fail_count = node->declare_parameter<int>(param_prefix + ".match_threshold_fail_count", simple_config_->match_threshold_fail_count);
  // clang-format on
}

void SimpleVertexTestModule::runImpl(QueryCache &qdata0,
                                     const Graph::ConstPtr &) {
  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);

  // default to creating candidate
  *qdata.keyframe_test_result = KeyframeTestResult::CREATE_CANDIDATE;

  /// \todo yuchen check first frame, is this appropriate? what if we do not
  /// have enough features here?
  if (*qdata.first_frame) {
    LOG(DEBUG) << "First frame encountered, make it a keyframe.";
    *qdata.keyframe_test_result = KeyframeTestResult::CREATE_VERTEX;
    return;
  }

  int32_t inlier_count = 0;
  if (qdata.ransac_matches.is_valid() == true) {
    auto &matches = *qdata.ransac_matches;
    for (auto &rig : matches) {
      for (auto &channel : rig.channels) {
        inlier_count += channel.matches.size();
      }
    }
  } else {
    // if we don't have ransac data, this cache probably doesn't have images, do
    // nothing
    *qdata.keyframe_test_result = KeyframeTestResult::DO_NOTHING;
    return;
  }
#if false
  if (qdata.triangulated_matches.is_valid() == true) {
    auto &matches = *qdata.triangulated_matches;
    for (auto &rig : matches) {
      for (auto &channel : rig.channels) {
        inlier_count += channel.matches.size();
      }
    }
  }
#endif
  if (inlier_count < simple_config_->match_threshold_fail_count) {
    LOG(ERROR) << "Uh oh, " << inlier_count << " is not enough inliers";
    *qdata.keyframe_test_result = KeyframeTestResult::FAILURE;
    *qdata.success = false;
    return;
  }

  if (*qdata.steam_failure == true || *qdata.success == false) {
    LOG(ERROR) << "Uh oh, state estimation failed";
    *qdata.success = false;
    return;
  }

  if (qdata.T_r_m.is_valid() == true) {
    // Inputs, Query Frame, Map Frame, Inliers, Initial Guess
    const auto &T_query_map = *qdata.T_r_m;

    // extract the translational component of the distance
    auto se3Vec = T_query_map.vec();

    // The maximum component of the translation
    double translation_distance = se3Vec.head<3>().norm();

    // The maximum component of the rotation.
    double rotation_distance = se3Vec.tail<3>().norm() * 57.29577;

    // If we have not moved enough to create a vertex, then just return
    if (translation_distance < simple_config_->min_distance &&
        rotation_distance < .1) {
      return;
    }

    else if (translation_distance > simple_config_->max_creation_distance) {
      LOG(ERROR) << "Uh oh, we have a huge translation " << translation_distance
                 << " m";
      *qdata.keyframe_test_result = KeyframeTestResult::FAILURE;
      *qdata.success = false;
      return;
    } else if (rotation_distance > simple_config_->rotation_threshold_max) {
      LOG(ERROR) << "Uh oh, we have a huge rotation " << rotation_distance
                 << " deg";
      *qdata.keyframe_test_result = KeyframeTestResult::FAILURE;
      *qdata.success = false;
      return;
    }

    // check if to see if we have met any candidate creation criteria
    if (translation_distance > simple_config_->min_creation_distance) {
      *qdata.keyframe_test_result = KeyframeTestResult::CREATE_VERTEX;
      return;
    } else if (rotation_distance > simple_config_->rotation_threshold_min) {
      *qdata.keyframe_test_result = KeyframeTestResult::CREATE_VERTEX;
      return;
    } else if (inlier_count < simple_config_->match_threshold_min_count) {
      *qdata.keyframe_test_result = KeyframeTestResult::CREATE_VERTEX;
      return;
    }
  } else {
    LOG(ERROR) << "QVO did not estimate T_r_m";
    *qdata.keyframe_test_result = KeyframeTestResult::FAILURE;
    *qdata.success = false;
  }

  LOG(DEBUG) << "Simple vertex test result: "
             << static_cast<int>(*qdata.keyframe_test_result);
}

}  // namespace vision
}  // namespace vtr
