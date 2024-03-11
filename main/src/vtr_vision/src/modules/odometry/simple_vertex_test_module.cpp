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
#include <vtr_vision/modules/odometry/simple_vertex_test_module.hpp>

namespace vtr {
namespace vision {

using namespace tactic;

auto SimpleVertexTestModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {

  auto simple_config = std::make_shared<SimpleVertexTestModule::Config>();

  // clang-format off
  simple_config->min_distance = node->declare_parameter<double>(param_prefix + ".min_distance", simple_config->min_distance);
  simple_config->min_creation_distance = node->declare_parameter<double>(param_prefix + ".min_creation_distance", simple_config->min_creation_distance);
  simple_config->max_creation_distance = node->declare_parameter<double>(param_prefix + ".max_creation_distance", simple_config->max_creation_distance);
  simple_config->rotation_threshold_min = node->declare_parameter<double>(param_prefix + ".rotation_threshold_min", simple_config->rotation_threshold_min);
  simple_config->rotation_threshold_max = node->declare_parameter<double>(param_prefix + ".rotation_threshold_max", simple_config->rotation_threshold_max);
  simple_config->match_threshold_min_count = node->declare_parameter<int>(param_prefix + ".match_threshold_min_count", simple_config->match_threshold_min_count);
  simple_config->match_threshold_fail_count = node->declare_parameter<int>(param_prefix + ".match_threshold_fail_count", simple_config->match_threshold_fail_count);
  // clang-format on
  return simple_config;

}

void SimpleVertexTestModule::run_(tactic::QueryCache &qdata0, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) {

  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);

  // default to creating candidate
  *qdata.vertex_test_result = VertexTestResult::CREATE_CANDIDATE;

  /// \todo yuchen check first frame, is this appropriate? what if we do not
  /// have enough features here?
  if (*qdata.first_frame) {
    CLOG(DEBUG, "First frame encountered, make it a keyframe.");
    *qdata.vertex_test_result = VertexTestResult::CREATE_VERTEX;
    return;
  }

  int32_t inlier_count = 0;
  if (qdata.ransac_matches.valid()) {
    auto &matches = *qdata.ransac_matches;
    for (auto &rig : matches) {
      for (auto &channel : rig.channels) {
        inlier_count += channel.matches.size();
      }
    }
  } else {
    // if we don't have ransac data, this cache probably doesn't have images, do
    // nothing
    *qdata.vertex_test_result = VertexTestResult::DO_NOTHING;
    return;
  }
#if false
  if (qdata.triangulated_matches.valid() == true) {
    auto &matches = *qdata.triangulated_matches;
    for (auto &rig : matches) {
      for (auto &channel : rig.channels) {
        inlier_count += channel.matches.size();
      }
    }
  }
#endif
  if (inlier_count < config_->match_threshold_fail_count) {
    CLOG(ERROR, "stereo.vertex_test") << "Uh oh, " << inlier_count << " is not enough inliers";
    *qdata.vertex_test_result = VertexTestResult::DO_NOTHING;
    *qdata.odo_success = false;
    return;
  }

  if (*qdata.odo_success == false) {
    CLOG(ERROR, "stereo.vertex_test") << "Uh oh, state estimation failed";
    *qdata.vertex_test_result = VertexTestResult::DO_NOTHING;
    return;
  }

  if (qdata.T_r_m.valid()) {
    // Inputs, Query Frame, Map Frame, Inliers, Initial Guess
    const auto &T_query_map = *qdata.T_r_m;

    // extract the translational component of the distance
    auto se3Vec = T_query_map.vec();

    // The maximum component of the translation
    double translation_distance = se3Vec.head<3>().norm();

    // The maximum component of the rotation.
    double rotation_distance = se3Vec.tail<3>().norm() * 57.29577;

    // If we have not moved enough to create a vertex, then just return
    if (translation_distance < config_->min_distance &&
        rotation_distance < config_->rotation_threshold_min) {
      *qdata.vertex_test_result = VertexTestResult::CREATE_CANDIDATE;
      return;
    }

    else if (translation_distance > config_->max_creation_distance) {
      CLOG(ERROR, "stereo.vertex_test") << "Uh oh, we have a huge translation " << translation_distance
                 << " m";
      *qdata.vertex_test_result = VertexTestResult::DO_NOTHING;
      *qdata.odo_success = false;
      return;
    } else if (rotation_distance > config_->rotation_threshold_max) {
      CLOG(ERROR, "stereo.vertex_test") << "Uh oh, we have a huge rotation " << rotation_distance
                 << " deg";
      *qdata.vertex_test_result = VertexTestResult::DO_NOTHING;
      *qdata.odo_success = false;
      return;
    }

    // check if to see if we have met any candidate creation criteria
    if (translation_distance > config_->min_creation_distance) {
      *qdata.vertex_test_result = VertexTestResult::CREATE_VERTEX;
    } else if (rotation_distance > config_->rotation_threshold_min) {
      *qdata.vertex_test_result = VertexTestResult::CREATE_VERTEX;
    } else if (inlier_count < config_->match_threshold_min_count) {
      *qdata.vertex_test_result = VertexTestResult::CREATE_VERTEX;
    }
  } else {
    CLOG(ERROR, "stereo.vertex_test") << "QVO did not estimate T_r_m";
    *qdata.vertex_test_result = VertexTestResult::DO_NOTHING;
    *qdata.odo_success = false;
  }

  CLOG(DEBUG, "stereo.vertex_test") << "Simple vertex test result: "
             << static_cast<int>(*qdata.vertex_test_result);
}

}  // namespace vision
}  // namespace vtr
