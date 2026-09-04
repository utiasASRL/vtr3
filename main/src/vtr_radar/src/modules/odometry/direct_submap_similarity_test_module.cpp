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
 * \file direct_submap_similarity_test_module.cpp
 * \author Daniil Lisus, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/direct_submap_similarity_test_module.hpp"

namespace vtr {
namespace radar {

using namespace tactic;

auto SubmapSimilarityTestModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->min_registration_score = node->declare_parameter<float>(param_prefix + ".min_registration_score", config->min_registration_score);
  config->max_translation = node->declare_parameter<float>(param_prefix + ".max_translation", config->max_translation);
  config->max_rotation = node->declare_parameter<float>(param_prefix + ".max_rotation", config->max_rotation);
  // clang-format on
  return config;
}

void SubmapSimilarityTestModule::setReference_(const cv::Mat &scan,
                                               double scan_res) {
  scan.convertTo(reference_scan_, CV_32F, 1.0 / 255.0);
  reference_scan_res_ = scan_res;
}

double SubmapSimilarityTestModule::registrationScore_(
    const cv::Mat &current, double current_res,
    const EdgeTransform &T_cur_ref) const {
  if (reference_scan_.empty() || current.empty()) return 1.0;

  // Planar rigid transform (rotation about z only): p_cur = R * p_ref + t.
  const auto M = T_cur_ref.matrix();
  const double tx = M(0, 3), ty = M(1, 3);
  const double c = M(0, 0), s = M(1, 0);

  const int h_cur = current.rows, w_cur = current.cols;
  const int h_ref = reference_scan_.rows, w_ref = reference_scan_.cols;

  // For every pixel of the current scan, find where it lands in the
  // reference scan once the accumulated pose estimate is undone
  cv::Mat map_x(h_cur, w_cur, CV_32F), map_y(h_cur, w_cur, CV_32F);
  for (int v = 0; v < h_cur; ++v) {
    const double x_cur = (h_cur - 1) / 2.0 * current_res - v * current_res;
    for (int u = 0; u < w_cur; ++u) {
      const double y_cur = u * current_res - (w_cur - 1) / 2.0 * current_res;

      // current -> reference: p_ref = R^T * (p_cur - t)
      const double dx = x_cur - tx, dy = y_cur - ty;
      const double x_ref = c * dx + s * dy;
      const double y_ref = -s * dx + c * dy;

      map_x.at<float>(v, u) = static_cast<float>(y_ref / reference_scan_res_ + (w_ref - 1) / 2.0);
      map_y.at<float>(v, u) = static_cast<float>((h_ref - 1) / 2.0 - x_ref / reference_scan_res_);
    }
  }

  // Border-clamped bilinear resample of the reference scan onto the current scan's grid
  cv::Mat warped_ref;
  cv::remap(reference_scan_, warped_ref, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_REPLICATE);

  // score = sum(warped_reference .* current) / sum(reference^2)
  const double numerator = cv::sum(warped_ref.mul(current))[0];
  const double denominator = cv::sum(reference_scan_.mul(reference_scan_))[0];
  return denominator > 1e-9 ? numerator / denominator : 1.0;
}

void SubmapSimilarityTestModule::run_(QueryCache &qdata0, OutputCache &,
                                      const Graph::Ptr &,
                                      const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  // input
  const auto &first_frame = *qdata.first_frame;
  const auto &T_r_v = *qdata.T_r_v_odo;
  const auto &success = *qdata.odo_success;
  const auto &vertex_result = *qdata.vertex_test_result;
  // output
  auto &result = *qdata.submap_test_result;

  // check first frame
  if (first_frame) {
    result = SubmapTestResult::CREATE_SUBMAP;
    if (qdata.smoothed_scan && qdata.smoothed_scan_res)
      setReference_(*qdata.smoothed_scan, *qdata.smoothed_scan_res);
    return;
  }

  // check if we successfully registered this frame
  if (!success) return;

  if (vertex_result != VertexTestResult::CREATE_VERTEX) return;

  // Accumulated pose since the reference scan, in the robot frame -- same
  // quantity and semantics as tactic::SubmapTestModule::T_v_odo_submap_v_.
  T_v_odo_submap_v_ = T_r_v * T_v_odo_submap_v_;

  const auto se3vec = T_v_odo_submap_v_.vec();
  const auto translation_distance = se3vec.head<3>().norm();
  const auto rotation_distance = se3vec.tail<3>().norm() * 57.29577;  // 180/pi
  CLOG(DEBUG, static_name) << "Total translation: " << translation_distance
                           << ", total rotation: " << rotation_distance;

  bool create_submap = false;

  if (translation_distance >= config_->max_translation ||
      rotation_distance >= config_->max_rotation) {
    CLOG(DEBUG, static_name) << "Hard translation/rotation cap reached - creating new submap.";
    create_submap = true;
  } else if (qdata.smoothed_scan && qdata.smoothed_scan_res && qdata.T_s_r &&
            !reference_scan_.empty()) {
    // Compute transform from reference scan to current scan in radar frame
    const auto &T_s_r = *qdata.T_s_r;
    const EdgeTransform T_cur_ref_sensor = T_s_r * T_v_odo_submap_v_ * T_s_r.inverse();

    cv::Mat current_f;
    qdata.smoothed_scan->convertTo(current_f, CV_32F, 1.0 / 255.0);

    const double score = registrationScore_(current_f, *qdata.smoothed_scan_res, T_cur_ref_sensor);
    CLOG(DEBUG, static_name) << "Registration score vs. reference submap scan: " << score;

    if (score < config_->min_registration_score) create_submap = true;
  }

  if (create_submap) {
    result = SubmapTestResult::CREATE_SUBMAP;
    T_v_odo_submap_v_ = EdgeTransform(true);
    if (qdata.smoothed_scan && qdata.smoothed_scan_res)
      setReference_(*qdata.smoothed_scan, *qdata.smoothed_scan_res);
  }
}

}  // namespace radar
}  // namespace vtr
