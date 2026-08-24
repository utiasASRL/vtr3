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
 * \file direct_scan_to_pointmap_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/direct_scan_to_pointmap_module.hpp"

#include <algorithm>
#include <unordered_set>

#include <opencv2/core/eigen.hpp>

#include "pcl_conversions/pcl_conversions.h"
#include "vtr_radar/dr_ba/scans/local_map_scan.hpp"

namespace vtr {
namespace radar {

using namespace tactic;

auto ScanToMapModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->map_resolution = node->declare_parameter<float>(param_prefix + ".map_resolution", config->map_resolution);
  config->scan_max_range = node->declare_parameter<float>(param_prefix + ".scan_max_range", config->scan_max_range);
  config->window_max_dist = node->declare_parameter<float>(param_prefix + ".window_max_dist", config->window_max_dist);
  config->gauss_blur_sigma = node->declare_parameter<double>(param_prefix + ".gauss_blur_sigma", config->gauss_blur_sigma);
  config->adaptive_blur = node->declare_parameter<bool>(param_prefix + ".adaptive_blur", config->adaptive_blur);
  config->max_blur_sigma = node->declare_parameter<double>(param_prefix + ".max_blur_sigma", config->max_blur_sigma);
  config->min_int_val_tol = node->declare_parameter<double>(param_prefix + ".min_int_val_tol", config->min_int_val_tol);
  config->min_percent_nonzero = node->declare_parameter<double>(param_prefix + ".min_percent_nonzero", config->min_percent_nonzero);
  config->ba_opts.meas_std = node->declare_parameter<double>(param_prefix + ".meas_std", config->ba_opts.meas_std);
  config->ba_opts.range_factor = node->declare_parameter<double>(param_prefix + ".range_factor", config->ba_opts.range_factor);
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

cv::Mat ScanToMapModule::blurScan(const cv::Mat &scan) const {
  cv::Mat img;
  scan.convertTo(img, CV_32F);

  // Raise sigma until enough of the scan is non-zero, as the dr_ba mapper does
  cv::Mat blurred;
  double sigma = config_->adaptive_blur ? 3.0 : config_->gauss_blur_sigma;
  double percent_nonzero = 0.0;
  do {
    const int ksize = (static_cast<int>(std::ceil(6.0 * sigma)) | 1);
    cv::GaussianBlur(img, blurred, cv::Size(ksize, ksize), sigma);

    double min_val, max_val;
    cv::minMaxLoc(blurred, &min_val, &max_val);
    blurred = (blurred - min_val) / std::max(max_val - min_val, 1e-9);

    percent_nonzero = 100.0 * cv::countNonZero(blurred > config_->min_int_val_tol) /
                      (blurred.rows * blurred.cols);
    if (!config_->adaptive_blur || sigma > config_->max_blur_sigma) break;
    sigma += 2.0;
  } while (percent_nonzero < config_->min_percent_nonzero);

  cv::threshold(blurred, blurred, 0.0, 0.0, cv::THRESH_TOZERO);
  cv::threshold(blurred, blurred, 1.0, 1.0, cv::THRESH_TRUNC);

  cv::Mat quantized;
  blurred.convertTo(quantized, CV_8U, 255.0);
  return quantized;
}

cv::Mat ScanToMapModule::cropScan(const cv::Mat &scan, double res) const {
  const int cx = (scan.cols - 1) / 2;
  const int cy = (scan.rows - 1) / 2;
  const int max_hw = std::min({cx, cy, scan.cols - 1 - cx, scan.rows - 1 - cy});
  int hw = static_cast<int>(std::round(config_->scan_max_range / res));
  if (hw > max_hw) {
    CLOG(WARNING, static_name) << "scan_max_range exceeds the local map extent, cropping to "
                               << max_hw * res << " m instead";
    hw = max_hw;
  }
  // Clone so the window does not pin the full sized scan alive
  return scan(cv::Rect(cx - hw, cy - hw, 2 * hw + 1, 2 * hw + 1)).clone();
}

void ScanToMapModule::trimWindow() {
  // Filter the whole window rather than popping the front: after a turn or a
  // loop an older scan can sit closer to the submap center than a newer one.
  // The newest scan is the anchor itself, so it always survives.
  if (config_->window_max_dist > 0.0) {
    const double max_dist_sq = std::pow(config_->window_max_dist, 2);
    window_.erase(std::remove_if(window_.begin(), window_.end(),
                                 [&max_dist_sq](const WindowEntry &entry) {
                                   return entry.T_a_s.r_ab_inb().head<2>().squaredNorm() > max_dist_sq;
                                 }),
                  window_.end());
  }
}

std::vector<pointmap::VoxKey> ScanToMapModule::initVoxelKeys() const {
  const double res = config_->map_resolution;
  const int n = static_cast<int>(std::ceil(config_->scan_max_range / res)) + 1;
  const double max_range_sq = std::pow(config_->scan_max_range, 2);

  std::unordered_set<pointmap::VoxKey> keys;
  for (const auto &entry : window_) {
    const Eigen::Vector3d r = entry.T_a_s.r_ab_inb();
    const int cx = static_cast<int>(std::floor(r(0) / res));
    const int cy = static_cast<int>(std::floor(r(1) / res));
    for (int dx = -n; dx <= n; ++dx) {
      for (int dy = -n; dy <= n; ++dy) {
        const pointmap::VoxKey key(cx + dx, cy + dy, 0);
        // Sample and store voxel centers so PointMap::getKey round-trips
        const double x = (key.x + 0.5) * res;
        const double y = (key.y + 0.5) * res;
        if (std::pow(x - r(0), 2) + std::pow(y - r(1), 2) > max_range_sq) continue;
        keys.insert(key);
      }
    }
  }

  // Sort keys for deterministic mapping
  std::vector<pointmap::VoxKey> sorted_keys(keys.begin(), keys.end());
  std::sort(sorted_keys.begin(), sorted_keys.end(),
            [](const pointmap::VoxKey &a, const pointmap::VoxKey &b) {
              return (a.x != b.x) ? (a.x < b.x) : (a.y < b.y);
            });
  return sorted_keys;
}

void ScanToMapModule::rebuildMap(PointMap<PointWithInfo> &map) const {
  const auto keys = initVoxelKeys();
  map.clear();
  if (keys.empty()) return;

  const double res = config_->map_resolution;
  const double max_range_sq = std::pow(config_->scan_max_range, 2);
  std::vector<double> numerator(keys.size(), 0.0);
  std::vector<double> denominator(keys.size(), 0.0);

  // Scan-outer keeps only one scan expanded to float at a time
  for (const auto &entry : window_) {
    cv::Mat img_float;
    entry.blurred.convertTo(img_float, CV_32F, 1.0 / 255.0);
    Eigen::MatrixXf local_map;
    cv::cv2eigen(img_float, local_map);

    const ba::LocalMapScan scan(entry.stamp, 0, entry.res,
                                config_->ba_opts, entry.T_a_s.toSE2().toSE3(),
                                lgmath::se3::Transformation(), local_map);
    const Eigen::Vector3d r_scan = entry.T_a_s.r_ab_inb();

    // Each iteration owns one voxel, so the accumulators need no atomics
    #pragma omp parallel for num_threads(config_->num_threads) schedule(static)
    for (size_t i = 0; i < keys.size(); ++i) {
      const double x = (keys[i].x + 0.5) * res;
      const double y = (keys[i].y + 0.5) * res;
      // Gate on the scan's range; its cropped image is square, so the corners
      // hold zeros that would otherwise fuse as real measurements
      if (std::pow(x - r_scan(0), 2) + std::pow(y - r_scan(1), 2) > max_range_sq) continue;
      const std::optional<ba::Scan::Measurement> meas = scan.interpolate(x, y);
      if (!meas.has_value()) continue;
      const double weight = 1.0 / meas->covariance;
      numerator[i] += meas->intensity * weight;
      denominator[i] += weight;
    }
  }

  pcl::PointCloud<PointWithInfo> map_pc;
  map_pc.reserve(keys.size());
  for (size_t i = 0; i < keys.size(); ++i) {
    if (denominator[i] <= 0.0) continue;
    PointWithInfo point;
    point.x = static_cast<float>((keys[i].x + 0.5) * res);
    point.y = static_cast<float>((keys[i].y + 0.5) * res);
    point.z = 0.0f;
    point.intensity = static_cast<float>(numerator[i] / denominator[i]);
    map_pc.push_back(point);
  }
  map.update(map_pc);
}

void ScanToMapModule::run_(QueryCache &qdata0, OutputCache &,
                                        const Graph::Ptr &,
                                        const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  // Do nothing if qdata does not contain any radar data (was populated by gyro)
  if(!qdata.radar_data)
  {
    return;
  }

  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    map_pub_ = qdata.node->create_publisher<PointCloudMsg>("submap_odo", 5);
    // clang-format on
    publisher_initialized_ = true;
  }


  // construct output (construct the map if not exist)
  if (!qdata.sliding_map_odo)
    qdata.sliding_map_odo.emplace(config_->map_resolution);

      // Do not update the map if registration failed.
  if (!(*qdata.odo_success)) {
    CLOG(WARNING, static_name) << "DRO failed - not updating the submap.";
    return;
  }

  // The window only changes on keyframes
  if (*qdata.vertex_test_result != VertexTestResult::CREATE_VERTEX) return;

  if (!qdata.smoothed_scan || !qdata.smoothed_scan_res) {
    CLOG(WARNING, static_name) << "No local map available - not updating the submap.";
    return;
  }

  // Get input and output data
  // input
  const auto T_r_s = qdata.T_s_r->inverse();
  auto &sliding_map_odo = *qdata.sliding_map_odo;

  // Re-anchor the window onto the sensor frame of this scan, where the DRO
  // local maps live. Anchoring in sensor frame to avoid SE3 transform issues
  const lgmath::se3::Transformation T_a_new_a_prev((*qdata.T_s_r * *qdata.T_r_v_odo * T_r_s).matrix());
  for (auto &entry : window_) entry.T_a_s = T_a_new_a_prev * entry.T_a_s;

  // Add new scan to the window anchored at itself
  const double scan_res = *qdata.smoothed_scan_res;
  window_.push_back({*qdata.stamp, cropScan(blurScan(*qdata.smoothed_scan), scan_res),
                     lgmath::se3::Transformation(), scan_res});
  
  // Trim the window to only keep scans within the configured distance of anchor
  trimWindow();

  // Reconstruct map using new window
  rebuildMap(sliding_map_odo);

  // Update the sliding map's tf
  sliding_map_odo.T_vertex_this() = qdata.T_r_v_odo->inverse() * T_r_s;

  CLOG(DEBUG, static_name) << "Submap has " << sliding_map_odo.size()
                           << " voxels fused from " << window_.size() << " scans";

  /// \note this visualization converts point map from its own frame to the
  /// vertex frame, so can be slow.
  if (config_->visualize) {
    // clang-format off
    // publish the map
    {
      Eigen::Matrix4f T_v_m = sliding_map_odo.T_vertex_this().matrix().cast<float>();
      // Sink the published copy so it does not bury the path; stored map is untouched
      T_v_m(2, 3) -= 1.0f;
      auto point_map = sliding_map_odo.point_cloud();  // makes a copy
      auto map_point_mat = point_map.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
      map_point_mat = T_v_m * map_point_mat;

      PointCloudMsg pc2_msg;
      pcl::toROSMsg(point_map, pc2_msg);
      pc2_msg.header.frame_id = "odo vertex frame";
      pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
      map_pub_->publish(pc2_msg);
    }

    // clang-format on
  }
}

}  // namespace radar
}  // namespace vtr
