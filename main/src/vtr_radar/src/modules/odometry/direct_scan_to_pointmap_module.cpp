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
 * \author Alec Krawciw, Daniil Lisus, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/direct_scan_to_pointmap_module.hpp"

#include <opencv2/core/eigen.hpp>

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
  // clang-format on
  return config;
}

void ScanToMapModule::rebuildMap(PointMap<PointWithInfo> &map,
                                 const cv::Mat &scan, double scan_res,
                                 tactic::Timestamp stamp) const {
  map.clear();

  cv::Mat img_float;
  scan.convertTo(img_float, CV_32F, 1.0 / 255.0);
  Eigen::MatrixXf local_map;
  cv::cv2eigen(img_float, local_map);

  // Identity pose: the scan's own sensor frame is the map frame.
  const ba::LocalMapScan bascan(stamp, 0, scan_res, ba::OptimizationOptions(),
                                lgmath::se3::Transformation(),
                                lgmath::se3::Transformation(), local_map);

  // Voxelize at map_resolution over the full extent of the scan.
  const double res = config_->map_resolution;
  const double half_x = (scan.cols - 1) * scan_res / 2.0;
  const double half_y = (scan.rows - 1) * scan_res / 2.0;
  const int nx = static_cast<int>(std::ceil(half_x / res));
  const int ny = static_cast<int>(std::ceil(half_y / res));

  pcl::PointCloud<PointWithInfo> map_pc;
  map_pc.reserve((2 * nx + 1) * (2 * ny + 1));
  for (int cx = -nx; cx <= nx; ++cx) {
    for (int cy = -ny; cy <= ny; ++cy) {
      // Sample at voxel centers so PointMap::getKey round-trips
      const double x = (cx + 0.5) * res;
      const double y = (cy + 0.5) * res;
      const auto meas = bascan.interpolate(x, y);
      if (!meas.has_value()) continue;
      PointWithInfo point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      point.z = 0.0f;
      point.intensity = static_cast<float>(meas->intensity);
      map_pc.push_back(point);
    }
  }
  map.update(map_pc);
}

void ScanToMapModule::updateSubmap(RadarQueryCache &qdata) const {
  // Do nothing if qdata does not contain any radar data (was populated by gyro)
  if (!qdata.radar_data) return;

  // construct output (construct the map if not exist)
  if (!qdata.sliding_map_odo)
    qdata.sliding_map_odo.emplace(config_->map_resolution);

  // Do not update the map if registration failed.
  if (!(*qdata.odo_success)) {
    CLOG(WARNING, static_name) << "DRO failed - not updating the submap.";
    return;
  }

  if (!qdata.smoothed_scan || !qdata.smoothed_scan_res) {
    CLOG(WARNING, static_name) << "No local map available - not updating the submap.";
    return;
  }

  auto &sliding_map_odo = *qdata.sliding_map_odo;
  rebuildMap(sliding_map_odo, *qdata.smoothed_scan, *qdata.smoothed_scan_res,
            *qdata.stamp);

  // Update the sliding map's tf
  const auto T_r_s = qdata.T_s_r->inverse();
  sliding_map_odo.T_vertex_this() = qdata.T_r_v_odo->inverse() * T_r_s;

  CLOG(DEBUG, static_name) << "Submap has " << sliding_map_odo.size()
                           << " voxels fused from the current scan";
}

void ScanToMapModule::run_(QueryCache &, OutputCache &, const Graph::Ptr &,
                                        const TaskExecutor::Ptr &) {
  // No-op: this module is not run automatically as part of a pipeline's
  // module list. The pipeline calls updateSubmap() directly, and only once
  // it already knows the result will be saved as a submap.
}

}  // namespace radar
}  // namespace vtr
