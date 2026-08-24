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
 * \file direct_scan_to_pointmap.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <deque>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_radar/cache.hpp"
#include "vtr_radar/dr_ba/utils/ba_config.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace radar {

/**
 * \brief Fuses a sliding window of DRO local maps into the odometry submap.
 *
 * Mirrors the offline dr_ba mapper: voxels are allocated uniformly at
 * map_resolution around every window pose and each one is the inverse-variance
 * weighted mean of the window scans that cover it. The window is anchored to
 * the sensor frame of the newest keyframe, so the map is rebuilt from scratch
 * on every keyframe rather than incrementally added to and subtracted from.
 */
class ScanToMapModule : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "radar.scan_to_pointmap";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    float map_resolution = 1.0;   // submap voxel size in meters

    // radius each scan is cropped to and contributes over
    float scan_max_range = 100.0;

    // scans farther than this from the submap center are dropped
    float window_max_dist = 30.0;

    // scan blurring, mirrors the dr_ba offline mapper
    double gauss_blur_sigma = 3.0;
    bool adaptive_blur = false;
    double max_blur_sigma = 15.0;
    double min_int_val_tol = 0.5;
    double min_percent_nonzero = 0.3;

    // meas_std and range_factor set the inverse-variance weights of the fusion
    ba::OptimizationOptions ba_opts;

    int num_threads = 8;
    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  ScanToMapModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

  void reset() override { window_.clear(); }

 private:
  /** \brief One window keyframe, kept 8-bit as the reference implementations do. */
  struct WindowEntry {
    tactic::Timestamp stamp;
    cv::Mat blurred;                     // CV_8U, blurred, renormalized and cropped
    lgmath::se3::Transformation T_a_s;   // anchor (newest keyframe sensor frame) <- sensor
    double res;                          // m/pixel of blurred, from its producer
  };

  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  /** \brief Blurs, renormalizes and quantizes a DRO local map for storage. */
  cv::Mat blurScan(const cv::Mat &scan) const;

  /** \brief Crops a scan of the given m/pixel to scan_max_range about its center. */
  cv::Mat cropScan(const cv::Mat &scan, double res) const;

  /** \brief Drops window scans too far from the submap center. */
  void trimWindow();

  /** \brief Initializes the voxel keys to fuse into, covering every window pose. */
  std::vector<pointmap::VoxKey> initVoxelKeys() const;

  /** \brief Rebuilds the map from the current window. */
  void rebuildMap(PointMap<PointWithInfo> &map) const;

  Config::ConstPtr config_;

  std::deque<WindowEntry> window_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr map_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(ScanToMapModule);
};

}  // namespace radar
}  // namespace vtr
