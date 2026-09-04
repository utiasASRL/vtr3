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

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_radar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace radar {

/**
 * \brief Turns the current keyframe's smoothed scan into the odometry submap.
 *
 * This is controlled by direct_odometry and is not run on every keyframe
 * since we only care to construct submap when we want to save one.
 */
class ScanToMapModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "radar.scan_to_pointmap";

  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    float map_resolution = 1.0;   // submap voxel size in meters

    // Possible additional blurring on top of DRO's fixed gauss_blur_sigma blur for mapping
    bool adaptive_blur = false;   // repeatedly re-blur until min_percent_nonzero of the scan is non-zero
    double blur_sigma_step = 3.0; // fixed sigma applied on each re-blur pass
    int max_num_reblur = 5;       // cap on the number of re-blur passes
    double min_int_val_tol = 0.5; // min intensity value to consider a pixel non-zero
    double min_percent_nonzero = 0.3; // percent of pixels above min_int_val_tol to stop blurring

    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  ScanToMapModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

  /** \brief (Re)builds qdata.sliding_map_odo from qdata.smoothed_scan. */
  void updateSubmap(RadarQueryCache &qdata) const;

 private:
  /** \brief No-op: required by BaseModule, but this module is invoked
   *  directly via updateSubmap() rather than run as part of a pipeline's
   *  module list. */
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  /** \brief Rebuilds the map from the given (already blurred) scan. */
  void rebuildMap(PointMap<PointWithInfo> &map, const cv::Mat &scan,
                  double scan_res, tactic::Timestamp stamp) const;

  /** \brief If scan doesn't already have enough non-zero coverage, repeatedly
   *  re-blurs it with blur_sigma_step (compounding, up to max_num_reblur
   *  passes) until it does. Returns scan unchanged if it already satisfies
   *  min_percent_nonzero. */
  cv::Mat adaptiveBlur(const cv::Mat &scan) const;

  Config::ConstPtr config_;

  mutable bool publisher_initialized_ = false;
  mutable rclcpp::Publisher<PointCloudMsg>::SharedPtr map_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(ScanToMapModule);
};

}  // namespace radar
}  // namespace vtr
