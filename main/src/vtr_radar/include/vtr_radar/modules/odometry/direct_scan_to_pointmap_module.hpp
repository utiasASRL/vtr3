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

#include "vtr_radar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace radar {

/**
 * \brief Turns the current keyframe's smoothed scan into the odometry submap.
 *
 * qdata.smoothed_scan (DRO's output, already blurred there so it matches
 * what localization's drl consumes) is resampled onto a voxel grid at
 * map_resolution and becomes the new submap, replacing whatever was there
 * before. updateSubmap() does the actual work and is meant to be called
 * directly (e.g. by the pipeline, once it knows a submap is actually
 * needed) rather than run every keyframe.
 */
class ScanToMapModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "radar.scan_to_pointmap";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    float map_resolution = 1.0;   // submap voxel size in meters

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

  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(ScanToMapModule);
};

}  // namespace radar
}  // namespace vtr
