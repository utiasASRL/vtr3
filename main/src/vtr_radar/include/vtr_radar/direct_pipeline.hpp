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
 * \file pipeline.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_radar/cache.hpp"
#include "vtr_radar/modules/modules.hpp"
#include "vtr_tactic/modules/modules.hpp"
#include "vtr_tactic/pipelines/base_pipeline.hpp"

// need to include radar b_scan_msg
#include "navtech_msgs/msg/radar_b_scan_msg.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"


namespace vtr {
namespace radar {

class RadarDirectPipeline : public tactic::BasePipeline {
 public:
  PTR_TYPEDEFS(RadarDirectPipeline);
  using ImageMsg = sensor_msgs::msg::Image;

  /** \brief Static pipeline identifier. */
  static constexpr auto static_name = "direct_radar";

  /** \brief Collection of config parameters */
  struct Config : public BasePipeline::Config {
    PTR_TYPEDEFS(Config);
    // modules
    std::vector<std::string> preprocessing;
    std::vector<std::string> odometry;
    std::vector<std::string> localization;
    // submap creation thresholds
    double submap_translation_threshold = 0.0;  // in meters
    double submap_rotation_threshold = 0.0;     // in degrees

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  RadarDirectPipeline(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name);

  virtual ~RadarDirectPipeline() {}

  tactic::OutputCache::Ptr createOutputCache() const override;

  void reset() override;

  void preprocess_(
      const tactic::QueryCache::Ptr &qdata,
      const tactic::OutputCache::Ptr &output, const tactic::Graph::Ptr &graph,
      const std::shared_ptr<tactic::TaskExecutor> &executor) override;

  void runOdometry_(
      const tactic::QueryCache::Ptr &qdata,
      const tactic::OutputCache::Ptr &output, const tactic::Graph::Ptr &graph,
      const std::shared_ptr<tactic::TaskExecutor> &executor) override;

  void runLocalization_(
      const tactic::QueryCache::Ptr &qdata,
      const tactic::OutputCache::Ptr &output, const tactic::Graph::Ptr &graph,
      const std::shared_ptr<tactic::TaskExecutor> &executor) override;

  void onVertexCreation_(
      const tactic::QueryCache::Ptr &qdata,
      const tactic::OutputCache::Ptr &output, const tactic::Graph::Ptr &graph,
      const std::shared_ptr<tactic::TaskExecutor> &executor) override;

 private:
  /** \brief Pipeline configuration */
  Config::ConstPtr config_;

  std::vector<tactic::BaseModule::Ptr> preprocessing_;
  std::vector<tactic::BaseModule::Ptr> odometry_;
  std::vector<tactic::BaseModule::Ptr> localization_;

  /// Builds the odometry submap; called directly by onVertexCreation_ only
  /// once a submap is actually needed, rather than run every keyframe as
  /// part of odometry_.
  std::shared_ptr<ScanToMapModule> scan_to_pointmap_;

  tactic::EdgeTransform T_v_odo_submap_v_ = tactic::EdgeTransform(true);
  tactic::VertexId submap_vid_odo_ = tactic::VertexId::Invalid();

  std::shared_ptr<const PointMap<PointWithInfo>> submap_loc_;

  VTR_REGISTER_PIPELINE_DEC_TYPE(RadarDirectPipeline);
};

}  // namespace radar
}  // namespace vtr
