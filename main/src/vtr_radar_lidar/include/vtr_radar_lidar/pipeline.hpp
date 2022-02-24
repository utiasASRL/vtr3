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
 * \file pipeline.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/modules/modules.hpp"
#include "vtr_radar/modules/modules.hpp"
#include "vtr_radar_lidar/cache.hpp"
#include "vtr_radar_lidar/modules/modules.hpp"
#include "vtr_tactic/modules/modules.hpp"
#include "vtr_tactic/pipelines/base_pipeline.hpp"

namespace vtr {
namespace radar_lidar {

class RadarLidarPipeline : public tactic::BasePipeline {
 public:
  PTR_TYPEDEFS(RadarLidarPipeline);

  /** \brief Static pipeline identifier. */
  static constexpr auto static_name = "radar_lidar";

  /** \brief Collection of config parameters */
  struct Config : public BasePipeline::Config {
    PTR_TYPEDEFS(Config);

    std::vector<std::string> preprocessing;
    std::vector<std::string> odometry;
    std::vector<std::string> localization;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  RadarLidarPipeline(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name);

  virtual ~RadarLidarPipeline() {}

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

  void processKeyframe_(
      const tactic::QueryCache::Ptr &qdata,
      const tactic::OutputCache::Ptr &output, const tactic::Graph::Ptr &graph,
      const std::shared_ptr<tactic::TaskExecutor> &executor) override;

 private:
  /** \brief Pipeline configuration */
  Config::ConstPtr config_;

  std::vector<tactic::BaseModule::Ptr> preprocessing_;
  std::vector<tactic::BaseModule::Ptr> odometry_;
  std::vector<tactic::BaseModule::Ptr> localization_;

  /// odometry cached data
  /** \brief Latest point cloud and BEV image for MCRANSAC */
  std::shared_ptr<const cv::Mat> cartesian_odo_;
  std::shared_ptr<const pcl::PointCloud<radar::PointWithInfo>> point_cloud_odo_;
  /** \brief Current point map for odometry */
  std::shared_ptr<radar::PointMap<radar::PointWithInfo>> point_map_odo_;
  /** \brief Current timestamp, pose and velocity */
  std::shared_ptr<tactic::Timestamp> timestamp_odo_;
  std::shared_ptr<tactic::EdgeTransform> T_r_m_odo_;
  std::shared_ptr<Eigen::Matrix<double, 6, 1>> w_m_r_in_r_odo_;
  /** \brief lidar scans that will be stored to the next vertex */
  std::map<tactic::Timestamp,
           std::shared_ptr<radar::PointScan<radar::PointWithInfo>>>
      new_scan_odo_;
#if false  /// store raw point cloud
  std::map<tactic::Timestamp,
           std::shared_ptr<radar::PointScan<radar::PointWithInfo>>>
      new_raw_scan_odo_;
#endif

  /// localization cached data
  /** \brief Current map for localization */
  std::shared_ptr<const lidar::PointMap<lidar::PointWithInfo>> curr_map_loc_;

  VTR_REGISTER_PIPELINE_DEC_TYPE(RadarLidarPipeline);
};

}  // namespace radar_lidar
}  // namespace vtr
