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
 * \brief LidarPipeline class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/cache.hpp"
#include "vtr_lidar/modules/modules.hpp"
#include "vtr_tactic/modules/modules.hpp"
#include "vtr_tactic/pipelines/base_pipeline.hpp"

namespace vtr {
namespace lidar {

class LidarPipeline : public tactic::BasePipeline {
 public:
  using Ptr = std::shared_ptr<LidarPipeline>;

  /** \brief Static pipeline identifier. */
  static constexpr auto static_name = "lidar";

  /** \brief Collection of config parameters */
  struct Config : public BasePipeline::Config {
    using Ptr = std::shared_ptr<Config>;
    using ConstPtr = std::shared_ptr<const Config>;

    std::vector<std::string> preprocessing;
    std::vector<std::string> odometry;
    std::vector<std::string> localization;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  LidarPipeline(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactoryV2> &module_factory = nullptr,
      const std::string &name = static_name);

  virtual ~LidarPipeline() {}

  tactic::OutputCache::Ptr createOutputCache() const override {
    return std::make_shared<LidarOutputCache>();
  }

  void preprocess(
      const tactic::QueryCache::Ptr &qdata,
      const tactic::OutputCache::Ptr &output, const tactic::Graph::Ptr &graph,
      const std::shared_ptr<tactic::TaskExecutor> &executor) override;

  void runOdometry(
      const tactic::QueryCache::Ptr &qdata,
      const tactic::OutputCache::Ptr &output, const tactic::Graph::Ptr &graph,
      const std::shared_ptr<tactic::TaskExecutor> &executor) override;

  void runLocalization(
      const tactic::QueryCache::Ptr &qdata,
      const tactic::OutputCache::Ptr &output, const tactic::Graph::Ptr &graph,
      const std::shared_ptr<tactic::TaskExecutor> &executor) override;

  void processKeyframe(
      const tactic::QueryCache::Ptr &qdata,
      const tactic::OutputCache::Ptr &output, const tactic::Graph::Ptr &graph,
      const std::shared_ptr<tactic::TaskExecutor> &executor) override;

  void wait() override;

  void reset() override;

 private:
  void setOdometryPrior(const LidarQueryCache::Ptr &qdata,
                        const tactic::Graph::Ptr &graph);

 private:
  /** \brief Pipeline configuration */
  Config::ConstPtr config_;

  std::vector<tactic::BaseModule::Ptr> preprocessing_;
  std::vector<tactic::BaseModule::Ptr> odometry_;
  std::vector<tactic::BaseModule::Ptr> localization_;

  /**
   * \brief A candidate cache in case for odometry failure, where the candidate
   * cache is used to create a keyframe.
   */
  LidarQueryCache::Ptr candidate_qdata_ = nullptr;

  /** \brief lidar scans that will be stored to the next vertex */
  std::map<tactic::Timestamp, std::shared_ptr<PointScan<PointWithInfo>>>
      new_scan_odo_;
#if false  /// store raw point cloud
  std::map<tactic::Timestamp, std::shared_ptr<PointScan<PointWithInfo>>>
      new_raw_scan_odo_;
#endif

  /** \brief Current map being built */
  std::shared_ptr<PointMap<PointWithInfo>> new_map_odo_;

  /** \brief Current map for odometry */
  std::shared_ptr<PointMap<PointWithInfo>> curr_map_odo_;

  /** \brief Current map for localization */
  std::shared_ptr<const PointMap<PointWithInfo>> curr_map_loc_;

  /**
   * \brief a trjacetory to estimate transform at a future time
   * \note no need to use a lock since this variable is only used in odometry to
   * get a better T_r_m prior.
   */
  std::shared_ptr<const steam::se3::SteamTrajInterface> trajectory_;
  /** \brief the time at which the trajectory was estimated */
  common::timing::time_point trajectory_time_point_;

  VTR_REGISTER_PIPELINE_DEC_TYPE(LidarPipeline);
};

}  // namespace lidar
}  // namespace vtr
