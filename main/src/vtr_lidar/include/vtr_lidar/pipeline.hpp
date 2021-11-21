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

#include <vtr_lidar/cache.hpp>
#include <vtr_lidar/modules/modules.hpp>
#include <vtr_logging/logging.hpp>
#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_tactic/modules/module_factory.hpp>
#include <vtr_tactic/pipelines/base_pipeline.hpp>

namespace vtr {
namespace lidar {

class LidarPipeline : public tactic::BasePipeline {
 public:
  using Ptr = std::shared_ptr<LidarPipeline>;

  /** \brief Static pipeline identifier. */
  static constexpr auto static_name = "lidar";

  /** \brief Collection of config parameters */
  struct Config {
    std::vector<std::string> preprocessing;
    std::vector<std::string> odometry;
    std::vector<std::string> localization;
  };

  LidarPipeline(const std::string &name = static_name)
      : tactic::BasePipeline{name} {
    addModules();
  }

  virtual ~LidarPipeline() {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string &param_prefix) override;

  void initialize(const tactic::Graph::Ptr &graph) override;

  void preprocess(const tactic::QueryCache::Ptr &qdata,
                  const tactic::Graph::Ptr &graph) override;
  void visualizePreprocess(const tactic::QueryCache::Ptr &qdata,
                           const tactic::Graph::Ptr &graph) override;

  void runOdometry(const tactic::QueryCache::Ptr &qdata,
                   const tactic::Graph::Ptr &graph) override;
  void visualizeOdometry(const tactic::QueryCache::Ptr &qdata,
                         const tactic::Graph::Ptr &graph) override;

  void runLocalization(const tactic::QueryCache::Ptr &qdata,
                       const tactic::Graph::Ptr &graph) override;
  void visualizeLocalization(const tactic::QueryCache::Ptr &qdata,
                             const tactic::Graph::Ptr &graph) override;

  void processKeyframe(const tactic::QueryCache::Ptr &qdata,
                       const tactic::Graph::Ptr &graph,
                       tactic::VertexId live_id) override;

  void wait() override;

  void reset() override;

 private:
  void addModules();

  void setOdometryPrior(const LidarQueryCache::Ptr &qdata,
                        const tactic::Graph::Ptr &graph);

 private:
  /** \brief Pipeline configuration */
  std::shared_ptr<Config> config_ = std::make_shared<Config>();

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
  std::shared_ptr<PointMap<PointWithInfo>> curr_map_loc_;

  /**
   * \brief a trjacetory to estimate transform at a future time
   * \note no need to use a lock since this variable is only used in odometry to
   * get a better T_r_m prior.
   */
  std::shared_ptr<steam::se3::SteamTrajInterface> trajectory_;
  /** \brief the time at which the trajectory was estimated */
  common::timing::time_point trajectory_time_point_;
};

}  // namespace lidar
}  // namespace vtr
