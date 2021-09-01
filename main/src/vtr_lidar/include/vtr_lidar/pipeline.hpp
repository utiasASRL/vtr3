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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vtr_messages/msg/movability.hpp>
#include <vtr_messages/msg/point_map.hpp>

namespace vtr {
namespace lidar {

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointXYZMsg = vtr_messages::msg::PointXYZ;
using MovabilityMsg = vtr_messages::msg::Movability;
using PointMapMsg = vtr_messages::msg::PointMap;

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

    float map_voxel_size = 0.2;
  };

  LidarPipeline(const std::string &name = static_name)
      : tactic::BasePipeline{name} {
    addModules();
  }

  virtual ~LidarPipeline() {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string &param_prefix) override;

  void initialize(const tactic::Graph::Ptr &graph) override;

  void preprocess(tactic::QueryCache::Ptr &qdata,
                  const tactic::Graph::Ptr &graph) override;
  void visualizePreprocess(tactic::QueryCache::Ptr &qdata,
                           const tactic::Graph::Ptr &graph) override;

  void runOdometry(tactic::QueryCache::Ptr &qdata,
                   const tactic::Graph::Ptr &graph) override;
  void visualizeOdometry(tactic::QueryCache::Ptr &qdata,
                         const tactic::Graph::Ptr &graph) override;

  void runLocalization(tactic::QueryCache::Ptr &qdata,
                       const tactic::Graph::Ptr &graph) override;
  void visualizeLocalization(tactic::QueryCache::Ptr &qdata,
                             const tactic::Graph::Ptr &graph) override;

  void processKeyframe(tactic::QueryCache::Ptr &qdata,
                       const tactic::Graph::Ptr &graph,
                       tactic::VertexId live_id) override;

  void wait() override;

  void reset() override;

 private:
  void addModules();

  void setOdometryPrior(LidarQueryCache::Ptr &qdata,
                        const tactic::Graph::Ptr &graph);

  void savePointcloudMap(LidarQueryCache::Ptr qdata,
                         const tactic::Graph::Ptr graph,
                         tactic::VertexId live_id);

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

  /** \brief Current map being built */
  std::shared_ptr<IncrementalPointMap> new_map_;

  /** \brief Current map and its vertex for odometry */
  std::shared_ptr<IncrementalPointMap> odo_map_;
  std::shared_ptr<tactic::VertexId> odo_map_vid_;
  std::shared_ptr<lgmath::se3::TransformationWithCovariance> odo_map_T_v_m_;

  /** \brief Current map and tis vertex for localization */
  std::shared_ptr<MultiExpPointMap> loc_map_;
  std::shared_ptr<tactic::VertexId> loc_map_vid_;

  std::mutex map_saving_mutex_;
  std::future<void> map_saving_thread_future_;

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
