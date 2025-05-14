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

#include "vtr_lidar/cache.hpp"
#include "vtr_lidar/modules/modules.hpp"
#include "vtr_tactic/modules/modules.hpp"
#include "vtr_tactic/pipelines/base_pipeline.hpp"

namespace vtr {
namespace lidar {

class LidarPipeline : public tactic::BasePipeline {
 public:
  PTR_TYPEDEFS(LidarPipeline);

  /** \brief Static pipeline identifier. */
  static constexpr auto static_name = "lidar";

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

    bool save_raw_point_cloud = false;
    bool save_nn_point_cloud = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  LidarPipeline(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name);

  virtual ~LidarPipeline() {}

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

  /// odometry cached data
  /** \brief current sliding map for odometry */
  std::shared_ptr<PointMap<PointWithInfo>> sliding_map_odo_;
  /** \brief current timestamp*/
  std::shared_ptr<tactic::Timestamp> timestamp_odo_;
  /** \brief current pose and body velocity w.r.t the sliding map */
  std::shared_ptr<tactic::EdgeTransform> T_r_m_odo_;
  std::shared_ptr<Eigen::Matrix<double, 6, 1>> w_m_r_in_r_odo_;
  /** \brief vertex id of the last submap */
  tactic::VertexId submap_vid_odo_ = tactic::VertexId::Invalid();
  /** \brief transformation from latest submap vertex to robot */
  tactic::EdgeTransform T_sv_m_odo_ = tactic::EdgeTransform(true);
  
  // Prior stuff
  std::shared_ptr<lgmath::se3::Transformation> T_r_m_odo_prior_;
  std::shared_ptr<int64_t> timestamp_prior_;
  std::shared_ptr<Eigen::Matrix<double, 6, 1>> w_m_r_in_r_odo_prior_;
  std::shared_ptr<Eigen::Matrix<double, 12, 12>> cov_prior_;

  /// localization cached data
  /** \brief Current submap for localization */
  std::shared_ptr<const PointMap<PointWithInfo>> submap_loc_;

  VTR_REGISTER_PIPELINE_DEC_TYPE(LidarPipeline);
};

}  // namespace lidar
}  // namespace vtr
