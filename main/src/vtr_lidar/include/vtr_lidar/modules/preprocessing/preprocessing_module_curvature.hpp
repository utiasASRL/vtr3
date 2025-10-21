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
 * \file preprocessing_module_curvature.hpp
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace lidar {

/** \brief Preprocess raw pointcloud points and compute normals */
class PreprocessingCurvatureModule : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.preprocessing_curvature";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    int num_threads = 1;

    float crop_range = 10.0;
    int num_sample = 10000;
    int k_neighbors = 30;

    bool downsample_teach_map = false;
    float plane_voxel_size = 1.0;
    float feature_voxel_size = 0.3;
    
    float t = 4.0;
    float d_prime = 2.0;
    float ground_plane_threshold = 0.8;
    int min_points_threshold = 0;

    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  PreprocessingCurvatureModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

 protected:
  void compute_curvature(pcl::PointCloud<PointWithInfo>::Ptr& cloud);
  void cluster_curvature(const pcl::PointCloud<PointWithInfo>::Ptr& cloud, std::vector<int>& cluster_ids);
  void remove_small_clusters(pcl::PointCloud<PointWithInfo>::Ptr& cloud, std::vector<int>& cluster_ids);


 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr filtered_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(PreprocessingCurvatureModule);
};

}  // namespace lidar
}  // namespace vtr