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
 * \file map_maintenance_module.hpp
 * \brief MapMaintenanceModule class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <pcl_conversions/pcl_conversions.h>

#include <vtr_lidar/cache.hpp>
#include <vtr_lidar/pointmap/pointmap.hpp>
#include <vtr_lidar/ray_tracing.hpp>
#include <vtr_tactic/modules/base_module.hpp>

// visualization
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>

namespace vtr {
namespace lidar {

/** \brief */
class MapMaintenanceModule : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  // using MarkerMsg = visualization_msgs::msg::Marker;
  // using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.map_maintenance";

  /** \brief Config parameters. */
  struct Config {
    float map_voxel_size = 0.2;
    // dynamic objects remocal
    float horizontal_resolution = 0.001;
    float vertical_resolution = 0.001;
    int min_num_observations = 0;
    int max_num_observations = 20;

    bool visualize = false;
  };

  MapMaintenanceModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(tactic::QueryCache &qdata,
               const tactic::Graph::ConstPtr &graph) override;

  void visualizeImpl(tactic::QueryCache &,
                     const tactic::Graph::ConstPtr &) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr aligned_points_pub_;
  // rclcpp::Publisher<MarkerArrayMsg>::SharedPtr aligned_normals_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr movability_map_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr movability_obs_map_pub_;
};

}  // namespace lidar
}  // namespace vtr