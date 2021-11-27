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
 * \brief OdometryMapMergingModule class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace vtr {
namespace lidar {

/** \brief */
class OdometryMapMergingModule : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.odometry_map_merging";

  /** \brief Config parameters. */
  struct Config {
    float map_voxel_size = 0.2;

    bool visualize = false;
  };

  OdometryMapMergingModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(tactic::QueryCache &qdata, const tactic::Graph::Ptr &graph,
               const tactic::TaskExecutor::Ptr &executor) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr map_pub_;
};

}  // namespace lidar
}  // namespace vtr