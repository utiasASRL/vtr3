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
 * \file odometry_map_maintenance_module.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace lidar {

/** \brief */
class OdometryMapMaintenanceModule : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.odometry_map_maintenance";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    float map_voxel_size = 0.2;

    float point_life_time = -1.0;  // negative means infinite life time

    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  OdometryMapMaintenanceModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr scan_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr map_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(OdometryMapMaintenanceModule);
};

}  // namespace lidar
}  // namespace vtr