// Copyright 2023, Autonomous Space Robotics Lab (ASRL)
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
 * \file queued_inflation_module.hpp
 * \author Alec Krawciw, Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <list>

#include "tf2/convert.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_lidar/modules/planning/costmap_inflation_module.hpp"

namespace vtr {
namespace lidar {

class QueuedCostmapModule : public CostmapInflationModule {
 public:
  PTR_TYPEDEFS(QueuedCostmapModule);
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;
  using VtrPointCloud = pcl::PointCloud<PointWithInfo>;

  static constexpr auto static_name = "lidar.costmap_queue";

  /** \brief Collection of config parameters */
  struct Config : public CostmapInflationModule::Config {
    PTR_TYPEDEFS(Config);

    // cost map
    unsigned int costmap_history_size = 10;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  QueuedCostmapModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : CostmapInflationModule{config, module_factory, name}, config_(config) {}



 private:
  VtrPointCloud assemble_pointcloud(tactic::QueryCache &qdata, 
              tactic::OutputCache &output, const tactic::Graph::Ptr &graph) override;

  Config::ConstPtr config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr concat_pc_pub_;


  std::list<std::pair<unsigned, VtrPointCloud>> detected_history;


  VTR_REGISTER_MODULE_DEC_TYPE(QueuedCostmapModule);
};

}  // namespace lidar
}  // namespace vtr