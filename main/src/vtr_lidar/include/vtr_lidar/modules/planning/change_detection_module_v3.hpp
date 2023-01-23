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
 * \file change_detection_module_v3.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "tf2/convert.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace lidar {

class ChangeDetectionModuleV3 : public tactic::BaseModule {
 public:
  PTR_TYPEDEFS(ChangeDetectionModuleV3);
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;

  static constexpr auto static_name = "lidar.change_detection_v3";

  /** \brief Collection of config parameters */
  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // change detection
    float detection_range = 10.0;
    float search_radius = 1.0;
    float negprob_threshold = 1.0;

    bool use_prior = false;
    float alpha0 = 1.0;
    float beta0 = 0.1;

    bool use_support_filtering = false;
    float support_radius = 0.25;
    float support_variance = 0.1;
    float support_threshold = 0.0;

    // cost map
    int costmap_history_size = 10;
    float resolution = 1.0;
    float size_x = 20.0;
    float size_y = 20.0;
    float influence_distance = 1.0;
    float minimum_distance = 0.5;

    //
    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  ChangeDetectionModuleV3(
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
  rclcpp::Publisher<OccupancyGridMsg>::SharedPtr costmap_pub_;
  rclcpp::Publisher<OccupancyGridMsg>::SharedPtr filtered_costmap_pub_ ;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr costpcd_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(ChangeDetectionModuleV3);


  // Modificatons for Temporal costmap filter
  std::vector<std::unordered_map<std::pair<float, float>, float>> costmap_history;
};

}  // namespace lidar
}  // namespace vtr
