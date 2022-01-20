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
 * \file obstacle_detection_module.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "tf2/convert.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_broadcaster.h"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace lidar {

class ObstacleDetectionModule : public tactic::BaseModule {
 public:
  using Ptr = std::shared_ptr<ObstacleDetectionModule>;
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;

  static constexpr auto static_name = "lidar.obstacle_detection";

  /** \brief Collection of config parameters */
  struct Config : public BaseModule::Config {
    using Ptr = std::shared_ptr<Config>;
    using ConstPtr = std::shared_ptr<const Config>;
    // points above this height is considered obstacle
    float z_min = 0.5f;
    float z_max = 2.0f;

    // ogm
    float resolution = 1.0;
    float size_x = 20.0;
    float size_y = 20.0;

    bool run_async = false;
    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  ObstacleDetectionModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name);

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  void runAsync_(tactic::QueryCache &qdata, tactic::OutputCache &output,
                 const tactic::Graph::Ptr &graph,
                 const tactic::TaskExecutor::Ptr &executor,
                 const tactic::Task::Priority &priority,
                 const tactic::Task::DepId &dep_id) override;

  Config::ConstPtr config_;

  /** \brief mutex to make publisher thread safe */
  std::mutex mutex_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr map_pub_;
  rclcpp::Publisher<OccupancyGridMsg>::SharedPtr ogm_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(ObstacleDetectionModule);
};

}  // namespace lidar
}  // namespace vtr