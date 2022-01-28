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
 * \file inter_exp_merging_module.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_lidar/modules/pointmap/dynamic_detection_module.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace lidar {

/**
 * \brief Uses ray-tracing to detect short-term dynamic objects.
 * Asynchronous. Optional.
 */
class InterExpMergingModule : public tactic::BaseModule {
 public:
  using Ptr = std::shared_ptr<InterExpMergingModule>;
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  static constexpr auto static_name = "lidar.inter_exp_merging";

  /** \brief Collection of config parameters */
  struct Config : public BaseModule::Config {
    using Ptr = std::shared_ptr<Config>;
    using ConstPtr = std::shared_ptr<const Config>;

    // dependency
    std::string dynamic_detection = DynamicDetectionModule::static_name;

    int depth = 0;

    float horizontal_resolution = 0.001;
    float vertical_resolution = 0.001;
    int max_num_observations = 20;

    int max_num_experiences = 128;

    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  InterExpMergingModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

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
  rclcpp::Publisher<PointCloudMsg>::SharedPtr old_map_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr new_map_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(InterExpMergingModule);
};

}  // namespace lidar
}  // namespace vtr