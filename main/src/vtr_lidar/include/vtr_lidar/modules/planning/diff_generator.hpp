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
 * \file diff_generator.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "tf2/convert.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace lidar {

class DifferenceDetector : public tactic::BaseModule {
 public:
  PTR_TYPEDEFS(DifferenceDetector);
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  static constexpr auto static_name = "lidar.diff_generator";

  /** \brief Collection of config parameters */
  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // change detection
    float detection_range = 10.0; //m
    float minimum_distance = 0.0; //m

    float neighbour_threshold = 0.05; //m
    float voxel_size = 0.2; //m

    float angle_weight = 10.0/2/M_PI;

    //
    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  DifferenceDetector(
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
  rclcpp::Publisher<PointCloudMsg>::SharedPtr diffpcd_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(DifferenceDetector);

};

}  // namespace lidar
}  // namespace vtr
