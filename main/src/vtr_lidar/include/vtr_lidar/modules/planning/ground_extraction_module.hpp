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
 * \file ground_extraction_module.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "tf2/convert.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_lidar/segmentation/himmelsbach.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace lidar {

class GroundExtractionModule : public tactic::BaseModule {
 public:
  using Ptr = std::shared_ptr<GroundExtractionModule>;
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;

  static constexpr auto static_name = "lidar.ground_extraction";

  /** \brief Collection of config parameters */
  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // himmelsbach
    float z_offset = 0.0f;

    float alpha = 2.0 * M_PI / 180.0;
    float tolerance = 0.25;
    float Tm = 0.4;
    float Tm_small = 0.2;
    float Tb = 0.8;
    float Trmse = 0.1;
    float Tdprev = 1.0;

    float rmin = 3.0;
    size_t num_bins_small = 30;
    float bin_size_small = 3.0;
    size_t num_bins_large = 30;
    float bin_size_large = 3.0;

    // ogm
    float resolution = 1.0;
    float size_x = 20.0;
    float size_y = 20.0;

    bool run_async = false;
    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  GroundExtractionModule(
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

  /** \brief */
  Himmelsbach<PointWithInfo> himmelsbach_;

  /** \brief mutex to make publisher thread safe */
  std::mutex mutex_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr map_pub_;
  rclcpp::Publisher<OccupancyGridMsg>::SharedPtr ogm_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(GroundExtractionModule);
};

}  // namespace lidar
}  // namespace vtr
