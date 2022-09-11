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
 * \file fake_obstacle_module.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

#include "vtr_lidar/mesh2pcd/mesh2pcd.hpp"

namespace vtr {
namespace lidar {

class FakeObstacleModule : public tactic::BaseModule {
 public:
  PTR_TYPEDEFS(FakeObstacleModule);
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  static constexpr auto static_name = "lidar.fake_obstacle";

  /** \brief Collection of config parameters */
  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    std::string path;
    std::vector<std::string> objs;

    std::vector<long int> fixed_types;
    std::vector<double> fixed_xs;
    std::vector<double> fixed_ys;
    std::vector<double> fixed_zs;
    std::vector<double> fixed_rolls;
    std::vector<double> fixed_pitchs;
    std::vector<double> fixed_yaws;

    // general
    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  FakeObstacleModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name);

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  std::vector<std::pair<size_t, Eigen::Matrix4f>> obj_T_scan_objs_;
  std::vector<mesh2pcd::Mesh2PcdConverter> converters_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr pointcloud_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(FakeObstacleModule);
};

}  // namespace lidar
}  // namespace vtr