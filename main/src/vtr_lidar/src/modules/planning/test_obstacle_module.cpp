// Copyright 2024, Autonomous Space Robotics Lab (ASRL)
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
 * \file test_obstacle_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/modules/planning/test_obstacle_module.hpp>
#include "vtr_lidar/data_types/costmap.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto TestObstacleModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  // clang-format on
  return std::make_shared<TestObstacleModule::Config>();
}


void TestObstacleModule::run_(QueryCache &qdata0, OutputCache &,
                            const Graph::Ptr &graph, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  auto point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(200, 1);

  for (size_t i = 0; i < point_cloud->size(); ++i) {
    point_cloud->at(i).x = 3.0 + (rand() % 100) / 100.0;
    point_cloud->at(i).y = 0.0 + (rand() % 100) / 100.0;
    point_cloud->at(i).z = 1.0 + (rand() % 100) / 100.0;
  }

  qdata.changed_points.emplace(*point_cloud);               
}

}  // namespace lidar
}  // namespace vtr