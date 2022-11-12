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
 * \file inter_exp_merging_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/pointmap/inter_exp_merging_module.hpp"

#include "vtr_lidar/data_types/multi_exp_pointmap.hpp"
#include "vtr_lidar/data_types/pointmap.hpp"
#include "vtr_tactic/modules/factory.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto InterExpMergingModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                            const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->dynamic_detection = node->declare_parameter<std::string>(param_prefix + ".dynamic_detection", config->dynamic_detection);

  config->depth = node->declare_parameter<int>(param_prefix + ".depth", config->depth);

  config->horizontal_resolution = node->declare_parameter<float>(param_prefix + ".horizontal_resolution", config->horizontal_resolution);
  config->vertical_resolution = node->declare_parameter<float>(param_prefix + ".vertical_resolution", config->vertical_resolution);
  config->max_num_observations = node->declare_parameter<int>(param_prefix + ".max_num_observations", config->max_num_observations);

  config->max_num_experiences = node->declare_parameter<int>(param_prefix + ".max_num_experiences", config->max_num_experiences);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void InterExpMergingModule::run_(QueryCache & /* qdata0 */,
                                 OutputCache & /* output0 */,
                                 const Graph::Ptr & /* graph */,
                                 const TaskExecutor::Ptr & /* executor */) {}

void InterExpMergingModule::runAsync_(QueryCache & /* qdata0 */,
                                      OutputCache & /* output0 */,
                                      const Graph::Ptr & /* graph */,
                                      const TaskExecutor::Ptr & /* executor */,
                                      const Task::Priority & /* priority */,
                                      const Task::DepId & /* dep_id */) {}
}  // namespace lidar
}  // namespace vtr