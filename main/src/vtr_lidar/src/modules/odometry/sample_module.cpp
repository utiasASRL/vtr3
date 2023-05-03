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
 * \file torch_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/modules/odometry/sample_module.hpp>


namespace vtr {
namespace lidar {

using namespace tactic;

auto TestNNModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<TestNNModule::Config>();
  auto base_config = std::static_pointer_cast<TorchModule::Config>(config);
  *base_config =  *nn::TorchModule::Config::fromROS(node, param_prefix);
  // clang-format off
  // clang-format on
  return config;
}


void TestNNModule::run_(QueryCache &qdata0, OutputCache &,
                            const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  std::vector<float> inputs {2, 5};
  evaluateModel<float>(inputs, {1, 2});
                            
}

}  // namespace nn
}  // namespace vtr