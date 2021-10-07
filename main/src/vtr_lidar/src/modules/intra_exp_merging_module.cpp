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
 * \file intra_exp_merging_module.cpp
 * \brief IntraExpMergingModule class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/modules/intra_exp_merging_module.hpp>

namespace vtr {
namespace lidar {

using namespace tactic;

void IntraExpMergingModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->parameter = node->declare_parameter<std::string>(param_prefix + ".parameter", config_->parameter);
  // clang-format on
  CLOG(INFO, "tactic.module")
      << "Template module parameter set to: " << config_->parameter;
}

void IntraExpMergingModule::runImpl(QueryCache &qdata0,
                                    const Graph::ConstPtr &) {
  /// Pure virtual method that must be overriden.
  /// Do the actual work of your module. Load data from and store data to
  /// QueryCache.
  CLOG(INFO, "tactic.module") << "Running the template module...";
}

}  // namespace lidar
}  // namespace vtr