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
 * \file keyframe_test_module_v2.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace lidar {

/** \brief Preprocesses raw pointcloud points and computes normals. */
class KeyframeTestModuleV2 : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.keyframe_test_v2";

  /** \brief Config parameters. */
  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    float max_translation = 10;
    float max_rotation = 30;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  KeyframeTestModuleV2(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(KeyframeTestModuleV2);
};

}  // namespace lidar
}  // namespace vtr