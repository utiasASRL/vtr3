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
 * \file dynamic_detection_module.hpp
 * \brief DynamicDetectionModule class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace lidar {

/**
 * \brief Uses ray-tracing to detect short-term dynamic objects.
 * Asynchronous. Optional.
 */
class DynamicDetectionModule : public tactic::BaseModule {
 public:
  static constexpr auto static_name = "lidar.dynamic_detection";

  /** \brief Collection of config parameters */
  struct Config {
    std::string parameter = "default value";
  };

  DynamicDetectionModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(tactic::QueryCache &, const tactic::Graph::ConstPtr &) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;  /// \todo no need to be a shared pointer.
};

}  // namespace lidar
}  // namespace vtr