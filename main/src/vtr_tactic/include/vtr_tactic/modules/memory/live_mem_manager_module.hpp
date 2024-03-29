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
 * \file live_mem_manager_module.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"  /// include this header if using the task queue

namespace vtr {
namespace tactic {

/** \brief A generic module that manages memory usage of the live run */
class LiveMemManagerModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "live_mem_manager";

  /** \brief Collection of config parameters */
  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    int window_size = 10;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &,
                            const std::string &);
  };

  LiveMemManagerModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : BaseModule{module_factory, name}, config_(config) {}

 private:
  void run_(QueryCache &, OutputCache &, const Graph::Ptr &,
            const TaskExecutor::Ptr &) override;

  void runAsync_(QueryCache &, OutputCache &, const Graph::Ptr &,
                 const TaskExecutor::Ptr &, const Task::Priority &,
                 const Task::DepId &) override;

  /** \brief Module configuration. */
  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(LiveMemManagerModule);
};

}  // namespace tactic
}  // namespace vtr