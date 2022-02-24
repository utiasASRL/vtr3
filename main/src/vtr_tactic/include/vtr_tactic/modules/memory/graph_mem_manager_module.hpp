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
 * \file graph_mem_manager_module.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"  /// include this header if using the task queue

namespace vtr {
namespace tactic {

class GraphMemManagerModule : public BaseModule {
 public:
  static constexpr auto static_name = "graph_mem_manager";

  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    int vertex_life_span = 10;
    int window_size = 5;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &,
                            const std::string &);
  };

  GraphMemManagerModule(
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

  /** \brief mutex to protect access to life map */
  std::mutex vid_life_map_mutex_;

  /** \brief Maps vertex ids to life spans. */
  std::unordered_map<VertexId, int> vid_life_map_;

  VertexId last_vid_ = VertexId::Invalid();

  /** \brief Module configuration. */
  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(GraphMemManagerModule);
};

}  // namespace tactic
}  // namespace vtr