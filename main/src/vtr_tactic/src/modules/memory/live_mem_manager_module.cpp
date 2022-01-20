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
 * \file live_mem_manager_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_tactic/modules/memory/live_mem_manager_module.hpp"

namespace vtr {
namespace tactic {

auto LiveMemManagerModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                           const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->window_size = node->declare_parameter<int>(param_prefix + ".window_size", config->window_size);
  // clang-format on
  return config;
}

void LiveMemManagerModule::run_(QueryCache &qdata, OutputCache &,
                                const Graph::Ptr &,
                                const TaskExecutor::Ptr &executor) {
  if (qdata.live_id->isValid() &&
      qdata.live_id->minorId() >= (unsigned)config_->window_size &&
      *qdata.keyframe_test_result == KeyframeTestResult::CREATE_VERTEX) {
    const auto vid_to_unload =
        VertexId(qdata.live_id->majorId(),
                 qdata.live_id->minorId() - (unsigned)config_->window_size);
    qdata.live_mem_async.emplace(vid_to_unload);

    executor->dispatch(std::make_shared<Task>(
        shared_from_this(), qdata.shared_from_this(), 0, Task::DepIdSet{},
        Task::DepId{}, "Live Mem Manager", vid_to_unload));
  }
}

void LiveMemManagerModule::runAsync_(QueryCache &qdata, OutputCache &,
                                     const Graph::Ptr &graph,
                                     const TaskExecutor::Ptr &,
                                     const Task::Priority &,
                                     const Task::DepId &) {
  auto vertex = graph->at(*qdata.live_mem_async);
  CLOG(DEBUG, "tactic.module.live_mem_manager")
      << "Saving and unloading data associated with vertex: " << *vertex;
  vertex->unload();
}

}  // namespace tactic
}  // namespace vtr