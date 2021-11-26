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
 * \brief LiveMemManagerModule class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_tactic/modules/memory/live_mem_manager_module.hpp>

namespace vtr {
namespace tactic {

using namespace std::literals::chrono_literals;

class LiveMemManagerModule::Task : public Task {
 public:
  Task(LiveMemManagerModule &module, const VertexId &vid_to_unload,
       const unsigned &priority = 0)
      : BaseTask(priority), module_(module), vid_to_unload_(vid_to_unload) {}

  void run(const TaskExecutor::Ptr &, const Graph::Ptr &graph) override {
    auto vertex = graph->at(vid_to_unload_);
    CLOG(DEBUG, "tactic.module.live_mem_manager")
        << "Saving and unloading data associated with vertex: " << *vertex;
    vertex->unload();
  }

 private:
  LiveMemManagerModule &module_;  /// does not copy
  const VertexId vid_to_unload_;
};

void LiveMemManagerModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                         const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->window_size = node->declare_parameter<int>(param_prefix + ".window_size", config_->window_size);
  // clang-format on
}

void LiveMemManagerModule::runImpl(QueryCache &qdata, const Graph::ConstPtr &) {
  if (!task_queue_) return;
  if (qdata.live_id->isValid() &&
      qdata.live_id->minorId() >= (unsigned)config_->window_size &&
      *qdata.keyframe_test_result == KeyframeTestResult::CREATE_VERTEX) {
    const auto vid_to_unload =
        VertexId(qdata.live_id->majorId(),
                 qdata.live_id->minorId() - (unsigned)config_->window_size);

    task_queue_->dispatch(std::make_shared<Task>(*this, vid_to_unload));
  }
}

}  // namespace tactic
}  // namespace vtr