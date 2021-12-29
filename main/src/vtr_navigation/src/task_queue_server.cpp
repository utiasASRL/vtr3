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
 * \file task_queue_server.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_navigation/task_queue_server.hpp"

namespace vtr {
namespace navigation {

TaskQueueServer::TaskQueueServer(const rclcpp::Node::SharedPtr& node) {
  // clang-format off
  /// Publishers and services
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //
  task_queue_update_pub_ = node->create_publisher<TaskQueueUpdate>("task_queue_update", 100);
  task_queue_state_srv_ = node->create_service<TaskQueueStateSrv>("task_queue_state_srv", std::bind(&TaskQueueServer::taskQueueStateSrvCallback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);
  // clang-format on
}

void TaskQueueServer::taskAdded(const tactic::Task::Ptr& task) {
  UniqueLock lock(mutex_);
  auto res = id2task_map_.emplace(task->id, TaskQueueTask());
  auto& task_msg = res.first->second;
  task_msg.id = task->id;
  task_msg.name = task->name;
  task_msg.vid = task->vid;
  // publish the task queue update message
  TaskQueueUpdate update;
  update.type = TaskQueueUpdate::ADD;
  update.task = task_msg;
  task_queue_update_pub_->publish(update);
}

void TaskQueueServer::taskRemoved(const tactic::Task::Id& id, const bool) {
  UniqueLock lock(mutex_);
  //
  id2task_map_.erase(id);
  //
  TaskQueueTask task_msg;
  task_msg.id = id;
  // publish the task queue update message
  TaskQueueUpdate update;
  update.type = TaskQueueUpdate::REMOVE;
  update.task = task_msg;
  task_queue_update_pub_->publish(update);
}

void TaskQueueServer::taskQueueStateSrvCallback(
    const std::shared_ptr<TaskQueueStateSrv::Request>,
    std::shared_ptr<TaskQueueStateSrv::Response> response) const {
  CLOG(DEBUG, "navigator.task_queue_server")
      << "Received task queue state request";
  SharedLock lock(mutex_);
  auto& tasks = response->task_queue_state.tasks;
  for (const auto& task : id2task_map_) tasks.push_back(task.second);
}

}  // namespace navigation
}  // namespace vtr