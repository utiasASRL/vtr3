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
 * \file task_queue_server.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

#include "vtr_navigation_msgs/msg/task_queue_task.hpp"
#include "vtr_navigation_msgs/msg/task_queue_update.hpp"
#include "vtr_navigation_msgs/srv/task_queue_state.hpp"

#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace navigation {

class TaskQueueServer : public tactic::TaskExecutor::Callback {
 public:
  PTR_TYPEDEFS(TaskQueueServer);

  using Mutex = std::shared_mutex;
  using UniqueLock = std::unique_lock<Mutex>;
  using SharedLock = std::shared_lock<Mutex>;

  using TaskQueueTask = vtr_navigation_msgs::msg::TaskQueueTask;
  using TaskQueueUpdate = vtr_navigation_msgs::msg::TaskQueueUpdate;
  using TaskQueueStateSrv = vtr_navigation_msgs::srv::TaskQueueState;

  TaskQueueServer(const rclcpp::Node::SharedPtr& node);

  void taskAdded(const tactic::Task::Ptr& task);
  void taskRemoved(const tactic::Task::Id& id, const bool completed);

 private:
  /// these functions, if necessary, must lock graph first then internal lock
  void taskQueueStateSrvCallback(
      const std::shared_ptr<TaskQueueStateSrv::Request>,
      std::shared_ptr<TaskQueueStateSrv::Response> response) const;

 private:
  /** \brief Protects all class member accesses */
  mutable Mutex mutex_;

  /** \brief Keep track of all tasks */
  std::unordered_map<tactic::Task::Id, TaskQueueTask> id2task_map_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  /** \brief Publishes updates to the relaxed graph */
  rclcpp::Publisher<TaskQueueUpdate>::SharedPtr task_queue_update_pub_;
  /** \brief Service to request a relaxed version of the graph */
  rclcpp::Service<TaskQueueStateSrv>::SharedPtr task_queue_state_srv_;
};

}  // namespace navigation
}  // namespace vtr