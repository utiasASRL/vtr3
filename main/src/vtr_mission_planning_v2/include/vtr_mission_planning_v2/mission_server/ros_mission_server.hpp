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
 * \file ros_mission_server.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "vtr_mission_planning_msgs/action/mission.hpp"
#include "vtr_mission_planning_msgs/msg/server_state.hpp"
#include "vtr_mission_planning_msgs/srv/command.hpp"
#include "vtr_mission_planning_msgs/srv/pause.hpp"

#include "vtr_mission_planning_v2/mission_server/mission_server.hpp"

namespace vtr {
namespace mission_planning {

/** \brief Template specialization to standardize the goal interface */
template <>
class GoalInterface<std::shared_ptr<rclcpp_action::ServerGoalHandle<
    vtr_mission_planning_msgs::action::Mission>>> {
 public:
  using Mission = vtr_mission_planning_msgs::action::Mission;
  using GoalHandle = std::shared_ptr<rclcpp_action::ServerGoalHandle<Mission>>;
  using Id = rclcpp_action::GoalUUID;
  using IdHash = std::hash<Id>;
  using Goal = Mission::Goal;

  using Mutex = std::mutex;
  using CondVar = std::condition_variable;
  using LockGuard = std::lock_guard<Mutex>;
  using UniqueLock = std::unique_lock<Mutex>;

  static const Id& InvalidId() {
    static Id invalid_id;
    return invalid_id;
  }
  static Id id(const GoalHandle& gh) { return gh->get_goal_id(); }
  static GoalTarget target(const GoalHandle& gh) {
    switch (gh->get_goal()->target) {
      case Goal::IDLE:
        return GoalTarget::Idle;
      case Goal::TEACH:
        return GoalTarget::Teach;
      case Goal::REPEAT:
        return GoalTarget::Repeat;
      default:
        return GoalTarget::Unknown;
    }
  }
  static std::list<tactic::VertexId> path(const GoalHandle& gh) {
    const auto& path = gh->get_goal()->path;
    return std::list<tactic::VertexId>(path.begin(), path.end());
  }
  static std::chrono::milliseconds pause_before(const GoalHandle& gh) {
    return std::chrono::milliseconds(gh->get_goal()->pause_before);
  }
  static std::chrono::milliseconds pause_after(const GoalHandle& gh) {
    return std::chrono::milliseconds(gh->get_goal()->pause_after);
  }
};

class ROSMissionServer
    : public MissionServer<std::shared_ptr<rclcpp_action::ServerGoalHandle<
          vtr_mission_planning_msgs::action::Mission>>> {
 public:
  PTR_TYPEDEFS(ROSMissionServer);

  using Mission = vtr_mission_planning_msgs::action::Mission;
  using GoalHandle = std::shared_ptr<rclcpp_action::ServerGoalHandle<Mission>>;
  using GoalId = rclcpp_action::GoalUUID;
  using Goal = Mission::Goal;
  using Result = Mission::Result;
  using Feedback = Mission::Feedback;

  using CommandSrv = vtr_mission_planning_msgs::srv::Command;
  using PauseSrv = vtr_mission_planning_msgs::srv::Pause;
  using ServerStateMsg = vtr_mission_planning_msgs::msg::ServerState;

  ROSMissionServer();
  ~ROSMissionServer() override;

  void start(const std::shared_ptr<rclcpp::Node>& node,
             const StateMachine::Ptr& state_machine);

  /// ROS callbacks
 private:
  rclcpp_action::GoalResponse handleGoal(const GoalId& goal_id,
                                         std::shared_ptr<const Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const GoalHandle goal_handle);
  void handleAccepted(const GoalHandle goal_handle);

  void handleCommand(std::shared_ptr<CommandSrv::Request> request,
                     std::shared_ptr<CommandSrv::Response> response);

  void handlePause(std::shared_ptr<PauseSrv::Request> request,
                   std::shared_ptr<PauseSrv::Response> response);

  /// Parent class overrides, only call from parent class
 private:
  void serverStateChanged(const ServerState& state) override;
  void goalStarted(const GoalHandle&) override;
  void goalWaiting(const GoalHandle&, bool) override;
  void goalFinished(const GoalHandle&) override;
  void goalCanceled(const GoalHandle&) override;
  void goalUpdated(const GoalHandle&, const double) override;

 private:
  void updateFeedback(const GoalHandle& goal_handle, const bool started,
                      const bool waiting, const double progress = -1.0);

  void process();

 private:
  rclcpp_action::Server<Mission>::SharedPtr mission_srv_;
  rclcpp::Service<PauseSrv>::SharedPtr pause_srv_;
  rclcpp::Service<CommandSrv>::SharedPtr command_srv_;
  rclcpp::Publisher<ServerStateMsg>::SharedPtr server_state_pub_;

 private:
  std::unordered_map<GoalId, std::shared_ptr<Feedback>> feedback_;

  /** \brief Protects: goal_canceling_queue_, stop_, thread_count_ */
  mutable Mutex mutex_;
  /** \brief wait until stop or current goal has changed */
  mutable CondVar cv_stop_or_goal_changed_;
  /** \brief wait until the goal canceling thread has finished */
  mutable CondVar cv_thread_finish_;

  /** \brief signal the process thread to stop */
  bool stop_ = false;

  /** \brief Goals to be canceled */
  std::queue<GoalHandle> goal_canceling_queue_;

  size_t thread_count_ = 0;
  /** \brief thread to wait and then start the current goal */
  std::thread goal_canceling_thread_;
};

}  // namespace mission_planning
}  // namespace vtr