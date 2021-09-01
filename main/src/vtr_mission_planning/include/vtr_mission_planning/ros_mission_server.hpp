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
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <vtr_messages/action/mission.hpp>
#include <vtr_messages/msg/goal_uuid.hpp>
#include <vtr_messages/msg/mission_status.hpp>
#include <vtr_messages/srv/mission_cmd.hpp>
#include <vtr_messages/srv/mission_pause.hpp>
#include <vtr_mission_planning/base_mission_server.hpp>

std::ostream& operator<<(std::ostream& os, const rclcpp_action::GoalUUID& id);

namespace vtr {
namespace mission_planning {

using Mission = vtr_messages::action::Mission;
using GoalUUID = vtr_messages::msg::GoalUUID;
using MissionStatus = vtr_messages::msg::MissionStatus;
using MissionCmd = vtr_messages::srv::MissionCmd;
using MissionPause = vtr_messages::srv::MissionPause;

/**
 * \brief Template specialization to coerce this goal into the interface we
 * need
 */
template <>
struct GoalInterface<
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Mission>>> {
  using GoalHandle = std::shared_ptr<rclcpp_action::ServerGoalHandle<Mission>>;
  using Goal = vtr_messages::action::Mission_Goal;
  using Result = vtr_messages::action::Mission_Result;
  using Feedback = vtr_messages::action::Mission_Feedback;
  using Id = rclcpp_action::GoalUUID;

  static const Id& id(const GoalHandle& gh) { return gh->get_goal_id(); }
  static Target target(const GoalHandle& gh) {
    switch (gh->get_goal()->target) {
      case Goal::IDLE:
        return Target::Idle;
      case Goal::TEACH:
        return Target::Teach;
      case Goal::REPEAT:
        return Target::Repeat;
      case Goal::MERGE:
        return Target::Merge;
      case Goal::LOCALIZE:
        return Target::Localize;
      default:
        return Target::Unknown;
    }
  }
  static std::list<VertexId> path(const GoalHandle& gh) {
    std::list<VertexId> path;
    for (auto&& it : gh->get_goal()->path) path.push_back(it);
    return path;
  }
  static VertexId vertex(const GoalHandle& gh) {
    return gh->get_goal()->vertex;
  }
  static std::chrono::milliseconds pauseBefore(const GoalHandle& gh) {
    return std::chrono::milliseconds(
        uint64_t(gh->get_goal()->pause_before.sec * 1E3 +
                 gh->get_goal()->pause_before.nanosec / 1E3));
  }
  static std::chrono::milliseconds pauseAfter(const GoalHandle& gh) {
    return std::chrono::milliseconds(
        uint64_t(gh->get_goal()->pause_after.sec * 1E3 +
                 gh->get_goal()->pause_after.nanosec / 1E3));
  }
};

/** \brief Mission server based on ROS */
class RosMissionServer
    : public BaseMissionServer<
          std::shared_ptr<rclcpp_action::ServerGoalHandle<Mission>>> {
 public:
  using GoalHandle = std::shared_ptr<rclcpp_action::ServerGoalHandle<Mission>>;
  using Goal = vtr_messages::action::Mission_Goal;
  using Result = vtr_messages::action::Mission_Result;
  using Feedback = vtr_messages::action::Mission_Feedback;

  using Parent = BaseMissionServer<GoalHandle>;

  PTR_TYPEDEFS(RosMissionServer)

  RosMissionServer(const std::shared_ptr<rclcpp::Node> node,
                   const typename StateMachine::Ptr& state = nullptr);

  ~RosMissionServer() override { halt(); }

  /** \brief Callback when the state machine changes state */
  void stateChanged(const state::BaseState::Ptr&) override {}
  /** \brief Callback when the state machine registers progress on a goal */
  void stateUpdate(double percent_complete) override;

  /** \brief Kill all goals and pause the server */
  void halt() override {
    Parent::halt();
    _publishStatus();
  }

 protected:
  /** \brief Terminates the goal due to an internal error */
  void abortGoal(GoalHandle gh, const std::string& msg) override;
  /** \brief Callback when an existing goal is cancelled by a user */
  void cancelGoal(GoalHandle goal) override;
  /** \brief Callback when a new goal is accepted */
  void executeGoal(GoalHandle gh) override;
  /** \brief Callback when a goal is finished waiting at the end */
  void finishGoal(GoalHandle gh) override;
  /** \brief Callback when the current goal completes successfully */
  void transitionToNextGoal(GoalHandle) override;
  /** \brief Callback when a new goal has started */
  void setGoalStarted(GoalHandle gh) override;
  /** \brief Callback when a new goal is in a waiting state */
  void setGoalWaiting(GoalHandle gh, bool waiting = true) override;

 private:
  /** \brief ROS-specific new goal callback */
  rclcpp_action::GoalResponse _handleGoal(
      const typename Iface::Id& uuid,
      std::shared_ptr<const Mission::Goal> goal);
  rclcpp_action::CancelResponse _handleCancel(GoalHandle gh);
  void _handleAccepted(GoalHandle gh);
#if 0
  /** \brief ROS-specific goal reordering service callback */
  bool _reorderCallback(std::shared_ptr<GoalReorder::Request> request,
                        std::shared_ptr<GoalReorder::Request> response);
#endif
  /** \brief ROS-specific pause service callback */
  void _pauseCallback(std::shared_ptr<MissionPause::Request> request,
                      std::shared_ptr<MissionPause::Response> response);

  /** \brief ROS-specific callback for mission commands */
  void _cmdCallback(std::shared_ptr<MissionCmd::Request> request,
                    std::shared_ptr<MissionCmd::Response> response);

  /** \brief ROS-specific status message */
  void _publishStatus();

  /** \brief ROS-specific feedback to ActionClient */
  void _publishFeedback(const Iface::Id& id);

  /** \brief Update the cached feedback messages */
  void _setFeedback(const Iface::Id& id, bool waiting) {
    LockGuard lck(lock_);
    if (!feedback_.count(id)) {
      std::string err{"No feedback registered!"};
      LOG(ERROR) << err;
      throw std::runtime_error{err};
    }
    _setFeedback(id, feedback_[id]->in_progress, waiting,
                 feedback_[id]->percent_complete);
  }
  void _setFeedback(const Iface::Id& id, double percent_complete) {
    LockGuard lck(lock_);
    if (!feedback_.count(id)) {
      std::string err{"No feedback registered!"};
      LOG(ERROR) << err;
      throw std::runtime_error{err};
    }
    _setFeedback(id, feedback_[id]->in_progress, feedback_[id]->waiting,
                 percent_complete);
  }
  void _setFeedback(const Iface::Id& id, bool started, bool waiting,
                    double percent_complete);
#if 0
  /**
   * \brief Utility function to serialize a protobuf UI message into a ROS
   * message an publish it
   */
  template <class MessageType>
  void _publishUI(const MessageType& msg,
                  const ros::Time& stamp = ros::Time::now()) {
    asrl__messages::UILog umsg;
    umsg.header.stamp = stamp;
    umsg.type = msg.GetTypeName();
    msg.SerializeToString(&umsg.payload);
    uiPublisher_.publish(umsg);
  }
#endif

  /** \brief ROS node handle */
  std::shared_ptr<rclcpp::Node> node_;

  /** \brief Service server for pausing mission execution */
  rclcpp::Service<MissionPause>::SharedPtr pause_service_;
#if 0
  /** \brief Service server for reordering existing goals */
  rclcpp::Service<GoalReorder>::SharedPtr reorder_service_;
#endif
  /** \brief Service server for mission commands */
  rclcpp::Service<MissionCmd>::SharedPtr cmd_service_;

  /** \brief Publish intermittent status updates */
  rclcpp::Publisher<MissionStatus>::SharedPtr status_publisher_;
#if 0
  /** \brief Republish commands to be logged for replay */
  rclcpp::Publisher<UILog>::SharedPtr ui_publisher_;
#endif
  /** \brief Timer to ensure we always send a heartbeat */
  rclcpp::TimerBase::SharedPtr status_timer_;

  /** \brief Action server that manages communication */
  rclcpp_action::Server<Mission>::SharedPtr action_server_;

  /** \brief */
  std::future<void> cancel_goal_future_;

  /** \brief Stored feedback msg to prevent passing things around too much */
  std::map<Iface::Id, std::shared_ptr<Mission::Feedback>> feedback_;
};

}  // namespace mission_planning
}  // namespace vtr