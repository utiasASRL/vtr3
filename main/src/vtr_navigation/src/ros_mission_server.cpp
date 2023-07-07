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
#include "vtr_navigation/ros_mission_server.hpp"

namespace vtr {
namespace navigation {

void ROSMissionServer::start(
    const std::shared_ptr<rclcpp::Node>& node,
    const mission_planning::StateMachineInterface::Ptr& state_machine) {
  MissionServer::start(state_machine);

  // clang-format off
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  server_state_pub_ = node->create_publisher<ServerStateMsg>("server_state", 1);
  server_state_srv_ = node->create_service<ServerStateSrv>("server_state_srv", std::bind(&ROSMissionServer::serverStateSrvCallback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);
  mission_command_sub_ = node->create_subscription<MissionCommandMsg>("mission_command", rclcpp::SystemDefaultsQoS(), std::bind(&ROSMissionServer::handleCommand, this, std::placeholders::_1), sub_opt);
  // clang-format on
}

void ROSMissionServer::serverStateSrvCallback(
    const std::shared_ptr<ServerStateSrv::Request>,
    std::shared_ptr<ServerStateSrv::Response> response) const {
  CLOG(DEBUG, "mission.server") << "Received server state request";
  LockGuard lock(mutex_);
  response->server_state = server_state_msg_;
}

void ROSMissionServer::handleCommand(
    const MissionCommandMsg::SharedPtr command) {
  mission_planning::Command tmp;
  switch (command->type) {
    case MissionCommandMsg::PAUSE:
      setPause(command->pause);
      return;
    case MissionCommandMsg::ADD_GOAL: {
      auto& gh = command->goal_handle;
      // generate a new uuid for this goal
      auto uuid = boost::uuids::random_generator()();
      std::copy(uuid.begin(), uuid.end(), gh.id.begin());
      CLOG(INFO, "mission.server") << "Adding goal with id: " << uuid;
      // sanity check
      if (gh.type == GoalHandle::REPEAT && gh.waypoints.size() == 0) {
        CLOG(WARNING, "mission.server") << "Issued a REPEAT Target without "
                                           "specifying a path - goal ignored";
        return;
      }
      addGoal(gh);
      return;
    }
    case MissionCommandMsg::CANCEL_GOAL: {
      cancelGoal(command->goal_handle);
      return;
    }
    case MissionCommandMsg::BEGIN_GOALS: {
      CLOG(INFO, "mission.server") << "Beginning to process goal queue";
      beginGoals();
      return;
    }
    case MissionCommandMsg::LOCALIZE:
      tmp.target = mission_planning::CommandTarget::Localize;
      tmp.vertex = command->vertex;
      processCommand(tmp);
      return;
    case MissionCommandMsg::START_MERGE:
      tmp.target = mission_planning::CommandTarget::StartMerge;
      tmp.path = std::vector<tactic::VertexId>(command->window.begin(),
                                               command->window.end());
      processCommand(tmp);
      return;
    case MissionCommandMsg::CONFIRM_MERGE:
      tmp.target = mission_planning::CommandTarget::ConfirmMerge;
      processCommand(tmp);
      return;
    case MissionCommandMsg::CONTINUE_TEACH:
      tmp.target = mission_planning::CommandTarget::ContinueTeach;
      processCommand(tmp);
      return;
    default:
      return;
  }
}

void ROSMissionServer::serverStateChanged() {
  // list of goals
  server_state_msg_.goals.clear();
  for (const auto& goal : goal_queue_)
    server_state_msg_.goals.push_back(goal_map_.at(goal));
  // current goal id
  auto& tmp = server_state_msg_.current_goal_id;
  std::copy(current_goal_id_.begin(), current_goal_id_.end(), tmp.begin());
  // current goal state
  switch (current_goal_state_) {
    case mission_planning::GoalState::Empty:
      server_state_msg_.current_goal_state = ServerStateMsg::EMPTY;
      break;
    case mission_planning::GoalState::Starting:
      server_state_msg_.current_goal_state = ServerStateMsg::STARTING;
      break;
    case mission_planning::GoalState::Running:
      server_state_msg_.current_goal_state = ServerStateMsg::RUNNING;
      break;
    case mission_planning::GoalState::Finishing:
      server_state_msg_.current_goal_state = ServerStateMsg::FINISHING;
      break;
  }
  // server state
  switch (current_server_state_) {
    case mission_planning::ServerState::Empty:
      server_state_msg_.server_state = ServerStateMsg::EMPTY;
      break;
    case mission_planning::ServerState::Paused:
      server_state_msg_.server_state = ServerStateMsg::PAUSED;
      break;
    case mission_planning::ServerState::PendingPause:
      server_state_msg_.server_state = ServerStateMsg::PENDING_PAUSE;
      break;
    case mission_planning::ServerState::Processing:
      server_state_msg_.server_state = ServerStateMsg::PROCESSING;
      break;
  }
  //
  if (server_state_pub_) server_state_pub_->publish(server_state_msg_);
}

}  // namespace navigation
}  // namespace vtr