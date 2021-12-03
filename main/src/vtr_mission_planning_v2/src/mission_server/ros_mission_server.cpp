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
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning_v2/mission_server/ros_mission_server.hpp"

namespace vtr {
namespace mission_planning {

ROSMissionServer::ROSMissionServer() {
  thread_count_ = 1;
  goal_canceling_thread_ = std::thread(&ROSMissionServer::process, this);
}

ROSMissionServer::~ROSMissionServer() {
  UniqueLock lock(mutex_);
  stop_ = true;
  cv_stop_or_goal_changed_.notify_all();
  cv_thread_finish_.wait(lock, [&] { return thread_count_ == 0; });
  if (goal_canceling_thread_.joinable()) goal_canceling_thread_.join();
  //
  MissionServer::stop();
}

void ROSMissionServer::start(const std::shared_ptr<rclcpp::Node>& node,
                             const StateMachine::Ptr& state_machine) {
  MissionServer::start(state_machine);

  using namespace std::placeholders;
  // clang-format off
  mission_srv_ = rclcpp_action::create_server<Mission>(
      node, "manager",
      std::bind(&ROSMissionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ROSMissionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&ROSMissionServer::handleAccepted, this, std::placeholders::_1));
  pause_srv_ = node->create_service<PauseSrv>(
      "pause", std::bind(&ROSMissionServer::handlePause, this, std::placeholders::_1, std::placeholders::_2));
  command_srv_ = node->create_service<CommandSrv>(
      "command", std::bind(&ROSMissionServer::handleCommand, this, std::placeholders::_1, std::placeholders::_2));
  server_state_pub_ = node->create_publisher<ServerStateMsg>("server_state", 1);
  // clang-format on
}

rclcpp_action::GoalResponse ROSMissionServer::handleGoal(
    const GoalId& goal_id, std::shared_ptr<const Goal>) {
  CLOG(INFO, "mission.server") << "New goal request with id: " << goal_id;
  if (hasGoal(goal_id)) return rclcpp_action::GoalResponse::REJECT;
  return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse ROSMissionServer::handleCancel(
    const GoalHandle gh) {
  if (!hasGoal(GoalInterface<GoalHandle>::id(gh)))
    return rclcpp_action::CancelResponse::REJECT;

  LockGuard lock(mutex_);
  goal_canceling_queue_.push(gh);
  cv_stop_or_goal_changed_.notify_all();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void ROSMissionServer::handleAccepted(const GoalHandle gh) {
  const auto id = GoalInterface<GoalHandle>::id(gh);
  const auto target = GoalInterface<GoalHandle>::target(gh);
  const auto path = GoalInterface<GoalHandle>::path(gh);
  if (target == GoalTarget::Unknown) {
    CLOG(ERROR, "mission.server")
        << "Goal target is not in {IDLE, TEACH, REPEAT}";
    auto result = std::make_shared<Mission::Result>();
    result->return_code = Mission::Result::UNKNOWN_GOAL;
    gh->execute();
    gh->abort(result);
  } else if (target == GoalTarget::Repeat && path.size() == 0) {
    CLOG(ERROR, "mission.server")
        << "Issued a REPEAT Target without specifying a path";
    auto result = std::make_shared<Mission::Result>();
    result->return_code = Mission::Result::PATH_INVALID;
    gh->execute();
    gh->abort(result);
  } else {
    CLOG(INFO, "mission.server") << "Adding goal with id: " << id;
    this->addGoal(gh);
  }
}

void ROSMissionServer::handlePause(std::shared_ptr<PauseSrv::Request> request,
                                   std::shared_ptr<PauseSrv::Response>) {
  setPause(request->pause);
}

void ROSMissionServer::handleCommand(
    std::shared_ptr<CommandSrv::Request> request,
    std::shared_ptr<CommandSrv::Response>) {
  Command command;
  switch (request->action) {
    case CommandSrv::Request::LOCALIZE:
      command.target = CommandTarget::Localize;
      command.vertex = request->vertex;
      break;
    case CommandSrv::Request::START_MERGE:
      command.target = CommandTarget::StartMerge;
      command.path = std::vector<tactic::VertexId>(request->path.begin(),
                                                   request->path.end());
      break;
    case CommandSrv::Request::CONFIRM_MERGE:
      command.target = CommandTarget::ConfirmMerge;
      break;
    case CommandSrv::Request::CONTINUE_TEACH:
      command.target = CommandTarget::ContinueTeach;
      break;
    default:
      break;
  }
  this->processCommand(command);
}

void ROSMissionServer::serverStateChanged(const ServerState& state) {
  ServerStateMsg msg;
  // status
  switch (state) {
    case ServerState::Empty:
      msg.state = ServerStateMsg::EMPTY;
      break;
    case ServerState::Paused:
      msg.state = ServerStateMsg::PAUSED;
      break;
    case ServerState::PendingPause:
      msg.state = ServerStateMsg::PENDING_PAUSE;
      break;
    case ServerState::Processing:
      msg.state = ServerStateMsg::PROCESSING;
      break;
  }
#if false
  msg.mission_queue.clear();
  for (auto&& it : goal_queue_) {
    GoalUUID uuid;
    uuid.uuid = Iface::id(it);
    msg.mission_queue.push_back(uuid);
  }
#endif
  server_state_pub_->publish(msg);
}

void ROSMissionServer::goalStarted(const GoalHandle& gh) {
  gh->execute();
  updateFeedback(gh, true, false, 0.0);
}

void ROSMissionServer::goalWaiting(const GoalHandle& gh, const bool waiting) {
  updateFeedback(gh, true, waiting);
}

void ROSMissionServer::goalUpdated(const GoalHandle& gh,
                                   const double progress) {
  updateFeedback(gh, true, false, progress);
}

void ROSMissionServer::goalFinished(const GoalHandle& gh) {
  // Notify the client of the success
  auto result = std::make_shared<Mission::Result>();
  result->return_code = Mission::Result::SUCCESS;
  gh->succeed(result);

  //
  feedback_.erase(GoalInterface<GoalHandle>::id(gh));
}

void ROSMissionServer::goalCanceled(const GoalHandle& gh) {
  // Notify the client of the cancelation
  auto result = std::make_shared<Mission::Result>();
  result->return_code = Mission::Result::USER_INTERRUPT;
  gh->canceled(result);

  //
  feedback_.erase(GoalInterface<GoalHandle>::id(gh));
}

void ROSMissionServer::updateFeedback(const GoalHandle& gh, const bool started,
                                      const bool waiting,
                                      const double progress) {
  const auto feedback =
      feedback_.try_emplace(GoalInterface<GoalHandle>::id(gh)).first->second;
  feedback->started = started;
  feedback->waiting = waiting;
  feedback->progress = progress < 0. ? feedback->progress : progress;
  gh->publish_feedback(feedback);
}

void ROSMissionServer::process() {
  el::Helpers::setThreadName("mission.server.goal_canceling");
  CLOG(INFO, "mission.server") << "Starting the cancel goal thread.";
  while (true) {
    UniqueLock lock(mutex_);
    //
    cv_stop_or_goal_changed_.wait(
        lock, [&] { return stop_ || goal_canceling_queue_.size() > 0; });
    if (stop_) {
      CLOG(INFO, "mission.server") << "Stopping the cancel goal thread.";
      --thread_count_;
      cv_thread_finish_.notify_all();
      return;
    }
    //
    auto gh = goal_canceling_queue_.front();
    while (!gh->is_canceling())
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    goal_canceling_queue_.pop();
    this->cancelGoal(gh);
  }
}

}  // namespace mission_planning
}  // namespace vtr