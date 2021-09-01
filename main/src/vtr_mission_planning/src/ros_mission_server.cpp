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
 * \file ros_mission_server.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_mission_planning/ros_mission_server.hpp>

std::ostream &operator<<(std::ostream &os, const rclcpp_action::GoalUUID &id) {
  for (auto i : id) os << std::to_string(i);
  return os;
}

namespace vtr {
namespace mission_planning {

RosMissionServer::RosMissionServer(const std::shared_ptr<rclcpp::Node> node,
                                   const typename StateMachine::Ptr &state)
    : BaseMissionServer(state), node_(node) {
  // clang-format off
  action_server_ = rclcpp_action::create_server<Mission>(
      node_, "manager",
      std::bind(&RosMissionServer::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosMissionServer::_handleCancel, this, std::placeholders::_1),
      std::bind(&RosMissionServer::_handleAccepted, this, std::placeholders::_1));
#if 0
  reorder_service_ = node->create_service<GoalReorder>("reorder", std::bind(&RosMissionServer::_reorderCallback, this, std::placeholders::_1, std::placeholders::_2));
#endif
  pause_service_ = node_->create_service<MissionPause>("pause", std::bind(&RosMissionServer::_pauseCallback, this, std::placeholders::_1, std::placeholders::_2));
  cmd_service_ = node->create_service<MissionCmd>("mission_cmd", std::bind(&RosMissionServer::_cmdCallback, this, std::placeholders::_1, std::placeholders::_2));

  status_publisher_ = node_->create_publisher<MissionStatus>("status", 1);
  status_timer_ = node_->create_wall_timer(1s, std::bind(&RosMissionServer::_publishStatus, this));
#if 0
  ui_publisher_ = node_->create_publisher<UILog>("out/ui_log", 10);
#endif
  // clang-format on
}

void RosMissionServer::stateUpdate(double percent_complete) {
  _setFeedback(Iface::id(top()), true, false, percent_complete);
  _publishFeedback(Iface::id(top()));
}

void RosMissionServer::abortGoal(GoalHandle gh, const std::string &msg) {
  LockGuard lck(lock_);
  // Notify the client of the cancellation
  auto result = std::make_shared<Mission::Result>();
  result->return_code = Mission::Result::EXCEPTION;
  gh->abort(result);

  _publishFeedback(Iface::id(gh));
  feedback_.erase(Iface::id(gh));

  Parent::abortGoal(gh, msg);
  _publishStatus();
}

void RosMissionServer::cancelGoal(GoalHandle gh) {
  LockGuard lck(lock_);

  _publishFeedback(Iface::id(gh));
  feedback_.erase(Iface::id(top()));

  Parent::cancelGoal(gh);

  // Notify the client of the cancellation
  auto result = std::make_shared<Mission::Result>();
  result->return_code = Mission::Result::USER_INTERRUPT;

  /// \todo this if checks if cancelGoal is initiated by the user or due to
  /// terminatng the process. Need a better way to handle this. Maybe send a
  /// terminating signal to UI.
  if (gh->is_canceling()) gh->canceled(result);

  _publishStatus();
}

void RosMissionServer::executeGoal(GoalHandle gh) {
  LockGuard lck(lock_);

  // Notify the client that the goal is in progress (potentially waiting)
  gh->execute();
  _publishStatus();

  // Accept the goal internally and deal with pauses.  NOTE: async = false
  Parent::executeGoal(gh);
}

void RosMissionServer::finishGoal(GoalHandle gh) {
  // Notify the client of the success
  auto result = std::make_shared<Mission::Result>();
  result->return_code = Mission::Result::SUCCESS;
  gh->succeed(result);

  // Remove the feedback entry of this goal
  feedback_.erase(Iface::id(gh));

  // Publish updated goal queue
  _publishStatus();
}

void RosMissionServer::transitionToNextGoal(GoalHandle gh) {
  LockGuard lck(lock_);

  // Publish a feedback message at 100%
  _setFeedback(Iface::id(gh), 100.0);
  _publishFeedback(Iface::id(gh));

  // Remove the goal from the queue and do any pauses
  Parent::transitionToNextGoal(gh);

  // Publish updated goal queue
  _publishStatus();
}

void RosMissionServer::setGoalStarted(GoalHandle gh) {
  LockGuard lck(lock_);
  _setFeedback(Iface::id(gh), true, false, 0);
  _publishFeedback(Iface::id(gh));
}

void RosMissionServer::setGoalWaiting(GoalHandle gh, bool waiting) {
  LockGuard lck(lock_);
  _setFeedback(Iface::id(gh), waiting);
  _publishFeedback(Iface::id(gh));
}

rclcpp_action::GoalResponse RosMissionServer::_handleGoal(
    const typename Iface::Id &uuid, std::shared_ptr<const Mission::Goal>) {
  LOG(INFO) << "Found new goal: " << uuid;
  if (isTracking(uuid)) return rclcpp_action::GoalResponse::REJECT;
  return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse RosMissionServer::_handleCancel(GoalHandle gh) {
  if (!isTracking(Iface::id(gh))) return rclcpp_action::CancelResponse::REJECT;

  // Launch a separate thread to cancel the goal after ros sets it to canceling.
  // Check if we have a goal to cancel, and block if we do.
  if (cancel_goal_future_.valid()) cancel_goal_future_.get();
  cancel_goal_future_ = std::async(std::launch::async, [this, gh] {
    while (!gh->is_canceling())
      ;  // wait until the ros server says the goal is canceling.
    cancelGoal(gh);
  });
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosMissionServer::_handleAccepted(GoalHandle gh) {
  if (Iface::target(gh) == Target::Unknown) {
    // Check to make sure the goal definition is in range
    auto result = std::make_shared<Mission::Result>();
    result->return_code = Mission::Result::UNKNOWN_GOAL;
    LOG(ERROR) << "Goal target not in {IDLE, TEACH, REPEAT}";
    gh->execute();
    gh->abort(result);
  } else if (Iface::target(gh) == Target::Repeat &&
             Iface::path(gh).size() == 0) {
    // Check to make sure that we got a path to repeat
    auto result = std::make_shared<Mission::Result>();
    result->return_code = Mission::Result::PATH_INVALID;
    LOG(ERROR) << "Issued a REPEAT Target without specifying a path";
    gh->execute();
    gh->abort(result);
  } else if (Iface::target(gh) == Target::Merge &&
             Iface::path(gh).size() == 0 &&
             Iface::vertex(gh) == VertexId::Invalid()) {
    auto result = std::make_shared<Mission::Result>();
    result->return_code = Mission::Result::PATH_INVALID;
    LOG(ERROR) << "Cannot merge without a target vertex and/or target path";
    gh->execute();
    gh->abort(result);
  } else if (Iface::target(gh) == Target::Localize &&
             Iface::path(gh).size() == 0 &&
             Iface::vertex(gh) == VertexId::Invalid()) {
    auto result = std::make_shared<Mission::Result>();
    result->return_code = Mission::Result::PATH_INVALID;
    LOG(ERROR) << "Cannot localize without a target vertex and/or target path";
    gh->execute();
    gh->abort(result);
  } else {
    // Otherwise we can accept this goal
    LOG(INFO) << "Adding goal: " << Iface::id(gh);
    LockGuard lck(lock_);
    _setFeedback(Iface::id(gh), false, false, 0);
    // need to addGoal between set and publish because _publishFeedback requires
    // gh to be in goal_map, which is added in this function
    addGoal(gh);
    _publishFeedback(Iface::id(gh));
  }
}

#if 0
bool RosMissionServer::_reorderCallback(GoalReorder::Request &request,
                                        GoalReorder::Response &response) {
  // Republish service request for UI logging
  asrl::ui_msgs::ReorderGoal msg;
  msg.set_goal_id(request.goalId);
  msg.set_before_id(request.beforeId);
  msg.set_to_idx(request.toIdx);

  for (auto &&it : request.totalOrder) {
    *msg.mutable_total_order()->Add() = it;
  }

  _publishUI(msg);

  LockGuard lck(this->lock_);
  if (request.goalId != "") {
    if (this->goal(request.goalId).getGoalStatus().status !=
        actionlib_msgs::GoalStatus::PENDING) {
      response.returnCode = GoalReorder::Response::GOAL_IN_PORGRESS;
      return true;
    } else if (request.beforeId != "") {
      // Relative case: move the goal to just before another goal
      this->moveGoal(request.goalId, request.beforeId);
    } else {
      // Index case: move the goal to a position, or the end if toIdx is not in
      // [0, goal_queue_.size()]
      this->moveGoal(request.goalId, request.toIdx);
    }

    //    this->_publishFeedback();
    this->_publishStatus();
    return true;
  } else if (request.totalOrder.size() > 0) {
    this->reorderGoals(std::list<std::string>(request.totalOrder.begin(),
                                              request.totalOrder.end()));
    //    this->_publishFeedback();
    this->_publishStatus();
    return true;
  } else {
    response.returnCode = 255;
    return true;
  }

  // If something goes really wrong, don't send a response
  return false;
}
#endif

void RosMissionServer::_pauseCallback(
    std::shared_ptr<MissionPause::Request> request,
    std::shared_ptr<MissionPause::Response> response) {
#if 0
  // Republish service request for UI logging
  asrl::ui_msgs::MissionPause msg;
  msg.set_pause(request.paused);
  _publishUI(msg);
#endif
  setPause(request->pause);

  if (request->pause) {
    switch (status()) {
      case ServerState::Processing:
      case ServerState::Empty:
        response->response_code = MissionPause::Response::FAILURE;
        return;
      case ServerState::Paused:
        response->response_code = MissionPause::Response::SUCCESS;
        _publishStatus();
        return;
      case ServerState::PendingPause:
        response->response_code = MissionPause::Response::PENDING;
        _publishStatus();
        return;
    }
  } else {
    switch (status()) {
      case ServerState::Processing:
      case ServerState::Empty:
        response->response_code = MissionPause::Response::SUCCESS;
        _publishStatus();
        return;
      case ServerState::Paused:
      case ServerState::PendingPause:
        response->response_code = MissionPause::Response::FAILURE;
        return;
    }
  }
}

void RosMissionServer::_cmdCallback(
    std::shared_ptr<MissionCmd::Request> request,
    std::shared_ptr<MissionCmd::Response> response) {
#if 0
  // Republish service request for UI logging
  asrl::ui_msgs::MissionCmd msg;
  msg.set_vertex(request.vertex);

  for (auto &&it : request.path) {
    msg.mutable_path()->Add(it);
  }
#endif
  LockGuard lck(lock_);
  std::string name = stateMachine()->name();

  switch (request->action) {
#if 0
    case MissionCmd::Request::ADD_RUN: {
      msg.set_action(asrl::ui_msgs::MissionCmd::ADD_RUN);
      _publishUI(msg);

      if (name == "::Idle") {
        this->addRun();
        response.success = true;
      } else {
        response.success = false;
        response.message = "Cannot add a run while not in ::Idle";
      }
    }
#endif
    case MissionCmd::Request::LOCALIZE: {
#if 0
      msg.set_action(asrl::ui_msgs::MissionCmd::LOCALIZE);
      _publishUI(msg);
#endif
      if (name == "::Idle") {
        LOG(INFO) << "Persistent vertex being set to: "
                  << VertexId(request->vertex);
        stateMachine()->tactic()->setTrunk(request->vertex);
        response->success = true;
      } else {
        response->success = false;
        response->message = "Cannot set the localization while not in ::Idle";
      }
      return;
    }
    case MissionCmd::Request::START_MERGE: {
#if 0
      msg.set_action(asrl::ui_msgs::MissionCmd::START_MERGE);
      _publishUI(msg);
#endif
      if (name == "::Teach::Branch") {
        VertexId::List tmp(request->path.begin(), request->path.end());
        stateMachine()->handleEvents(Event::StartMerge(tmp, request->vertex));
        response->success = true;
      } else if (name == "::Teach::Merge") {
        VertexId::Vector path(request->path.begin(), request->path.end());
        typename state::teach::Merge::Ptr tmp(new state::teach::Merge());
        tmp->setTarget(path, request->vertex);
        stateMachine()->handleEvents(
            Event(tmp, state::Signal::SwitchMergeWindow));
        response->success = true;
      } else {
        response->success = false;
        response->message =
            "Must be in ::Teach::Branch to move to ::Teach::Merge";
      }
      return;
    }
    case MissionCmd::Request::CONFIRM_MERGE: {
#if 0
      msg.set_action(asrl::ui_msgs::MissionCmd::CONFIRM_MERGE);
      _publishUI(msg);
#endif
      if (name == "::Teach::Merge") {
        stateMachine()->handleEvents(Event(state::Signal::AttemptClosure));
        response->success = true;
      } else {
        response->success = false;
        response->message =
            "Must be in ::Teach::Merge to confirm a loop closure";
      }
      return;
    }
    case MissionCmd::Request::CONTINUE_TEACH: {
#if 0
      msg.set_action(asrl::ui_msgs::MissionCmd::CONTINUE_TEACH);
      _publishUI(msg);
#endif
      if (name == "::Teach::Merge") {
        stateMachine()->handleEvents(Event(state::Signal::ContinueTeach));
        response->success = true;
      } else if (name == "::Teach::Branch") {
        // nothing to do if we are already in branch mode.
        response->success = true;
      } else {
        response->success = false;
        response->message =
            "Must be in ::Teach::Merge or ::Teach::Branch to continue teach";
      }
      return;
    }
    case MissionCmd::Request::LOC_SEARCH: {
#if 0
      msg.set_action(asrl::ui_msgs::MissionCmd::LOC_SEARCH);
      _publishUI(msg);
#endif
      if (name == "::Repeat::Follow") {
        this->stateMachine()->handleEvents(Event(state::Signal::LocalizeFail));
        response->success = true;
      } else {
        response->success = false;
        response->message =
            "Must be in ::Repeat::Follow to force a localization search";
      }
      return;
    }
  }

  LOG(ERROR) << "[RosMissionServer] Unhandled action received: "
             << request->action;
}

void RosMissionServer::_publishFeedback(const Iface::Id &id) {
  LockGuard lck(lock_);
  try {
    if (feedback_[id] == nullptr) return;
    (*goal_map_.at(id))->publish_feedback(feedback_[id]);
  } catch (const std::out_of_range &e) {
    LOG(ERROR) << "Couldn't find goal in map: " << e.what();
  } catch (const rclcpp::exceptions::RCLError &e) {
    /// \todo may due to cancel all
    LOG(ERROR) << "ROS error: " << e.what();
  }
}

void RosMissionServer::_setFeedback(const Iface::Id &id, bool started,
                                    bool waiting, double percent_complete) {
  LockGuard lck(lock_);
  if (!feedback_.count(id))
    feedback_[id] = std::make_shared<Mission::Feedback>();
  feedback_[id]->in_progress = started;
  feedback_[id]->waiting = waiting;
  feedback_[id]->percent_complete = percent_complete;
}

void RosMissionServer::_publishStatus() {
  LockGuard lck(lock_);
  auto msg = MissionStatus{};
  // status
  switch (status()) {
    case ServerState::Empty:
      msg.status = MissionStatus::EMPTY;
      break;
    case ServerState::Paused:
      msg.status = MissionStatus::PAUSED;
      break;
    case ServerState::PendingPause:
      msg.status = MissionStatus::PENDING_PAUSE;
      break;
    case ServerState::Processing:
      msg.status = MissionStatus::PROCESSING;
      break;
  }
  // goals
  msg.mission_queue.clear();
  for (auto &&it : goal_queue_) {
    GoalUUID uuid;
    uuid.uuid = Iface::id(it);
    msg.mission_queue.push_back(uuid);
  }

  status_publisher_->publish(msg);
}

}  // namespace mission_planning
}  // namespace vtr