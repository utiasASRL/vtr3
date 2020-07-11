#include <vtr/planning/ros_mission_server.h>

#if 0
#include <actionlib/client/simple_goal_state.h>
#include <asrl/messages/Command.pb.h>
#include <asrl/messages/Goal.pb.h>
#include <vtr_planning/MissionGoal.h>
#endif

namespace vtr {
namespace planning {

RosMissionServer::RosMissionServer(const ros::NodeHandle &nh,
                                   const typename StateMachine::Ptr &state)
    : BaseMissionServer(state), nh_(nh), actionServer_(nh, "manager", false) {
  actionServer_.registerGoalCallback(
      boost::bind(&RosMissionServer::_goalCallback, this, _1));
  actionServer_.registerCancelCallback(
      boost::bind(&RosMissionServer::goalCancelled, this, _1));
  actionServer_.start();

#if 0
  reorderService_ = nh_.advertiseService(
      "reorder", &RosMissionServer::_reorderCallback, this);
#endif
  pauseService_ =
      nh_.advertiseService("pause", &RosMissionServer::_pauseCallback, this);
#if 0
  cmdService_ =
      nh_.advertiseService("cmd", &RosMissionServer::_cmdCallback, this);
  uavCmdService_ =
      nh_.advertiseService("uavcmd", &RosMissionServer::_uavCmdCallback, this);
#endif
  statusPublisher_ =
      nh_.advertise<vtr_planning::MissionStatus>("status", 1, true);
  statusTimer_ = nh.createTimer(ros::Duration(0.25),
                                &RosMissionServer::_publishStatus, this);
#if 0
  uiPublisher_ = nh_.advertise<asrl__messages::UILog>("out/ui_log", 10, true);
#endif
}

RosMissionServer::~RosMissionServer() {
  // If we don't kill all of the goals, clients get out of sync and weird things
  // happen
  this->halt();
}

/// @brief ROS-specific new goal callback
void RosMissionServer::_goalCallback(GoalHandle gh) {
  LockGuard lck(this->lock_);
  LOG(INFO) << "Found new goal: " << Iface::id(gh);
  if (isTracking(gh.getGoalID().id)) {
    // We may get duplicate goal submissions if there are multiple clients, due
    // to an "eccentricity" in actionlib where there is no way to sync goals
    // between clients without adding them in each client
    return;
  }

  if (Iface::target(gh) == Target::Unknown) {
    // Check to make sure the goal definition is in range
    Result tmp;
    tmp.returnCode = Result::UNKNOWN_GOAL;
    gh.setRejected(tmp, "Goal target not in {IDLE, TEACH, REPEAT}");
  } else if (gh.getGoal()->target == Goal::REPEAT &&
             gh.getGoal()->path.size() == 0) {
    // Check to make sure that we got a path to repeat
    Result tmp;
    tmp.returnCode = Result::PATH_INVALID;
    gh.setRejected(tmp, "Issued a REPEAT goal without specifying a path");
  } else if (gh.getGoal()->target == Goal::MERGE &&
             gh.getGoal()->path.size() == 0 &&
             gh.getGoal()->vertex == VertexId::Invalid()) {
    Result tmp;
    tmp.returnCode = Result::PATH_INVALID;
    gh.setRejected(tmp,
                   "Cannot merge without a target vertex and/or target path");
  } else if (gh.getGoal()->target == Goal::LOCALIZE &&
             gh.getGoal()->path.size() == 0 &&
             gh.getGoal()->vertex == VertexId::Invalid()) {
    Result tmp;
    tmp.returnCode = Result::PATH_INVALID;
    gh.setRejected(
        tmp, "Cannot localize without a target vertex and/or target path");
  } else {
    // Otherwise we can accept this goal
    LOG(INFO) << "Adding goal: " << Iface::id(gh);
    this->addGoal(gh);
    this->_setFeedback(Iface::id(gh), false, 0);
    this->_publishFeedback(Iface::id(gh));
  }
}

#if 0
/// @brief ROS-specific goal reordering service callback
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
/// @brief ROS-specific pause service callback
bool RosMissionServer::_pauseCallback(MissionPause::Request &request,
                                      MissionPause::Response &response) {
#if 0
  // Republish service request for UI logging
  asrl::ui_msgs::MissionPause msg;
  msg.set_pause(request.paused);
  _publishUI(msg);
#endif
  ROS_INFO_NAMED("_pauseCallback", "In callback...");
  this->setPaused(request.paused);

  ROS_INFO_NAMED("_pauseCallback", "Parent pause finished");

  if (request.paused) {
    ROS_INFO_NAMED("_pauseCallback", "Requested a pause");
    switch (this->status()) {
      case ServerState::Processing:
      case ServerState::Empty:
        response.responseCode = MissionPause::Response::FAILURE;
        return true;
      case ServerState::Paused:
        response.responseCode = MissionPause::Response::SUCCESS;
        this->_publishStatus();
        return true;
      case ServerState::PendingPause:
        response.responseCode = MissionPause::Response::PENDING;
        this->_publishStatus();
        return true;
    }
  } else {
    ROS_INFO_NAMED("_pauseCallback", "Requested a continue");
    switch (this->status()) {
      case ServerState::Processing:
      case ServerState::Empty:
        response.responseCode = MissionPause::Response::SUCCESS;
        this->_publishStatus();
        return true;
      case ServerState::Paused:
      case ServerState::PendingPause:
        response.responseCode = MissionPause::Response::FAILURE;
        return true;
    }
  }

  // GCC throws a warning without this, as it cannot tell that an if/else on a
  // bool, containing a switch on an enum class, must necessarily cover all
  // options
  return false;
}
#if 0
/// @brief ROS-specific callback for mission commands
bool RosMissionServer::_cmdCallback(MissionCmd::Request &request,
                                    MissionCmd::Response &response) {
  // Republish service request for UI logging
  asrl::ui_msgs::MissionCmd msg;
  msg.set_vertex(request.vertex);

  for (auto &&it : request.path) {
    msg.mutable_path()->Add(it);
  }

  LockGuard lck(this->lock_);
  std::string name = stateMachine()->name();

  switch (request.action) {
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
      return true;
    }
    case MissionCmd::Request::LOCALIZE: {
      msg.set_action(asrl::ui_msgs::MissionCmd::LOCALIZE);
      _publishUI(msg);

      if (name == "::Idle") {
        LOG(INFO) << "Persistent vertex being set to: "
                  << VertexId(request.vertex);
        this->stateMachine()->tactic()->setTrunk(request.vertex);
        response.success = true;
      } else {
        response.success = false;
        response.message = "Cannot set the localization while not in ::Idle";
      }
      return true;
    }
    case MissionCmd::Request::START_MERGE: {
      msg.set_action(asrl::ui_msgs::MissionCmd::START_MERGE);
      _publishUI(msg);

      if (name == "::Teach::Branch") {
        VertexId::List tmp(request.path.begin(), request.path.end());
        this->stateMachine()->handleEvents(
            Event::StartMerge(tmp, request.vertex));
        response.success = true;
      } else {
        response.success = false;
        response.message =
            "Must be in ::Teach::Branch to move to ::Teach::Merge";
      }
      return true;
    }
    case MissionCmd::Request::CONFIRM_MERGE: {
      msg.set_action(asrl::ui_msgs::MissionCmd::CONFIRM_MERGE);
      _publishUI(msg);

      if (name == "::Teach::Merge") {
        this->stateMachine()->handleEvents(
            Event(state::Signal::AttemptClosure));
        response.success = true;
      } else {
        response.success = false;
        response.message =
            "Must be in ::Teach::Merge to confirm a loop closure";
      }
      return true;
    }
    case MissionCmd::Request::LOC_SEARCH: {
      msg.set_action(asrl::ui_msgs::MissionCmd::LOC_SEARCH);
      _publishUI(msg);

      if (name == "::Repeat::Follow") {
        this->stateMachine()->handleEvents(Event(state::Signal::LocalizeFail));
        response.success = true;
      } else {
        response.success = false;
        response.message =
            "Must be in ::Repeat::Follow to force a localization search";
      }
      return true;
    }
  }

  LOG(ERROR) << "[RosMissionServer] Unhandled action received: "
             << request.action;
  return false;
}

/// @brief ROS-specific callback for mission commands
bool RosMissionServer::_uavCmdCallback(UAVMissionCmd::Request &request,
                                       UAVMissionCmd::Response &response) {
  // Republish service request for UI logging
  asrl::ui_msgs::MissionCmd msg;
  msg.set_vertex(0);

  LockGuard lck(this->lock_);
  std::string name = stateMachine()->name();

  switch (request.action) {
    case UAVMissionCmd::Request::LEARN: {
      this->stateMachine()->handleEvents(planning::state::Event::StartLearn());
      response.success = true;
      return true;
    }
    case UAVMissionCmd::Request::LOITER: {
      this->addRun(false);
      this->stateMachine()->handleEvents(planning::state::Event::StartLoiter());
      response.success = true;
      return true;
    }
    case UAVMissionCmd::Request::HOVER: {
      this->stateMachine()->handleEvents(planning::state::Event::StartHover());
      response.success = true;
      return true;
    }
    case UAVMissionCmd::Request::RETURN: {
      VertexId::List tmp(request.path.begin(), request.path.end());
      this->stateMachine()->handleEvents(
          planning::state::Event::StartReturn(tmp));
      response.success = true;
      return true;
    }
    case UAVMissionCmd::Request::IDLE: {
      this->stateMachine()->handleEvents(planning::state::Event::StartIdle());
      response.success = true;
      return true;
    }
    case UAVMissionCmd::Request::START_JOIN: {
      if (name == "::Learn::Branch") {
        VertexId::List tmp(request.path.begin(), request.path.end());
        this->stateMachine()->handleEvents(
            Event::StartJoin(tmp, request.vertex));
        response.success = true;
      } else {
        response.success = false;
        response.message =
            "Must be in ::Learn::Branch to move to ::Learn::Merge";
      }
      return true;
    }
    case UAVMissionCmd::Request::CONFIRM_JOIN: {
      if (name == "::Learn::Merge") {
        this->stateMachine()->handleEvents(
            Event(state::Signal::AttemptClosure));
        response.success = true;
      } else {
        response.success = false;
        response.message =
            "Must be in ::Learn::Merge to confirm a loop closure";
      }
      return true;
    }
  }

  LOG(ERROR) << "[RosMissionServer] Unhandled action received: "
             << request.action;
  return false;
}
#endif
/// @brief ROS-specific feedback to ActionClient
void RosMissionServer::_publishFeedback(const std::string &id) {
  LockGuard lck(this->lock_);
  try {
    goal_map_.at(id)->publishFeedback(feedback_[id]);
  } catch (const std::out_of_range &e) {
    LOG(ERROR) << "Couldn't find goal in map: " << e.what();
  }
}

/// @brief ROS-specific feedback to ActionClient
void RosMissionServer::_setFeedback(const std::string &id, bool waiting,
                                    double percentComplete) {
  LockGuard lck(this->lock_);
  Feedback fbk;
  if (percentComplete >= 0) fbk.percentComplete = percentComplete;
  fbk.waiting = waiting;

  feedback_[id] = fbk;
}

/// @brief ROS-specific status message
void RosMissionServer::_publishStatus(const ros::TimerEvent &) {
  LockGuard lck(this->lock_);
  vtr_planning::MissionStatus msg;
  switch (this->status()) {
    case ServerState::Empty:
      msg.status = vtr_planning::MissionStatus::EMPTY;
      break;
    case ServerState::Paused:
      msg.status = vtr_planning::MissionStatus::PAUSED;
      break;
    case ServerState::PendingPause:
      msg.status = vtr_planning::MissionStatus::PENDING_PAUSE;
      break;
    case ServerState::Processing:
      msg.status = vtr_planning::MissionStatus::PROCESSING;
      break;
  }

  msg.missionQueue.clear();
  for (auto &&it : this->goal_queue_) {
    msg.missionQueue.push_back(Iface::id(it));
  }

  statusPublisher_.publish(msg);
}

/// @brief Callback when a new goal is in a waiting state
void RosMissionServer::goalWaiting(GoalHandle goal) {
  feedback_[Iface::id(goal)].waiting = true;
  this->_publishFeedback(Iface::id(goal));
}

/// @brief Callback when a new goal is accepted
void RosMissionServer::goalAccepted(GoalHandle goal) {
  LockGuard lck(this->lock_);

  // Notify the client that the goal is in progress (potentially waiting)
  goal.setAccepted();
  this->_publishStatus();

  // Accept the goal internally and deal with pauses.  NOTE: async = false
  Parent::goalAccepted(goal);
}

/// @brief Callback when a goal is finished waiting
void RosMissionServer::finishAccept(GoalHandle goal) {
  // Publish a status to clear the waiting flag
  feedback_[Iface::id(goal)].waiting = false;
  this->_publishFeedback(Iface::id(goal));
}

#if 0
/// @brief Callback when a new goal is rejected
void RosMissionServer::goalRejected(GoalHandle) {
  // TODO: do we need this function?
}

/// @brief Callback when the current goal completes successfully
void RosMissionServer::goalSucceeded() {
  LockGuard lck(this->lock_);
  // Cache these, because they are going to get deleted before we are done with
  // them
  GoalHandle gh(top());
  std::string gid = Iface::id(top());

  // Publish a feedback message at 100%
  feedback_[gid].percentComplete = 100.0;
  this->_publishFeedback(gid);
  feedback_.erase(Iface::id(top()));

  // Remove the goal from the queue and do any pauses
  Parent::goalSucceeded();

  // Publish updated goal queue
  this->_publishStatus();
}

/// @brief Callback when a goal is finished waiting
void RosMissionServer::finishSuccess(GoalHandle goal) {
  // Notify the client of the success
  Result res;
  res.returnCode = Result::SUCCESS;
  goal.setSucceeded(res);

  // Publish updated goal queue
  this->_publishStatus();
}

/// @brief Callback when the current goal terminates due to an internal error
void RosMissionServer::goalAborted(const std::string &msg) {
  LockGuard lck(this->lock_);
  // Notify the client of the cancellation
  Result res;
  res.returnCode = Result::EXCEPTION;
  top().setAborted(res, msg);

  this->_publishFeedback(Iface::id(top()));
  feedback_.erase(Iface::id(top()));

  Parent::goalAborted(msg);
  this->_publishStatus();
}
#endif
/// @brief Callback when an existing goal is cancelled by a user
void RosMissionServer::goalCancelled(GoalHandle goal) {
  LockGuard lck(this->lock_);
  // LOG(INFO) << "Canceling goal: " << Iface::id(goal);

  this->_publishFeedback(Iface::id(goal));
  feedback_.erase(Iface::id(top()));

  // This will (probably??) get triggered when the setCancelled callback is
  // invoked
  Parent::goalCancelled(goal);

  // Notify the client of the cancellation
  Result res;
  res.returnCode = Result::USER_INTERRUPT;
  goal.setCanceled(res, "User cancelled the goal");

  this->_publishStatus();
}
#if 0
/// @brief Callback when the state machine changes state
void RosMissionServer::stateChanged(const state::BaseState::Ptr &) {
  // TODO: State publishing
}

/// @brief Callback when the state machine registers progress on a goal
void RosMissionServer::stateUpdate(double percentComplete) {
  this->_setFeedback(Iface::id(top()), false, percentComplete);
  this->_publishFeedback(Iface::id(top()));
}
#endif

}  // namespace planning
}  // namespace vtr
