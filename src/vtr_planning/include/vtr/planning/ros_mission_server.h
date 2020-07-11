#pragma once

#include <actionlib/server/action_server.h>

#include <vtr/planning/base_mission_server.h>
#include <vtr_planning/MissionAction.h>
#include <vtr_planning/MissionPause.h>
#include <vtr_planning/MissionStatus.h>

#if 0
#include <actionlib/client/action_client.h>
#include <std_srvs/Trigger.h>

#include <asrl__messages/UILog.h>
#include <asrl__planning/CloseLoop.h>
#include <asrl__planning/GoalReorder.h>
#include <asrl__planning/MissionCmd.h>
#include <asrl__planning/SetLocalization.h>
#include <asrl__planning/UAVMissionCmd.h>

#endif

namespace vtr {
namespace planning {

using vtr_planning::MissionPause;
#if 0
using asrl__planning::CloseLoop;
using asrl__planning::GoalReorder;
using asrl__planning::MissionCmd;
using asrl__planning::SetLocalization;
using asrl__planning::UAVMissionCmd;
#endif
/// @brief Template specialization to coerce this goal into the interface we
/// need
template <>
struct GoalInterface<
    actionlib::ServerGoalHandle<vtr_planning::MissionAction> > {
  using GoalHandle = actionlib::ServerGoalHandle<vtr_planning::MissionAction>;
  ACTION_DEFINITION(vtr_planning::MissionAction)

  static inline std::string id(const GoalHandle& gh) {
    return gh.getGoalID().id;
  }
  static inline Target target(const GoalHandle& gh) {
    switch (gh.getGoal()->target) {
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
#if 0      
      case Goal::LOITER:
        return Target::Loiter;
      case Goal::RETURN:
        return Target::Return;
      case Goal::LEARN:
        return Target::Learn;
      case Goal::HOVER:
        return Target::Hover;
#endif
      default:
        return Target::Unknown;
    }
  }
  static inline std::list<VertexId> path(const GoalHandle& gh) {
    std::list<VertexId> path;
    for (auto&& it : gh.getGoal()->path) {
      path.push_back(it);
    }
    return path;
  }
  static inline VertexId vertex(const GoalHandle& gh) {
    return gh.getGoal()->vertex;
  }
  static inline std::chrono::milliseconds pauseBefore(const GoalHandle& gh) {
    return std::chrono::milliseconds(
        uint64_t(gh.getGoal()->pauseBefore.sec * 1E3 +
                 gh.getGoal()->pauseBefore.nsec / 1E3));
  }
  static inline std::chrono::milliseconds pauseAfter(const GoalHandle& gh) {
    return std::chrono::milliseconds(
        uint64_t(gh.getGoal()->pauseAfter.sec * 1E3 +
                 gh.getGoal()->pauseAfter.nsec / 1E3));
  }
};

/// @brief Mission server subclass that uses a ROS action server to communicate
class RosMissionServer
    : public BaseMissionServer<
          actionlib::ServerGoalHandle<vtr_planning::MissionAction> > {
 public:
  using GoalHandle = actionlib::ServerGoalHandle<vtr_planning::MissionAction>;
  using Parent = BaseMissionServer<GoalHandle>;
#if 0
  using Iface =
      GoalInterface<actionlib::ServerGoalHandle<vtr_planning::MissionAction> >;
#endif

  // Stolen from the ActionLib implementation: adds typedefs for
  // Goal/Result/Feedback + pointers
  ACTION_DEFINITION(vtr_planning::MissionAction)

  RosMissionServer(const ros::NodeHandle& nh,
                   const typename StateMachine::Ptr& state = nullptr);
  virtual ~RosMissionServer();

  /// @brief Callback when a new goal is in a waiting state
  virtual void goalWaiting(GoalHandle goal);

  /// @brief Callback when a new goal is accepted
  virtual void goalAccepted(GoalHandle goal);
#if 0
  /// @brief Callback when a new goal is rejected
  virtual void goalRejected(GoalHandle);

  /// @brief Callback when the current goal completes successfully
  virtual void goalSucceeded();

  /// @brief Callback when the current goal terminates due to an internal error
  virtual void goalAborted(const std::string& msg);
#endif
  /// @brief Callback when an existing goal is cancelled by a user
  virtual void goalCancelled(GoalHandle goal);
#if 0
  /// @brief Callback when the state machine changes state
  virtual void stateChanged(const state::BaseState::Ptr&);

  /// @brief Callback when the state machine registers progress on a goal
  virtual void stateUpdate(double percentComplete);

  /// @brief Kill all goals and pause the server
  virtual void halt() {
    this->Parent::halt();
    this->_publishStatus();
  }
#endif
 protected:
  /// @brief Callback when a goal is finished waiting
  virtual void finishAccept(GoalHandle goal);
#if 0
  /// @brief Callback when a goal is finished waiting at the end
  virtual void finishSuccess(GoalHandle goal);
#endif
 private:
  /// @brief ROS-specific new goal callback
  void _goalCallback(GoalHandle gh);
#if 0
  /// @brief ROS-specific goal reordering service callback
  bool _reorderCallback(GoalReorder::Request& request,
                        GoalReorder::Response& response);
#endif
  /// @brief ROS-specific pause service callback
  bool _pauseCallback(MissionPause::Request& request,
                      MissionPause::Response& response);
#if 0
  /// @brief ROS-specific callback for mission commands
  bool _cmdCallback(MissionCmd::Request& request,
                    MissionCmd::Response& response);

  /// @brief ROS-specific callback for mission commands
  bool _uavCmdCallback(UAVMissionCmd::Request& request,
                       UAVMissionCmd::Response& response);
#endif
  /// @brief ROS-specific status message
  void _publishStatus(const ros::TimerEvent& e = ros::TimerEvent());
  /// @brief ROS-specific feedback to ActionClient
  void _publishFeedback(const std::string& id);
  /// @brief Update the cached feedback messages
  void _setFeedback(const std::string& id, bool waiting = false,
                    double percentComplete = -1);
#if 0
  /// @brief Utility function to serialize a protobuf UI message into a ROS
  /// message an publish it
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
  /// @brief ROS node handle
  ros::NodeHandle nh_;

  /// @brief Service server for pausing mission execution
  ros::ServiceServer pauseService_;

#if 0
  /// @brief Service server for reordering existing goals
  ros::ServiceServer reorderService_;


  /// @brief Service server for mission commands
  ros::ServiceServer cmdService_;

  /// @brief Service server for UAV mission commands
  ros::ServiceServer uavCmdService_;
#endif
  /// @brief Publish intermittent status updates
  ros::Publisher statusPublisher_;
#if 0
  /// @brief Republish commands to be logged for replay
  ros::Publisher uiPublisher_;
#endif
  /// @brief Timer to ensure we always send a heartbeat
  ros::Timer statusTimer_;
  /// @brief Action server that manages communication
  actionlib::ActionServer<vtr_planning::MissionAction> actionServer_;

  /// @brief Stored feedback messages to prevent passing things around too much
  std::map<std::string, Feedback> feedback_;
};

}  // namespace planning
}  // namespace vtr
