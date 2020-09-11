#pragma once

/// #include <actionlib/server/action_server.h>
/// #include <actionlib/client/action_client.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/// #include <vtr_planning/MissionAction.h>
#include <vtr_messages/action/mission.hpp>
/// #include <vtr_planning/MissionStatus.h>
#include <vtr_messages/msg/mission_status.hpp>
/// #include <vtr_planning/MissionPause.h>
#include <vtr_messages/srv/mission_pause.hpp>
#include <vtr_mission_planning/base_mission_server.hpp>

#if 0
#include <asrl__messages/UILog.h>
#include <asrl__planning/CloseLoop.h>
#include <asrl__planning/GoalReorder.h>
#include <asrl__planning/MissionCmd.h>
#include <asrl__planning/SetLocalization.h>
#include <std_srvs/Trigger.h>
#endif

std::ostream& operator<<(std::ostream& os, const rclcpp_action::GoalUUID& id);

namespace vtr {
namespace mission_planning {

/// using vtr_planning::MissionPause;
/// using asrl__planning::CloseLoop;
/// using asrl__planning::GoalReorder;
/// using asrl__planning::MissionCmd;
/// using asrl__planning::SetLocalization;
using Mission = vtr_messages::action::Mission;
using MissionStatus = vtr_messages::msg::MissionStatus;
using MissionPause = vtr_messages::srv::MissionPause;

/**
 * \brief Template specialization to coerce this goal into the interface we
 * need
 */
template <>
/// struct GoalInterface<
///     actionlib::ServerGoalHandle<vtr_planning::MissionAction> > {
struct GoalInterface<
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Mission>>> {
  /// using GoalHandle =
  /// actionlib::ServerGoalHandle<vtr_planning::MissionAction>;
  using GoalHandle = std::shared_ptr<rclcpp_action::ServerGoalHandle<Mission>>;
  /// ACTION_DEFINITION(vtr_planning::MissionAction)
  using Goal = vtr_messages::action::Mission_Goal;
  using Result = vtr_messages::action::Mission_Result;
  using Feedback = vtr_messages::action::Mission_Feedback;
  using Id = rclcpp_action::GoalUUID;

  /// static inline std::string id(const GoalHandle& gh) {
  ///   return gh.getGoalID().id;
  /// }
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

/**
 * \brief Mission server subclass that uses a ROS action server to communicate
 */
/// class RosMissionServer
///     : public BaseMissionServer<
///           actionlib::ServerGoalHandle<vtr_planning::MissionAction> > {
class RosMissionServer
    : public BaseMissionServer<
          std::shared_ptr<rclcpp_action::ServerGoalHandle<Mission>>> {
 public:
  /// using GoalHandle =
  /// actionlib::ServerGoalHandle<vtr_planning::MissionAction>;
  using GoalHandle = std::shared_ptr<rclcpp_action::ServerGoalHandle<Mission>>;
  // Stolen from the ActionLib implementation: adds typedefs for
  // Goal/Result/Feedback + pointers
  /// ACTION_DEFINITION(vtr_planning::MissionAction)
  using Goal = vtr_messages::action::Mission_Goal;
  using Result = vtr_messages::action::Mission_Result;
  using Feedback = vtr_messages::action::Mission_Feedback;

  using Parent = BaseMissionServer<GoalHandle>;
#if 0
  /// using Iface =
  ///     GoalInterface<actionlib::ServerGoalHandle<vtr_planning::MissionAction> >;
  using Iface = GoalInterface<GoalHandle>;
#endif

  PTR_TYPEDEFS(RosMissionServer)

  /// RosMissionServer(const ros::NodeHandle& nh,
  ///                  const typename StateMachine::Ptr& state = nullptr);
  RosMissionServer(const std::shared_ptr<rclcpp::Node> node,
                   const typename StateMachine::Ptr& state = nullptr);

  ~RosMissionServer() override { halt(); }

  /** \brief Callback when a new goal is in a waiting state */
  void goalWaiting(GoalHandle gh) override;
  /** \brief Callback when a new goal is accepted */
  void goalAccepted(GoalHandle gh) override;
  /**
   * \brief Callback when a new goal is rejected
   * \todo (yuchen) do we need this function?
   */
  void goalRejected(GoalHandle) override {}
  /** \brief Callback when the current goal completes successfully */
  void goalSucceeded() override;
  /**
   * \brief Callback when the current goal terminates due to an internal error
   */
  void goalAborted(const std::string& msg) override;
  /** \brief Callback when an existing goal is cancelled by a user */
  void goalCanceled(GoalHandle goal) override;
  /**
   * \brief Callback when the state machine changes state
   * \todo (yuchen) publish state
   */
  void stateChanged(const state::BaseState::Ptr&) {}
  /** \brief Callback when the state machine registers progress on a goal */
  void stateUpdate(double percent_complete);

  /** \brief Kill all goals and pause the server
   */
  virtual void halt() {
    Parent::halt();
    _publishStatus();
  }

 protected:
  /** \brief Callback when a goal is finished waiting */
  virtual void finishAccept(GoalHandle gh);
  /** \brief Callback when a goal is finished waiting at the end */
  virtual void finishSuccess(GoalHandle gh);

 private:
  /** \brief ROS-specific new goal callback */
  /// void _goalCallback(GoalHandle gh);
  rclcpp_action::GoalResponse _handleGoal(
      const typename Iface::Id& uuid,
      std::shared_ptr<const Mission::Goal> goal);
  void _handleAccepted(GoalHandle gh);
  rclcpp_action::CancelResponse _handleCancel(GoalHandle gh);
#if 0
  /** \brief ROS-specific goal reordering service callback */
  /// bool _reorderCallback(GoalReorder::Request& request,
  ///                       GoalReorder::Response& response);
  bool _reorderCallback(std::shared_ptr<GoalReorder::Request> request,
                        std::shared_ptr<GoalReorder::Request> response);
#endif
  /** \brief ROS-specific pause service callback */
  /// bool _pauseCallback(MissionPause::Request& request,
  ///                     MissionPause::Response& response);
  void _pauseCallback(std::shared_ptr<MissionPause::Request> request,
                      std::shared_ptr<MissionPause::Response> response);
#if 0
  /** \brief ROS-specific callback for mission commands */
  /// bool _cmdCallback(MissionCmd::Request& request,
  ///                   MissionCmd::Response& response);
  bool _cmdCallback(std::shared_ptr<MissionCmd::Request> request,
                    std::shared_ptr<MissionCmd::Response> response);
#endif
  /** \brief ROS-specific status message */
  /// void _publishStatus(const ros::TimerEvent& e = ros::TimerEvent());
  void _publishStatus();

  /** \brief ROS-specific feedback to ActionClient */
  void _publishFeedback(const Iface::Id& id);
  /** \brief Update the cached feedback messages */
  void _setFeedback(const Iface::Id& id, bool waiting = false,
                    double percentComplete = -1);
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
  /// ros::NodeHandle nh_;
  std::shared_ptr<rclcpp::Node> node_;

  /** \brief Service server for pausing mission execution */
  /// ros::ServiceServer pauseService_;
  rclcpp::Service<MissionPause>::SharedPtr pause_service_;
#if 0
  /** \brief Service server for reordering existing goals */
  /// ros::ServiceServer reorderService_;
  rclcpp::Service<GoalReorder>::SharedPtr reorder_service_;

  /** \brief Service server for mission commands */
  /// ros::ServiceServer cmdService_;
  rclcpp::Service<MissionCmd>::SharedPtr cmd_service_;
#endif
  /** \brief Publish intermittent status updates */
  /// ros::Publisher statusPublisher_;
  rclcpp::Publisher<MissionStatus>::SharedPtr status_publisher_;
#if 0
  /** \brief Republish commands to be logged for replay */
  /// ros::Publisher uiPublisher_;
  rclcpp::Publisher<UILog>::SharedPtr ui_publisher_;
#endif
  /** \brief Timer to ensure we always send a heartbeat */
  /// ros::Timer statusTimer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  /** \brief Action server that manages communication */
  /// actionlib::ActionServer<vtr_planning::MissionAction> actionServer_;
  rclcpp_action::Server<Mission>::SharedPtr action_server_;

  /** */
  std::future<void> cancel_goal_future_;

  /**
   * \brief Stored feedback messages to prevent passing things around too much
   */
  std::map<Iface::Id, std::shared_ptr<Mission::Feedback>> feedback_;
};

}  // namespace mission_planning
}  // namespace vtr