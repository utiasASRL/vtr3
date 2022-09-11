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
 * \file mission_server.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

#include "rclcpp/rclcpp.hpp"

#include "vtr_mission_planning/mission_server/mission_server.hpp"

#include "vtr_navigation_msgs/msg/mission_command.hpp"
#include "vtr_navigation_msgs/msg/server_state.hpp"
#include "vtr_navigation_msgs/srv/server_state.hpp"

namespace vtr {
namespace mission_planning {

/** \brief Template specialization to standardize the goal interface */
template <>
class GoalInterface<vtr_navigation_msgs::msg::GoalHandle> {
 public:
  using GoalHandle = vtr_navigation_msgs::msg::GoalHandle;
  using Id = boost::uuids::uuid;
  using IdHash = boost::hash<Id>;

  static const Id& InvalidId() {
    static Id invalid_id = boost::uuids::random_generator()();
    return invalid_id;
  }
  static Id id(const GoalHandle& gh) {
    boost::uuids::uuid tmp;
    std::copy(gh.id.begin(), gh.id.end(), tmp.begin());
    return tmp;
  }
  static GoalTarget target(const GoalHandle& gh) {
    switch (gh.type) {
      case GoalHandle::IDLE:
        return GoalTarget::Idle;
      case GoalHandle::TEACH:
        return GoalTarget::Teach;
      case GoalHandle::REPEAT:
        return GoalTarget::Repeat;
      default:
        return GoalTarget::Unknown;
    }
  }
  static std::list<tactic::VertexId> path(const GoalHandle& gh) {
    const auto& path = gh.waypoints;
    return std::list<tactic::VertexId>(path.begin(), path.end());
  }
  static std::chrono::milliseconds pause_before(const GoalHandle& gh) {
    return std::chrono::milliseconds(gh.pause_before);
  }
  static std::chrono::milliseconds pause_after(const GoalHandle& gh) {
    return std::chrono::milliseconds(gh.pause_after);
  }
};

}  // namespace mission_planning

namespace navigation {

class ROSMissionServer : public mission_planning::MissionServer<
                             vtr_navigation_msgs::msg::GoalHandle> {
 public:
  PTR_TYPEDEFS(ROSMissionServer);
  using GoalHandle = vtr_navigation_msgs::msg::GoalHandle;
  using GoalInterface = mission_planning::GoalInterface<GoalHandle>;
  using GoalId = GoalInterface::Id;
  using ServerState = mission_planning::ServerState;

  using MissionCommandMsg = vtr_navigation_msgs::msg::MissionCommand;
  using ServerStateMsg = vtr_navigation_msgs::msg::ServerState;
  using ServerStateSrv = vtr_navigation_msgs::srv::ServerState;

  ~ROSMissionServer() override { stop(); }

  void start(const std::shared_ptr<rclcpp::Node>& node,
             const mission_planning::StateMachineInterface::Ptr& state_machine);

  /// ROS callbacks
 private:
  void handleCommand(const MissionCommandMsg::SharedPtr);

  /// Parent class overrides, only call from parent class
 private:
  void serverStateSrvCallback(
      const std::shared_ptr<ServerStateSrv::Request>,
      std::shared_ptr<ServerStateSrv::Response> response) const;
  void serverStateChanged() override;

 private:
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<MissionCommandMsg>::SharedPtr mission_command_sub_;
  rclcpp::Publisher<ServerStateMsg>::SharedPtr server_state_pub_;
  rclcpp::Service<ServerStateSrv>::SharedPtr server_state_srv_;

 private:
  /** \brief cached server state message, protected by mission server mutex */
  ServerStateMsg server_state_msg_;
};

}  // namespace navigation
}  // namespace vtr