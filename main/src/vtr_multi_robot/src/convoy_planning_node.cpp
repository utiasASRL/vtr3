// Copyright 2025, Autonomous Space Robotics Lab (ASRL)
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
 * \file convoy_planning_node.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_navigation_msgs/msg/goal_handle.hpp"
#include "vtr_navigation_msgs/msg/mission_command.hpp"
#include "vtr_navigation_msgs/msg/robot_state.hpp"
#include "vtr_navigation_msgs/msg/server_state.hpp"
#include "vtr_navigation_msgs/srv/robot_state.hpp"
#include "vtr_route_planning/route_planning.hpp"
#include "vtr_tactic/types.hpp"

using namespace std::chrono_literals;
using namespace vtr::route_planning;

namespace vtr::multi_robot {

class ConvoyPlanningNode : public rclcpp::Node {
 public:
  using MissionCommand = vtr_navigation_msgs::msg::MissionCommand;
  using Goal = vtr_navigation_msgs::msg::GoalHandle;
  using GraphPath = pose_graph::Path<tactic::Graph>;
  using RobotState = vtr_navigation_msgs::msg::RobotState;
  using ServerState = vtr_navigation_msgs::msg::ServerState;
  using RobotStateSrv = vtr_navigation_msgs::srv::RobotState;
  using FutureRobotState = rclcpp::Client<RobotStateSrv>::SharedFuture;

  ConvoyPlanningNode() : Node("convoy_planning_node") {
    logging::configureLogging("", true,
                              declare_parameter<std::vector<std::string>>(
                                  "log_enabled", std::vector<std::string>{}));

    subs_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = subs_cb_group_;

    client_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Parameters
    std::vector<std::string> robot_names =
        declare_parameter<std::vector<std::string>>("robot_names");

    if (robot_names.size() != 2) {
      CLOG(ERROR, "convoy_command")
          << "Convoy is only implemented for 2 robots so far.";
      throw std::runtime_error(
          "Convoy is only implemented for 2 robots so far.");
    }

    const auto data_dir = declare_parameter<std::string>("data_dir");
    graph_ = tactic::Graph::MakeShared(
        data_dir + "/graph", true, std::make_shared<tactic::Graph::Callback>(),
        true);
    route_planner_ = std::make_shared<BFSPlanner>(graph_);
    active_path_ = std::make_shared<GraphPath>(graph_);

    // Publisher: status updates for the convoy
    for (const auto& robot : robot_names) {
      CLOG(DEBUG, "convoy_route")
          << "Robot topic " << ("/" + robot + "/vtr/robot_state_srv");
      robots.push_back({robot,
                        create_publisher<MissionCommand>(
                            "/" + robot + "/vtr/mission_command", 10),
                        create_client<RobotStateSrv>(
                            "/" + robot + "/vtr/robot_state_srv",
                            rmw_qos_profile_services_default, client_cb_group_),
                        create_subscription<ServerState>(
                            "/" + robot + "/vtr/server_state", 10,
                            [this, robot](const ServerState::SharedPtr msg) {
                              this->onServerState(robot, msg);
                            },
                            sub_options)});
    }

    // Subscriber: example command topic for convoy control
    command_sub_ = this->create_subscription<MissionCommand>(
        "convoy_command", 10,
        std::bind(&ConvoyPlanningNode::onCommand, this, std::placeholders::_1),
        sub_options);
  }

 private:
  struct RobotInfo {

    RobotInfo(const std::string& name0, rclcpp::Publisher<MissionCommand>::SharedPtr mission_pub0,
      rclcpp::Client<RobotStateSrv>::SharedPtr state_srv0, rclcpp::Subscription<ServerState>::SharedPtr server_sub0) :
         name{name0}, mission_pub{mission_pub0}, state_srv{state_srv0}, server_sub{server_sub0} {}
    

    void selectController() {
      MissionCommand select_controller;
      select_controller.type = MissionCommand::ADD_GOAL;
      select_controller.goal_handle.type = Goal::SELECT_CONTROLLER;
      select_controller.goal_handle.controller_name =
          (isLeader) ? "bicycle_mpc" : "bicycle_mpc_follower";
      mission_pub->publish(select_controller);
      CLOG(DEBUG, "convoy_route")
          << "Publishing controller "
          << select_controller.goal_handle.controller_name << " to " << name;
      std::this_thread::sleep_for(10ms);
    }

    void sendPause() {
      MissionCommand pause;
      pause.type = MissionCommand::ADD_GOAL;
      pause.goal_handle.type = Goal::PAUSE;
      mission_pub->publish(pause);
      std::this_thread::sleep_for(10ms);
    }

    void sendUnpause() {
      MissionCommand unpause;
      unpause.type = MissionCommand::CANCEL_GOAL;
      unpause.goal_handle.id = current_id_;
      mission_pub->publish(unpause);
      std::this_thread::sleep_for(10ms);
    }

    void setLeader() { isLeader = true; }
    void setFollower() { isLeader = false; }

    void onServerUpdate(const ServerState::SharedPtr& msg) {
      paused = (msg->server_state == ServerState::PROCESSING && msg->goals.size() > 0 && msg->goals[0].type == Goal::PAUSE);
      if (paused) {
        current_id_ = msg->current_goal_id;
      } else {
        current_id_.fill(0);
      }
    }

    std::string name;
    rclcpp::Publisher<MissionCommand>::SharedPtr mission_pub;
    rclcpp::Client<RobotStateSrv>::SharedPtr state_srv;
    rclcpp::Subscription<ServerState>::SharedPtr server_sub;
    bool isLeader = false;    
    bool paused = false;

    private:
      std::array<uint8_t, 16> current_id_;
  };

  void onCommand(const MissionCommand::SharedPtr msg) {
    CLOG(INFO, "convoy_route") << "Received msg";
    if (msg->type != MissionCommand::ADD_GOAL ||
        msg->goal_handle.type != Goal::REPEAT) {
      CLOG(INFO, "convoy_route") << "Invalid message type";
      return;
    }

    std::list<FutureRobotState> robot_futures;

    // Resize destination_vector to hold the transformed elements
    robot_futures.resize(robots.size());

    std::transform(robots.begin(), robots.end(), robot_futures.begin(),
                   [](RobotInfo& r) {
                     return r.state_srv->async_send_request(
                         std::make_shared<RobotStateSrv::Request>());
                   });

    std::list<uint64_t> sequence_ids;
    const std::list<tactic::VertexId> waypoints(
        msg->goal_handle.waypoints.begin(), msg->goal_handle.waypoints.end());
    tactic::PathType combined_path =
        route_planner_->path(msg->vertex, waypoints, sequence_ids);
    active_path_->setSequence(combined_path);

    tactic::PathType direction_switches;

    for (GraphPath::Iterator it = active_path_->begin(1);
         it != active_path_->end(); ++it) {
      if (areVectorsOpposite(it->T(), (it - 1)->T())) {
        CLOG(DEBUG, "convoy_route")
            << "Found a direction switch at " << it->from();
        direction_switches.push_back(it->from());
      }
    }

    const auto status_a = robot_futures.front().wait_for(1s);
    const auto status_b = robot_futures.back().wait_for(1s);
    if (status_a != std::future_status::ready ||
        status_b != std::future_status::ready) {
      CLOG(ERROR, "convoy_route") << "Failed to get the services in 1 second";
      return;
    }

    const auto& robot_a_state = robot_futures.front().get()->robot_state;
    const auto& robot_b_state = robot_futures.back().get()->robot_state;
    tactic::PathType inter_robot_vertices =
        route_planner_->path(static_cast<uint64_t>(robot_a_state.index),
                             static_cast<uint64_t>(robot_b_state.index));
    GraphPath inter_robot_path(graph_);
    inter_robot_path.setSequence(inter_robot_vertices);

    if (areVectorsOpposite(inter_robot_path.begin(1)->T(),
                           active_path_->begin(1)->T())) {
      CLOG(DEBUG, "convoy_route")
          << "Robot " << robots.front().name << " is the leader";
      robots.front().setLeader();
      robots.back().setFollower();
    } else {
      CLOG(DEBUG, "convoy_route")
          << "Robot " << robots.back().name << " is the leader";
      robots.front().setFollower();
      robots.back().setLeader();
    }



    for (const auto& cusp_vertex : direction_switches) {
      MissionCommand route_section;
      route_section.type = MissionCommand::ADD_GOAL;
      route_section.goal_handle.type = Goal::REPEAT;
      route_section.goal_handle.waypoints.push_back(cusp_vertex);
      std::for_each(
          robots.begin(), robots.end(), [route_section](RobotInfo& x) {
            x.selectController();
            x.isLeader = !x.isLeader;
            CLOG(DEBUG, "convoy_route") << "Publishing route to " << x.name;
            x.mission_pub->publish(route_section);
            std::this_thread::sleep_for(10ms);
            x.sendPause();
          });
    }

    MissionCommand final_goal;
    final_goal.type = MissionCommand::ADD_GOAL;
    final_goal.goal_handle.type = Goal::REPEAT;
    final_goal.goal_handle.waypoints.push_back(waypoints.back());
    std::for_each(robots.begin(), robots.end(), [final_goal](RobotInfo& x) {
      x.selectController();
      CLOG(DEBUG, "convoy_route") << "Publishing route to " << x.name;
      x.mission_pub->publish(final_goal);
      std::this_thread::sleep_for(10ms);
    });
  }

  void onServerState(const std::string robot_name,
                     const ServerState::SharedPtr& msg) {
    for (auto& robot : robots) {
      if (robot.name == robot_name) {
        robot.onServerUpdate(msg);
        break;
      }
    }
    tryUnpause();
  }

  void tryUnpause() {
    if (std::all_of(robots.begin(), robots.end(),
                    [](RobotInfo& r) { return r.paused; })) {
      CLOG(DEBUG, "convoy_route") << "All robots paused. Unpausing.";
      std::for_each(robots.begin(), robots.end(),
                    [](RobotInfo& r) { r.sendUnpause(); });
    }
  }

  bool areVectorsOpposite(const tactic::EdgeTransform& a,
                          const tactic::EdgeTransform& b) {
    Eigen::Matrix<double, 6, 1> vec_cur = a.vec();
    Eigen::Matrix<double, 6, 1> vec_last = b.vec();

    double r_dot = vec_last.head<3>().dot(vec_cur.head<3>());
    return (r_dot < 0);
  }

  // Members
  std::list<RobotInfo> robots;
  rclcpp::Subscription<MissionCommand>::SharedPtr command_sub_;
  tactic::Graph::Ptr graph_;
  BFSPlanner::Ptr route_planner_;
  GraphPath::Ptr active_path_;

  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr subs_cb_group_;
};

}  // namespace vtr::multi_robot

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<vtr::multi_robot::ConvoyPlanningNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}