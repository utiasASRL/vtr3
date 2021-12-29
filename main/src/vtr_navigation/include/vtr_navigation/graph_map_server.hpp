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
 * \file graph_map_server.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <proj.h>

#include "rclcpp/rclcpp.hpp"

#include "vtr_tactic/rviz_tactic_callback.hpp"
#include "vtr_tactic/tactic.hpp"
#include "vtr_tactic/types.hpp"

#include "vtr_navigation_msgs/msg/annotate_route.hpp"
#include "vtr_navigation_msgs/msg/graph_route.hpp"
#include "vtr_navigation_msgs/msg/graph_state.hpp"
#include "vtr_navigation_msgs/msg/graph_update.hpp"
#include "vtr_navigation_msgs/msg/move_graph.hpp"
#include "vtr_navigation_msgs/msg/robot_state.hpp"
#include "vtr_navigation_msgs/srv/following_route.hpp"
#include "vtr_navigation_msgs/srv/graph_state.hpp"
#include "vtr_navigation_msgs/srv/robot_state.hpp"

namespace vtr {
namespace navigation {

class GraphMapServer : public tactic::Graph::Callback,
                       virtual public tactic::Tactic::Callback {
 public:
  PTR_TYPEDEFS(GraphMapServer);

  using GraphRoute = vtr_navigation_msgs::msg::GraphRoute;
  using GraphVertex = vtr_navigation_msgs::msg::GraphVertex;
  using GraphState = vtr_navigation_msgs::msg::GraphState;
  using GraphUpdate = vtr_navigation_msgs::msg::GraphUpdate;
  using GraphStateSrv = vtr_navigation_msgs::srv::GraphState;

  using RobotState = vtr_navigation_msgs::msg::RobotState;
  using RobotStateSrv = vtr_navigation_msgs::srv::RobotState;

  using FollowingRoute = vtr_navigation_msgs::msg::GraphRoute;
  using FollowingRouteSrv = vtr_navigation_msgs::srv::FollowingRoute;

  using MoveGraphMsg = vtr_navigation_msgs::msg::MoveGraph;
  using AnnotateRouteMsg = vtr_navigation_msgs::msg::AnnotateRoute;

  using VertexPtr = tactic::Graph::VertexPtr;
  using EdgePtr = tactic::Graph::EdgePtr;
  using RunPtr = tactic::Graph::RunPtr;
  using EdgeId = tactic::Graph::EdgeIdType;
  using VertexId = tactic::Graph::VertexIdType;
  using Transform = tactic::Graph::TransformType;

  using GraphPtr = tactic::Graph::Ptr;
  using GraphWeakPtr = tactic::Graph::WeakPtr;
  using GraphBasePtr = tactic::GraphBase::Ptr;

  using VertexId2TransformMap = std::unordered_map<VertexId, Transform>;
  using VertexId2IdxMap = std::unordered_map<VertexId, size_t>;
  using ProjectVertex =
      std::function<std::tuple<double, double, double>(const VertexId&)>;
  using ProjectRobot = std::function<std::tuple<double, double, double>(
      const VertexId&, const Transform&)>;

  using Mutex = std::shared_mutex;
  using UniqueLock = std::unique_lock<Mutex>;
  using SharedLock = std::shared_lock<Mutex>;

  void start(const rclcpp::Node::SharedPtr& node, const GraphPtr& graph);

 private:
  /// these functions, if necessary, must lock graph first then internal lock
  void graphStateSrvCallback(
      const std::shared_ptr<GraphStateSrv::Request>,
      std::shared_ptr<GraphStateSrv::Response> response) const;
  void robotStateSrvCallback(
      const std::shared_ptr<RobotStateSrv::Request>,
      std::shared_ptr<RobotStateSrv::Response> response) const;
  void followingRouteSrvCallback(
      const std::shared_ptr<FollowingRouteSrv::Request>,
      std::shared_ptr<FollowingRouteSrv::Response> response) const;
  void moveGraphCallback(const MoveGraphMsg::ConstSharedPtr msg);
  void annotateRouteCallback(const AnnotateRouteMsg::ConstSharedPtr msg);

 private:
  /// these functions are called with graph mutex locked
  void vertexAdded(const VertexPtr& v) override;
  void edgeAdded(const EdgePtr& e) override;

 private:
  void endRun() override;
  void robotStateUpdated(const tactic::Localization& persistent,
                         const tactic::Localization& target) override;
  void pathUpdated(const VertexId::Vector& path) override;

 private:
  /// these functions are called by functions above, do not lock mutex inside
  /** \brief Helper to get a shared pointer to the graph */
  GraphPtr getGraph() const;
  /** \brief Returns a privileged graph (only contains teach routes) */
  GraphBasePtr getPrivilegedGraph() const;
  /** \brief Compute graph in a privileged frame, changes vid2tf_map_ */
  void optimizeGraph(const GraphBasePtr& priv_graph);
  void updateVertexProjection();
  void updateVertexType();
  void computeRoutes(const GraphBasePtr& priv_graph);
  /** \brief Update the graph incrementally when no optimization is needed */
  bool updateIncrementally(const EdgePtr& e);

  void updateRobotProjection();

 private:
  /** \brief Graph that generates the callbacks */
  GraphWeakPtr graph_;

  /** \brief Protects all class member accesses */
  mutable Mutex mutex_;

  /** \brief Cached T_vertex_root transform */
  VertexId2TransformMap vid2tf_map_;
  /** \brief VertexId to its index in graph_state_.vertices */
  VertexId2IdxMap vid2idx_map_;
  /** \brief Vertices and routes */
  GraphState graph_state_;

  /**
   * \brief Cached robot persistent & target localization used after graph
   * relaxation and calibration
   */
  tactic::Localization robot_persistent_loc_;
  tactic::Localization robot_target_loc_;
  RobotState robot_state_;

  /** \brief Cached current route being followed by the robot */
  GraphRoute following_route_;

  /** \brief PJ object dynamically allocated */
  PJ* pj_utm_ = nullptr;
  /** \brief Dynamically generated projection function for graph*/
  ProjectVertex project_vertex_ = nullptr;
  /** \brief Dynamically generated projection function for live robot pose */
  ProjectRobot project_robot_ = nullptr;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  /** \brief Publishes updates to the relaxed graph */
  rclcpp::Publisher<GraphUpdate>::SharedPtr graph_update_pub_;
  /** \brief Publishes updates to the relaxed graph */
  rclcpp::Publisher<GraphState>::SharedPtr graph_state_pub_;
  /** \brief Service to request a relaxed version of the graph */
  rclcpp::Service<GraphStateSrv>::SharedPtr graph_state_srv_;

  rclcpp::Publisher<RobotState>::SharedPtr robot_state_pub_;
  rclcpp::Service<RobotStateSrv>::SharedPtr robot_state_srv_;

  /** \brief Publishes current route being followed */
  rclcpp::Publisher<FollowingRoute>::SharedPtr following_route_pub_;
  rclcpp::Service<FollowingRouteSrv>::SharedPtr following_route_srv_;

  /** \brief subscription to move graph (rotation, translation, scale) */
  rclcpp::Subscription<MoveGraphMsg>::SharedPtr move_graph_sub_;
  rclcpp::Subscription<AnnotateRouteMsg>::SharedPtr annotate_route_sub_;
};

class RvizGraphMapServer : public GraphMapServer,
                           public tactic::RvizTacticCallback {
 public:
  RvizGraphMapServer(const rclcpp::Node::SharedPtr& node)
      : tactic::RvizTacticCallback(node) {}
};

}  // namespace navigation
}  // namespace vtr