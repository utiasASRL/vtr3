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

#include "vtr_tactic/types.hpp"

#include "vtr_navigation_msgs/msg/graph_route.hpp"
#include "vtr_navigation_msgs/msg/graph_state.hpp"
#include "vtr_navigation_msgs/msg/move_graph.hpp"
#include "vtr_navigation_msgs/srv/graph_state.hpp"

namespace vtr {
namespace navigation {

class GraphMapServer {
 public:
  PTR_TYPEDEFS(GraphMapServer);

  using GraphRoute = vtr_navigation_msgs::msg::GraphRoute;
  using GraphVertex = vtr_navigation_msgs::msg::GraphVertex;
  using GraphState = vtr_navigation_msgs::msg::GraphState;

  using GraphStateSrv = vtr_navigation_msgs::srv::GraphState;
  using MoveGraphMsg = vtr_navigation_msgs::msg::MoveGraph;

#if false
  using RunPtr = tactic::Graph::RunPtr;
  using VertexPtr = tactic::Graph::VertexPtr;
  using EdgePtr = tactic::Graph::EdgePtr;
  using EdgeId = tactic::Graph::EdgeIdType;
#endif
  using VertexId = tactic::Graph::VertexIdType;
  using Transform = tactic::Graph::TransformType;

  using GraphPtr = tactic::Graph::Ptr;
  using GraphWeakPtr = tactic::Graph::WeakPtr;

  using VertexId2TransformMap = std::unordered_map<VertexId, Transform>;
  using ProjectVertex =
      std::function<std::tuple<double, double, double>(const VertexId&)>;
#if false
  using ProjectRobot =
      std::function<std::vector<double>(const VertexId&, const Transform&)>;
  using MsgMapType = std::unordered_map<VertexId, VertexMsg>;
#endif
  using Mutex = std::mutex;
  using UniqueLock = std::unique_lock<Mutex>;
  using SharedLock = std::shared_lock<Mutex>;
  using LockGuard = std::lock_guard<Mutex>;

  void start(const rclcpp::Node::SharedPtr& node, const GraphPtr& graph);

 private:
  void graphStateSrvCallback(
      const std::shared_ptr<GraphStateSrv::Request>,
      std::shared_ptr<GraphStateSrv::Response> response) const;

  void moveGraphCallback(const MoveGraphMsg::ConstSharedPtr msg);

 private:
  void computeGraphState();

  /** \brief Helper to get a shared pointer to the graph */
  GraphPtr getGraph() {
    if (auto graph_acquired = graph_.lock())
      return graph_acquired;
    else {
      std::string err{"Graph has expired"};
      CLOG(WARNING, "navigation.map_projector") << err;
      throw std::runtime_error(err);
    }
    return nullptr;
  }

  /** \brief Graph that generates the callbacks */
  GraphWeakPtr graph_;

  /** \brief Cached transform map to bootstrap incremental relaxation */
  VertexId2TransformMap vid2tf_map_;
  /** \brief Cached response to avoid recomputation on every request */
  GraphState graph_state_;

#if false
  /**
   * \brief Cached robot persistent & target localization used after graph
   * relaxation and calibration
   */
  tactic::Localization robot_persistent_loc_;
  tactic::Localization robot_target_loc_;
  RobotState robot_state_;
#endif
  /** \brief The PROJ string defining what projection is required */
  inline static const std::string pj_str_ =
      "+proj=utm +ellps=WGS84 +datum=WGS84 +units=m +no_defs +zone=";
  /** \brief PJ object dynamically allocated */
  PJ* pj_utm_ = nullptr;
  /** \brief Dynamically generated projection function for graph*/
  ProjectVertex project_vertex_;
#if false
  /** \brief Dynamically generated projection function for live robot pose */
  ProjectRobot project_robot_;
#endif
  /** \brief Keep access to project_ and project_robot_ thread safe */
  Mutex mutex_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
#if false
  /** \brief Publishes updates to the relaxed graph */
  rclcpp::Publisher<GraphUpdateMsg>::SharedPtr graph_update_pub_;
  /** \brief Publishes updates to the relaxed graph */
  rclcpp::Publisher<GraphStateMsg>::SharedPtr graph_state_pub_;
#endif
  /** \brief Service to request a relaxed version of the graph */
  rclcpp::Service<GraphStateSrv>::SharedPtr graph_state_srv_;
#if false
  rclcpp::Publisher<RobotState>::SharedPtr robot_state_pub_;
#endif

  /** \brief subscription to move graph (rotation, translation, scale) */
  rclcpp::Subscription<MoveGraphMsg>::SharedPtr move_graph_sub_;
};

}  // namespace navigation
}  // namespace vtr