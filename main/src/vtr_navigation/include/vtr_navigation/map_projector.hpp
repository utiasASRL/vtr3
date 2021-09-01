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
 * \file map_projector.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <proj.h>

#include "rclcpp/rclcpp.hpp"

#include <steam.hpp>

#include <vtr_common/rosutils/transformations.hpp>
#include <vtr_common/utils/thread_pool.hpp>
#include <vtr_path_planning/planning_interface.hpp>
#include <vtr_pose_graph/evaluator/common.hpp>
#include <vtr_pose_graph/index/callback_interface.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_pose_graph/relaxation/pgr_vertex_pin_prior.hpp>
#include <vtr_pose_graph/relaxation/pose_graph_relaxation.hpp>
#include <vtr_pose_graph/relaxation/privileged_frame.hpp>
#include <vtr_tactic/publisher_interface.hpp>

#include <vtr_messages/msg/graph_global_edge.hpp>
#include <vtr_messages/msg/graph_global_vertex.hpp>
#include <vtr_messages/msg/graph_map_info.hpp>
#include <vtr_messages/msg/graph_update.hpp>
#include <vtr_messages/msg/robot_status.hpp>
#include <vtr_messages/srv/graph_calibration.hpp>
#include <vtr_messages/srv/graph_pinning.hpp>
#include <vtr_messages/srv/graph_relaxation.hpp>

#define ANGLE_NOISE -M_PI / 16.0 / 6.0
#define LINEAR_NOISE 0.2 / 6.0

namespace vtr {
namespace navigation {

using pose_graph::RCEdge;
using pose_graph::RCGraph;
using pose_graph::RCGraphBase;
using pose_graph::RCRun;
using pose_graph::RCVertex;
using pose_graph::VertexId;

// ROS related messages and services
using RobotStatusMsg = vtr_messages::msg::RobotStatus;
using ComponentMsg = vtr_messages::msg::GraphComponent;
using VertexMsg = vtr_messages::msg::GraphGlobalVertex;
using EdgeMsg = vtr_messages::msg::GraphGlobalEdge;
using UpdateMsg = vtr_messages::msg::GraphUpdate;
using MapInfoMsg = vtr_messages::msg::GraphMapInfo;
using GraphPinMsg = vtr_messages::msg::GraphPin;
using GraphCalibSrv = vtr_messages::srv::GraphCalibration;
using GraphPinningSrv = vtr_messages::srv::GraphPinning;
using GraphRelaxSrv = vtr_messages::srv::GraphRelaxation;

class MapProjector
    : public virtual pose_graph::CallbackInterface<RCVertex, RCEdge, RCRun> {
 public:
  using Base = pose_graph::CallbackInterface<RCVertex, RCEdge, RCRun>;

  using RunPtr = RCGraph::RunPtr;
  using VertexPtr = RCGraph::VertexPtr;
  using EdgePtr = RCGraph::EdgePtr;
  using EdgeIdType = RCGraph::EdgeIdType;
  using TransformType = RCGraph::TransformType;

  using GraphPtr = RCGraph::Ptr;
  using GraphWeakPtr = RCGraph::WeakPtr;
  using GraphBasePtr = RCGraphBase::Ptr;

  using ProjectionType = std::function<void(VertexMsg&)>;
  using RobotProjectionType =
      std::function<std::vector<double>(const VertexId&, const TransformType&)>;
  using TfMapType = std::unordered_map<VertexId, TransformType>;
  using MsgMapType = std::unordered_map<VertexId, VertexMsg>;

  using MutexPtr = std::shared_ptr<std::mutex>;

  // using SolverType = steam::LevMarqGaussNewtonSolver;
  using SolverType = steam::DoglegGaussNewtonSolver;

  PTR_TYPEDEFS(MapProjector)

  MapProjector(const GraphPtr& graph, const rclcpp::Node::SharedPtr node);

  void setPublisher(tactic::PublisherInterface* pub) { publisher_ = pub; }

  /** \brief Callback for a new run */
  void runAdded(const RunPtr&) override;
  /** \brief Callback for a new vertex */
  void vertexAdded(const VertexPtr&) override;
  /** \brief Callback for a new edge */
  void edgeAdded(const EdgePtr& e) override;

  /** \brief */
  void projectRobot(const tactic::Localization& persistent_loc,
                    const tactic::Localization& target_loc,
                    RobotStatusMsg& msg);

  /** \brief Updates the cached map projection */
  void updateProjection();
  /** \brief Updates the cached graph relaxation */
  void updateRelaxation() override;

 private:
  /** \brief Callback for graph relaxation service */
  void relaxGraphCallback(GraphRelaxSrv::Request::SharedPtr request,
                          GraphRelaxSrv::Response::SharedPtr response);

  /** \brief Callback for calibration update service */
  void updateCalibCallback(GraphCalibSrv::Request::SharedPtr request,
                           GraphCalibSrv::Response::SharedPtr);

  /** \brief Callback for graph pinning service */
  void pinGraphCallback(GraphPinningSrv::Request::SharedPtr request,
                        GraphPinningSrv::Response::SharedPtr);

  /** \brief Unique sequence ID generator */
  uint32_t nextSeq() {
    change_lock_.lock();
    uint32_t rval = seq_++;
    change_lock_.unlock();
    return rval;
  }

  /** \brief Helper to get a shred pointer to the graph */
  GraphPtr getGraph() {
    auto shared_graph = graph_.lock();
    if (!shared_graph) {
      throw std::runtime_error(
          "[getGraph] Attempted to use relaxation on an expired graph!");
    }
    return shared_graph;
  }

  /** \brief Initialize the message/transform maps for non-empty graphs */
  void initPoses();

  /** \brief Builds the map projection from SE(3) --> SE(2) */
  void buildProjection();

  /** \brief Incrementally relax an edge into the graph, if possible */
  bool incrementalRelax(const EdgePtr& e);

  /** \brief Graph that generates the callbacks */
  GraphWeakPtr graph_;

  /** \brief Working copy of the manual subset of the graph */
  GraphBasePtr working_graph_;

  /** \brief ROS node handle for publishing */
  std::shared_ptr<rclcpp::Node> node_;
  /** \brief Publishes structural edge updates */
  rclcpp::Publisher<EdgeMsg>::SharedPtr edge_updates_;
  /** \brief Publishes updates to the relaxed graph */
  rclcpp::Publisher<UpdateMsg>::SharedPtr graph_updates_;
  /** \brief Service to request a relaxed version of the graph */
  rclcpp::Service<GraphRelaxSrv>::SharedPtr relaxation_service_;
  /** \brief Service to move graph (rotation, translation, scale) */
  rclcpp::Service<GraphCalibSrv>::SharedPtr calibration_service_;
  /** \brief Service to set vertex to lat lng correspendences, a.k.a pins */
  rclcpp::Service<GraphPinningSrv>::SharedPtr graph_pinning_service_;

  tactic::PublisherInterface* publisher_;

  /** \brief Cached response to avoid recomputation on every request */
  GraphRelaxSrv::Response cached_response_;

  /** \brief Indicates whether or not the current relaxation is valid */
  bool relaxation_valid_ = false;

  /** \brief Indicates whether or not the current projection is valid */
  bool projection_valid_ = false;

  /** \brief Root vertex for relaxation */
  VertexId root_ = VertexId(0, 0);

  /** \brief Dynamically generated projection function for graph*/
  ProjectionType project_;

  /** \brief Dynamically generated projection function for live robot pose */
  RobotProjectionType project_robot_;

  /** \brief The PROJ string defining what projection is required */
  static const std::string pj_str_;

  /** \brief PJ object dynamically allocated */
  PJ* pj_utm_ = nullptr;

  /** \brief Keep access to project_ and project_robot_ thread safe */
  std::mutex project_mutex_;

  /** \brief Cached transform map to bootstrap incremental relaxation */
  TfMapType tf_map_;

  /** \brief Cached vertex message map */
  MsgMapType msg_map_;

  /** \brief Keep the tf_map_, msg_map_, seq_ thread safe */
  std::recursive_mutex change_lock_;

  /** \brief Unique message sequence */
  uint32_t seq_ = 0;

  /** \brief Concurrent thread pool for background relaxation */
  common::thread_pool pool_;

  /** \brief Default map to use when we have no config */
  MapInfoMsg default_map_;

  /**
   * \brief Cached robot persistent & target localization used after graph
   * relaxation and calibration
   */
  tactic::Localization cached_persistent_loc_;
  tactic::Localization cached_target_loc_;
};

}  // namespace navigation
}  // namespace vtr