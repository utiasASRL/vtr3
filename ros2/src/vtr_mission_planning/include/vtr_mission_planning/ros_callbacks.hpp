#pragma once

#include <proj.h>  // #include <proj_api.h>

#include "rclcpp/rclcpp.hpp"  /// #include <ros/ros.h>

#if 0
#include <std_msgs/Empty.h>
#include <future>
#endif

#include <steam/solver/DoglegGaussNewtonSolver.hpp>

#include <vtr_path_planning/planning_interface.hpp>
#include <vtr_pose_graph/index/callback_interface.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_pose_graph/relaxation/pose_graph_relaxation.hpp>
#if 0
#include <vtr_pose_graph/relaxation/GpsVertexPrior.hpp>
#endif

#include <vtr_common/utils/thread_pool.hpp>
#include <vtr_messages/msg/graph_global_edge.hpp>
#include <vtr_messages/msg/graph_global_vertex.hpp>
#include <vtr_messages/msg/graph_map_info.hpp>
#include <vtr_messages/msg/graph_update.hpp>
#include <vtr_messages/srv/graph_relaxation.hpp>
#if 0
#include <asrl__planning/Overlay.h>
#include <asrl__pose_graph/CalibrationService.h>
#include <asrl__pose_graph/GlobalGraph.h>
#include <asrl__pose_graph/GraphComponent.h>
#include <babelfish_robochunk_translator/BabelfishMessage.h>
#include <ros/node_handle.h>
#include <asrl/common/rosutil/param.hpp>
#include <asrl/common/utils/CommonMacros.hpp>
#endif

namespace vtr {
namespace mission_planning {

using vtr::pose_graph::RCEdge;
using vtr::pose_graph::RCGraph;
using vtr::pose_graph::RCGraphBase;
using vtr::pose_graph::RCRun;
using vtr::pose_graph::RCVertex;
using vtr::pose_graph::VertexId;

// template<class GRAPH>
class RosCallbacks
    : public virtual pose_graph::CallbackInterface<RCVertex, RCEdge, RCRun> {
 public:
  using Base = pose_graph::CallbackInterface<RCVertex, RCEdge, RCRun>;

  using RunPtr = RCGraph::RunPtr;
  using EdgePtr = RCGraph::EdgePtr;
  using VertexPtr = RCGraph::VertexPtr;
  using TransformType = RCGraph::TransformType;

  using EdgeIdType = RCGraph::EdgeIdType;

  using GraphPtr = RCGraph::Ptr;
  using GraphWeakPtr = RCGraph::WeakPtr;

  // This looks ugly, but it lets us determine the base pointer type for any
  // graph
  using GraphBasePtr = RCGraphBase::Ptr;

  // ROS related messages and services
  // using GraphSrv = asrl__pose_graph::RelaxationService;
  using ComponentMsg = vtr_messages::msg::GraphComponent;
  using VertexMsg = vtr_messages::msg::GraphGlobalVertex;
  using EdgeMsg = vtr_messages::msg::GraphGlobalEdge;
  using UpdateMsg = vtr_messages::msg::GraphUpdate;
  using MapInfoMsg = vtr_messages::msg::GraphMapInfo;
  using GraphSrv = vtr_messages::srv::GraphRelaxation;
#if 0
  using GraphMsg = asrl__pose_graph::GlobalGraph;
  using CalibSrv = asrl__pose_graph::CalibrationService;
  using OverlaySrv = asrl__planning::Overlay;
#endif

  using ProjectionType = std::function<void(VertexMsg&)>;
  using TfMapType = std::unordered_map<VertexId, TransformType>;
  using MsgMapType = std::unordered_map<VertexId, VertexMsg>;

#if 0
  using MutexPtr = std::shared_ptr<std::mutex>;
#endif
  using PlannerPtr = path_planning::PlanningInterface::Ptr;
#if 0
  using PlannerWeakPtr = asrl::planning::PlanningInterface::WeakPtr;
#endif
  // using SolverType = steam::LevMarqGaussNewtonSolver;
  using SolverType = steam::DoglegGaussNewtonSolver;

#if 0
  static constexpr char gpsStream[] = "/DGPS/fix";
#endif

  PTR_TYPEDEFS(RosCallbacks)

  /// RosCallbacks(const GraphPtr& graph, const ros::NodeHandle& nh);
  RosCallbacks(const GraphPtr& graph, const std::shared_ptr<rclcpp::Node> node);

  /** \brief Callback for a new run */
  void runAdded(const RunPtr&) override;
  /** \brief Callback for a new vertex */
  void vertexAdded(const VertexPtr&) override;
  /** \brief Callback for a new edge */
  void edgeAdded(const EdgePtr& e) override;
  /** \brief Updates the cached graph relaxation */
  void updateRelaxation(const MutexPtr& mutex = nullptr) override;
  /** \brief set planer */
  void setPlanner(const PlannerPtr& planner) override {
#if 0
    planner_ = planner;
    overlayStatus_.publish(std_msgs::Empty());
#endif
  };

  /** \brief Updates the cached map projection */
  void updateProjection();

 private:
  /** \brief Callback for graph relaxation service */
  void _relaxGraphCallback(std::shared_ptr<GraphSrv::Request> request,
                           std::shared_ptr<GraphSrv::Response> response);
#if 0
  /** \brief Callback for calibration update service */
  bool updateCalib(CalibSrv::Request& request, CalibSrv::Response&);

  /** \brief Callback for graph overlay service */
  bool getOverlay(OverlaySrv::Request& request, OverlaySrv::Response& response);
#endif

  /** \brief Unique sequence ID generator */
  inline uint32_t _nextSeq() {
    changeLock_.lock();
    uint32_t rval = seq_++;
    changeLock_.unlock();
    return rval;
  }

  /** \brief Helper to get a shred pointer to the graph */
  inline GraphPtr _getGraph() {
    auto sharedGraph = graph_.lock();
    if (!sharedGraph) {
      throw std::runtime_error(
          "[getGraph] Attempted to use relaxation on an expired graph!");
    }
    return sharedGraph;
  }
#if 0
  /** \brief Helper to get a shred pointer to the graph */
  inline PlannerPtr _getPlanner() {
    auto sharedPlanner = planner_.lock();
    if (!sharedPlanner) {
      throw std::runtime_error(
          "[getPlanner] Attempted to get a non-existent planner!");
    }
    return sharedPlanner;
  }
#endif
  /** \brief Incrementally relax an edge into the graph, if possible */
  bool _incrementalRelax(const EdgePtr& e);

  /** \brief Initialize the message/transform maps for non-empty graphs */
  void _initPoses();

  /** \brief Builds the map projection from SE(3) --> SE(2) */
  void _buildProjection();
#if 0
  /** \brief Checks if GPS data has been added and updates the map */
  void _checkGps();
#endif
  /** \brief Graph that generates the callbacks */
  GraphWeakPtr graph_;

  /** \brief Working copy of the manual subset of the graph */
  GraphBasePtr workingGraph_;

  /** \brief ROS node handle for publishing */
  /// ros::NodeHandle nh_;
  std::shared_ptr<rclcpp::Node> node_;

  /** \brief Publishes structural edge updates */
  rclcpp::Publisher<EdgeMsg>::SharedPtr edgeUpdates_;

  /** \brief Publishes updates to the relaxed graph */
  rclcpp::Publisher<UpdateMsg>::SharedPtr graphUpdates_;
#if 0
  /** \brief Publishes a trigger when the overlay values change */
  ros::Publisher overlayStatus_;
#endif
  /** \brief Service to request a relaxed version of the graph */
  rclcpp::Service<GraphSrv>::SharedPtr relaxation_service_;

#if 0
  /** \brief Service to update the map alignment */
  ros::ServiceServer mapCalibrationServer_;

  /** \brief Service to produce scalar map overlays */
  ros::ServiceServer overlayServer_;
#endif
  /** \brief Cached response to avoid recomputation on every request */
  GraphSrv::Response cachedResponse_;

  /** \brief Indicates whether or not the current relaxation is valid */
  bool relaxationValid_;

  /** \brief Indicates whether or not the current projection is valid */
  bool projectionValid_;
#if 0
  /** \brief Flag to indicate the presence of GPS */
  bool usingGps_;
#endif
  /** \brief Root vertex for relaxation */
  VertexId root_;

  /** \brief Dynamically generated projection function */
  ProjectionType project_;

  /** \brief Cached transform map to bootstrap incremental relaxation */
  TfMapType tfMap_;

  /** \brief Cached vertex message map */
  MsgMapType msgMap_;

  /** \brief Keep things thread safe */
  std::recursive_mutex changeLock_;

  /** \brief Unique message sequence */
  uint32_t seq_;

  /** \brief Concurrent thread pool for background relaxation */
  common::thread_pool pool_;
#if 0
  /** \brief Reference to the planner object used for path planning */
  PlannerWeakPtr planner_;
#endif
  /** \brief Default map to use when we have no config */
  MapInfoMsg defaultMap_;
#if 0
  /** \brief VTR2->Translator communication publisher */
  ros::Publisher babelfish_;
#endif
};

}  // namespace mission_planning
}  // namespace vtr
