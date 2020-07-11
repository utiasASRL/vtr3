#pragma once

#include <ros/ros.h>

#include <asrl__pose_graph/CalibrationService.h>
#include <asrl__pose_graph/GlobalGraph.h>
#include <asrl__pose_graph/GlobalVertex.h>
#include <asrl__pose_graph/GraphComponent.h>
#include <asrl__pose_graph/GraphUpdate.h>
#include <asrl__pose_graph/IndexEdge.h>
#include <asrl__pose_graph/RelaxationService.h>
#include <asrl/planning/PlanningInterface.hpp>
#include <asrl/pose_graph/index/CallbackInterface.hpp>
#include <asrl/pose_graph/relaxation/GpsVertexPrior.hpp>
#include <asrl/pose_graph/relaxation/PoseGraphRelaxation.hpp>

#if 0
#include <future>

#include <proj_api.h>
#include <std_msgs/Empty.h>

#include <asrl/messages/MapInfo.pb.h>
#include <asrl__planning/Overlay.h>
#include <babelfish_robochunk_translator/BabelfishMessage.h>
#include <ros/node_handle.h>
#include <asrl/common/rosutil/param.hpp>
#include <asrl/common/utils/CommonMacros.hpp>
#include <asrl/common/utils/thread_pool.hpp>
#include <steam/solver/DoglegGaussNewtonSolver.hpp>
#endif

namespace vtr {
namespace planning {

using asrl::pose_graph::RCEdge;
using asrl::pose_graph::RCGraph;
using asrl::pose_graph::RCGraphBase;
using asrl::pose_graph::RCRun;
using asrl::pose_graph::RCVertex;
using asrl::pose_graph::VertexId;

// template<class GRAPH>
class RosCallbacks
    : public virtual asrl::pose_graph::CallbackInterface<RCVertex, RCEdge,
                                                         RCRun> {
 public:
  using Base = asrl::pose_graph::CallbackInterface<RCVertex, RCEdge, RCRun>;

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

  using GraphMsg = asrl__pose_graph::GlobalGraph;
  using ComponentMsg = asrl__pose_graph::GraphComponent;
  using UpdateMsg = asrl__pose_graph::GraphUpdate;
  using VertexMsg = asrl__pose_graph::GlobalVertex;
  using EdgeMsg = asrl__pose_graph::IndexEdge;
  using GraphSrv = asrl__pose_graph::RelaxationService;
  using CalibSrv = asrl__pose_graph::CalibrationService;
#if 0
  using OverlaySrv = asrl__planning::Overlay;

  using ProjectionType = std::function<void(VertexMsg&)>;
  using TfMapType = std::unordered_map<VertexId, TransformType>;
  using MsgMapType = std::unordered_map<VertexId, VertexMsg>;

  using MutexPtr = __shared_ptr<std::mutex>;
#endif
  using PlannerPtr = asrl::planning::PlanningInterface::Ptr;
#if 0
  using PlannerWeakPtr = asrl::planning::PlanningInterface::WeakPtr;

  // using SolverType = steam::LevMarqGaussNewtonSolver;
  using SolverType = steam::DoglegGaussNewtonSolver;
#endif

#if 0
  static constexpr char gpsStream[] = "/DGPS/fix";
#endif

  PTR_TYPEDEFS(RosCallbacks)

  RosCallbacks(const GraphPtr& graph, const ros::NodeHandle& nh);

  /// @brief Callback for a new run
  virtual void runAdded(const RunPtr&);
  /// @brief Callback for a new vertex
  virtual void vertexAdded(const VertexPtr&);
  /// @brief Callback for a new edge
  virtual void edgeAdded(const EdgePtr& e);
  /// @brief Updates the cached graph relaxation
  virtual void updateRelaxation(const MutexPtr& mutex = nullptr);
  ///
  virtual void setPlanner(const PlannerPtr& planner) {
#if 0    
    planner_ = planner;
    overlayStatus_.publish(std_msgs::Empty());
#endif
  };

#if 0
  /// @brief Updates the cached map projection
  void updateProjection();


 private:
  /// @brief Callback for graph relaxation service
  bool relaxGraph(GraphSrv::Request& request, GraphSrv::Response& response);

  /// @brief Callback for calibration update service
  bool updateCalib(CalibSrv::Request& request, CalibSrv::Response&);

  /// @brief Callback for graph overlay service
  bool getOverlay(OverlaySrv::Request& request, OverlaySrv::Response& response);

  /// @brief Unique sequence ID generator
  inline uint32_t _nextSeq() {
    this->changeLock_.lock();
    uint32_t rval = this->seq_++;
    this->changeLock_.unlock();
    return rval;
  }

  /// @brief Helper to get a shred pointer to the graph
  inline GraphPtr _getGraph() {
    auto sharedGraph = graph_.lock();
    if (!sharedGraph) {
      throw std::runtime_error(
          "[getGraph] Attempted to use relaxation on an expired graph!");
    }
    return sharedGraph;
  }

  /// @brief Helper to get a shred pointer to the graph
  inline PlannerPtr _getPlanner() {
    auto sharedPlanner = planner_.lock();
    if (!sharedPlanner) {
      throw std::runtime_error(
          "[getPlanner] Attempted to get a non-existent planner!");
    }
    return sharedPlanner;
  }

  /// @brief Incrementally relax an edge into the graph, if possible
  bool _incrementalRelax(const EdgePtr& e);

  /// @brief Builds the map projection from SE(3) --> SE(2)
  void _buildProjection();

  /// @brief Initialize the message/transform maps for non-empty graphs
  void _initPoses();

  /// @brief Checks if GPS data has been added and updates the map
  void _checkGps();
#endif
  /// @brief Graph that generates the callbacks
  GraphWeakPtr graph_;
#if 0
  /// @brief Working copy of the manual subset of the graph
  GraphBasePtr workingGraph_;
#endif
  /// @brief ROS node handle for publishing
  ros::NodeHandle nh_;
#if 0
  /// @brief Publishes structural edge updates
  ros::Publisher edgeUpdates_;

  /// @brief Publishes updates to the relaxed graph
  ros::Publisher graphUpdates_;

  /// @brief Publishes a trigger when the overlay values change
  ros::Publisher overlayStatus_;

  /// @brief Service to request a relaxed version of the graph
  ros::ServiceServer relaxedGraphServer_;

  /// @brief Service to update the map alignment
  ros::ServiceServer mapCalibrationServer_;

  /// @brief Service to produce scalar map overlays
  ros::ServiceServer overlayServer_;

  /// @brief Cached response to avoid recomputation on every request
  GraphSrv::Response cachedResponse_;

  /// @brief Indicates whether or not the current relaxation is valid
  bool relaxationValid_;

  /// @brief Indicates whether or not the current projection is valid
  bool projectionValid_;

  /// @brief Flag to indicate the presence of GPS
  bool usingGps_;

  /// @brief Root vertex for relaxation
  VertexId root_;

  /// @brief Dynamically generated projection function
  ProjectionType project_;

  /// @brief Cached transform map to bootstrap incremental relaxation
  TfMapType tfMap_;

  /// @brief Cached vertex message map
  MsgMapType msgMap_;

  /// @brief Keep things thread safe
  std::recursive_mutex changeLock_;

  /// @brief Unique message sequence
  uint32_t seq_;

  /// @brief Concurrent thread pool for background relaxation
  asrl::common::thread_pool pool_;

  /// @brief Reference to the planner object used for path planning
  PlannerWeakPtr planner_;

  /// @brief Default map to use when we have no config
  asrl::graph_msgs::MapInfo defaultMap_;

  /// @brief VTR2->Translator communication publisher
  ros::Publisher babelfish_;
#endif
};

}  // namespace planning
}  // namespace vtr
