#pragma once

#include "rclcpp/rclcpp.hpp"

// temp: for broadcast odometry transforms
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_tactic/caches.hpp>
#include <vtr_tactic/localization_chain/localization_chain.hpp>
#include <vtr_tactic/pipelines/base_pipeline.hpp>
#include <vtr_tactic/publisher_interface.hpp>
#include <vtr_tactic/types.hpp>

using OdometryMsg = nav_msgs::msg::Odometry;
using ROSPathMsg = nav_msgs::msg::Path;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using TimeStampMsg = vtr_messages::msg::TimeStamp;

namespace vtr {
namespace tactic {

using namespace vtr::mission_planning;

class Tactic : public mission_planning::StateMachineInterface {
 public:
  using Ptr = std::shared_ptr<Tactic>;

  // Convenient typedef
  using Path = VertexId::Vector;

  struct Config {
    using Ptr = std::shared_ptr<Config>;
    /** \brief Configuration for the localization chain */
    LocalizationChain::Config chain_config;

    Eigen::Matrix<double, 6, 6> default_loc_cov;

    static const Ptr fromROS(const rclcpp::Node::SharedPtr node);
  };

  Tactic(Config::Ptr config, BasePipeline::Ptr pipeline, Graph::Ptr graph)
      : config_(config),
        pipeline_(pipeline),
        graph_(graph),
        chain_(config->chain_config, graph) {}

  virtual ~Tactic() {}

  void setPublisher(const PublisherInterface* pub) { publisher_ = pub; }

  void initializePipeline();
  LockType lockPipeline() {
    /// Lock to make sure all frames clear the pipeline
    LockType lck(pipeline_mutex_);
    return lck;
  }
  void setPipeline(const PipelineMode& pipeline_mode) override {
    LOG(DEBUG) << "[Lock Requested] setPipeline";
    auto lck = lockPipeline();
    LOG(DEBUG) << "[Lock Acquired] setPipeline";

    pipeline_mode_ = pipeline_mode;

    LOG(DEBUG) << "[Lock Released] setPipeline";
  };
  void runPipeline(QueryCache::Ptr qdata);
  const MapCache::Ptr getMapCache() { return mdata_; };

  void addRun(bool ephemeral = false, bool extend = false,
              bool save = true) override {
    LOG(DEBUG) << "[Lock Requested] addRun";
    auto lck = lockPipeline();
    LOG(DEBUG) << "[Lock Acquired] addRun";

    /// \todo (yuchen) robot id is 0 for now
    graph_->addRun(0);

    // re-initialize the run
    first_frame_ = true;

    LOG(DEBUG) << "[Lock Released] addRun";
  }

  void publishPath(rclcpp::Time rcl_stamp) {
    std::vector<Eigen::Affine3d> eigen_poses;
    /// publish the repeat path in
    chain_.expand();
    for (unsigned i = 0; i < chain_.sequence().size(); i++) {
      eigen_poses.push_back(Eigen::Affine3d(chain_.pose(i).matrix()));
    }

    /// Publish the repeat path
    ROSPathMsg path;
    path.header.frame_id = "world";
    path.header.stamp = rcl_stamp;
    auto& poses = path.poses;
    for (const auto& pose : eigen_poses) {
      PoseStampedMsg ps;
      ps.pose = tf2::toMsg(pose);
      poses.push_back(ps);
    }
    loc_path_pub_->publish(path);
  }

 public:
  /// \todo a bunch of functions to be implemented for state machine
  const Localization& persistentLoc() const { return persistent_loc_; }
  const Localization& targetLoc() const { return target_loc_; }
  void setTrunk(const VertexId& v) override {
    LOG(DEBUG) << "[Lock Requested] setTrunk";
    auto lck = lockPipeline();
    LOG(DEBUG) << "[Lock Acquired] setTrunk";

    persistent_loc_ = Localization(v);
    target_loc_ = Localization();

    if (publisher_) publisher_->publishRobot(persistent_loc_);

    LOG(DEBUG) << "[Lock Released] setTrunk";
  }
  double distanceToSeqId(const uint64_t&) { return 9001; }
  TacticStatus status() const { return status_; }
  LocalizationStatus tfStatus(const pose_graph::RCEdge::TransformType&) const {
    return LocalizationStatus::Forced;
  }
#if false
  const VertexId& closestVertexID() const override {
    return chain_.trunkVertexId();
  }
#endif
  const VertexId& currentVertexID() const override {
    return current_vertex_id_;
  }

  const bool canCloseLoop() const override {
    std::string reason = "";
    /// Check persistent localization
    if (!persistent_loc_.localized)
      reason += "cannot localize against the live vertex; ";
    /// \todo check covariance

    /// Check target localization
    if (!target_loc_.localized)
      reason += "cannot localize against the target vertex for merging; ";
    /// \todo check covariance
    else {
      // Offset in the x, y, and yaw directions
      auto& T = target_loc_.T;
      double dx = T.r_ba_ina()(0), dy = T.r_ba_ina()(1), dt = T.vec()(5);
      if (dx > 0.5 || dy > 0.25 || dt > 0.2) {
        reason += "offset from path is too large to merge; ";
        LOG(WARNING) << "Offset from path is too large to merge (x, y, th): "
                     << dx << ", " << dy << " " << dt;
      }
    }
    if (!reason.empty()) {
      LOG(WARNING) << "Cannot merge because " << reason;
      return false;
    }
    return true;
  }

  void connectToTrunk(bool privileged) override {
    LOG(DEBUG) << "[Lock Requested] connectToTrunk";
    auto lck = lockPipeline();
    LOG(DEBUG) << "[Lock Acquired] connectToTrunk";

    /// \todo consider making a keyframe when leaf to petiole is large
    auto neighbours = graph_->at(current_vertex_id_)->spatialNeighbours();
    if (neighbours.size() == 1) {
      graph_->at(current_vertex_id_, *neighbours.begin())
          ->setManual(privileged);
    } else if (neighbours.empty()) {
      LOG(DEBUG) << "Adding closure " << current_vertex_id_ << " --> "
                 << chain_.trunkVertexId()
                 << " with transform: " << chain_.T_petiole_trunk().inverse();
      graph_->addEdge(current_vertex_id_, chain_.trunkVertexId(),
                      chain_.T_petiole_trunk().inverse(), pose_graph::Spatial,
                      privileged);
    }

    LOG(DEBUG) << "[Lock Released] connectToTrunk";
  }

  void relaxGraph() { graph_->callbacks()->updateRelaxation(steam_mutex_ptr_); }
  void saveGraph() {
    LOG(DEBUG) << "[Lock Requested] saveGraph";
    auto lck = lockPipeline();
    LOG(DEBUG) << "[Lock Acquired] saveGraph";
    graph_->save();
    LOG(DEBUG) << "[Lock Released] saveGraph";
  }

  void setPath(const Path& path, bool follow = false) {
    LOG(DEBUG) << "[Lock Requested] setPath";
    auto lck = lockPipeline();
    LOG(DEBUG) << "[Lock Acquired] setPath";

    /// Clear any existing path in UI
    if (publisher_) publisher_->clearPath();

    /// Set path and target localization
    /// \todo is it possible to put target loc into chain?
    LOG(INFO) << "Set path of size " << path.size();
    chain_.setSequence(path);
    target_loc_ = Localization();

    if (path.size() > 0) {
      chain_.expand();
      if (follow && publisher_) publisher_->publishPath(chain_);
    }

    LOG(DEBUG) << "[Lock Released] setPath";
  }

  VertexId closest_ = VertexId::Invalid();
  VertexId current_ = VertexId::Invalid();
  TacticStatus status_;
  Localization loc_;

 private:
  void addConnectedVertex(
      const TimeStampMsg& stamp,
      const lgmath::se3::TransformationWithCovariance& T_r_m) {
    /// Add the new vertex
    auto previous_vertex_id = current_vertex_id_;
    addDanglingVertex(stamp);

    /// Add connection
    bool manual = (pipeline_mode_ == PipelineMode::Branching) ||
                  (pipeline_mode_ == PipelineMode::Merging);
    (void)graph_->addEdge(previous_vertex_id, current_vertex_id_, T_r_m,
                          pose_graph::Temporal, manual);
  }
  void addDanglingVertex(const TimeStampMsg& stamp) {
    /// Add the new vertex
    auto vertex = graph_->addVertex(stamp);
    current_vertex_id_ = vertex->id();
  }

  void updatePersistentLoc(const VertexId& v, const EdgeTransform& T,
                           bool localized) {
    // reset localization successes when switching to a new vertex
    if (persistent_loc_.v != v) persistent_loc_.successes = 0;
    persistent_loc_.v = v;
    persistent_loc_.T = T;
    persistent_loc_.localized = localized;

    if (!T.covarianceSet()) {
      LOG(WARNING) << "Attempted to set target loc without a covariance!";
    }
  }

  void updateTargetLoc(const VertexId& v, const EdgeTransform& T,
                       bool localized) {
    // reset localization successes when switching to a new vertex
    if (target_loc_.v != v) target_loc_.successes = 0;
    target_loc_.v = v;
    target_loc_.T = T;
    target_loc_.localized = localized;

    if (!T.covarianceSet()) {
      LOG(WARNING) << "Attempted to set target loc without a covariance!";
    }
  }

 private:
  void branch(QueryCache::Ptr qdata);
  void merge(QueryCache::Ptr qdata);
  void follow(QueryCache::Ptr qdata);

  /// temporary functions
  void publishOdometry(QueryCache::Ptr qdata);
  void publishLocalization(QueryCache::Ptr qdata);

 private:
  Config::Ptr config_;
  PipelineMode pipeline_mode_;
  std::recursive_timed_mutex pipeline_mutex_;
  BasePipeline::Ptr pipeline_;
  Graph::Ptr graph_;
  LocalizationChain chain_;
  const PublisherInterface* publisher_;

  MapCache::Ptr mdata_ = std::make_shared<MapCache>();
  bool first_frame_ = true;

  /** \brief STEAM is not thread safe, so need a mutex.*/
  std::shared_ptr<std::mutex> steam_mutex_ptr_ = std::make_shared<std::mutex>();

  /** \brief Vertex id of the latest keyframe, initialized to invalid */
  VertexId current_vertex_id_ = VertexId((uint64_t)-1);

  /** \brief Localization against the map, that persists across runs. */
  Localization persistent_loc_;
  /** \brief Localization against a target for merging. */
  Localization target_loc_;

  // temporary
  // estimated pose of the last keyframe in world frame
  std::vector<lgmath::se3::TransformationWithCovariance> T_m_w_odo_ = {
      Eigen::Matrix4d(Eigen::Matrix4d::Identity(4, 4))};
  // estimated pose of the map keyframe in world frame (localization)
  std::vector<lgmath::se3::TransformationWithCovariance> T_m_w_loc_ = {
      Eigen::Matrix4d(Eigen::Matrix4d::Identity(4, 4))};

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<ROSPathMsg>::SharedPtr odo_path_pub_;
  rclcpp::Publisher<ROSPathMsg>::SharedPtr loc_path_pub_;
};

}  // namespace tactic
}  // namespace vtr