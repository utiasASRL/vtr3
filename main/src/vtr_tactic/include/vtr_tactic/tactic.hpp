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
 * \file tactic.hpp
 * \brief Tactic class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

/// for visualization in ROS
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <vtr_path_tracker/base.hpp>
#include <vtr_tactic/cache.hpp>
#include <vtr_tactic/memory_manager/live_memory_manager.hpp>
#include <vtr_tactic/memory_manager/map_memory_manager.hpp>
#include <vtr_tactic/pipelines/base_pipeline.hpp>
#include <vtr_tactic/publisher_interface.hpp>
#include <vtr_tactic/types.hpp>

using OdometryMsg = nav_msgs::msg::Odometry;
using ROSPathMsg = nav_msgs::msg::Path;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using TimeStampMsg = vtr_messages::msg::TimeStamp;

/// \todo define PathTracker::Ptr in Base
using PathTrackerPtr = std::shared_ptr<vtr::path_tracker::Base>;

namespace vtr {
namespace tactic {

using namespace vtr::mission_planning;

class Tactic : public mission_planning::StateMachineInterface {
 public:
  using Ptr = std::shared_ptr<Tactic>;

  using ChainLockType = std::lock_guard<std::recursive_mutex>;

  struct Config {
    using Ptr = std::shared_ptr<Config>;
    /** \brief Configuration for the localization chain */
    LocalizationChain::Config chain_config;
    /** \brief Configuration for the live memory manager */
    LiveMemoryManager::Config live_mem_config;
    /** \brief Configuration for the map memory manager */
    MapMemoryManager::Config map_mem_config;

    /** \brief Whether to extrapolate using STEAM trajectory for path tracker */
    bool extrapolate_odometry = false;

    /** \brief Whether to perform localization only on keyframe data */
    bool localization_only_keyframe = false;
    /**
     * \brief Whether localization is skippable when running in a separate
     * thread
     */
    bool localization_skippable = true;
    /** \brief Default localization covariance when chain is not localized. */
    Eigen::Matrix<double, 6, 6> default_loc_cov =
        Eigen::Matrix<double, 6, 6>::Zero();

    /** \brief Threshold for merging <x, y, theta> */
    std::vector<double> merge_threshold = {0.5, 0.25, 0.2};

    /**
     * \brief Whether to call the pipeline visualization functions and publish
     * odometry and localization to ROS.
     */
    bool visualize = false;
    Eigen::Matrix<double, 3, 1> vis_loc_path_offset =
        Eigen::Matrix<double, 3, 1>::Zero();

    static const Ptr fromROS(const rclcpp::Node::SharedPtr node);
  };

  Tactic(Config::Ptr config, const rclcpp::Node::SharedPtr node,
         BasePipeline::Ptr pipeline, Graph::Ptr graph)
      : node_(node),
        config_(config),
        graph_(graph),
        pipeline_(pipeline),
        chain_(
            std::make_shared<LocalizationChain>(config->chain_config, graph)),
        live_mem_(config->live_mem_config, this, graph),
        map_mem_(config->map_mem_config, chain_, graph) {
    /// pipeline specific initialization
    pipeline_->initialize(graph_);

    /// start the memory managers
    live_mem_.start();
    map_mem_.start();

    /// for visualization only
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    odo_path_pub_ = node_->create_publisher<ROSPathMsg>("odo_path", 10);
    loc_path_pub_ = node_->create_publisher<ROSPathMsg>("loc_path", 10);
    // world offset for localization path visualization
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    Eigen::Affine3d T(Eigen::Translation3d(config_->vis_loc_path_offset));
    auto msg = tf2::eigenToTransform(T);
    msg.header.frame_id = "world";
    msg.child_frame_id = "world (offset)";
    tf_static_broadcaster_->sendTransform(msg);
  }

  virtual ~Tactic() {
    // wait for the pipeline to clear
    auto lck = lockPipeline();
    // stop the path tracker
    if (path_tracker_) path_tracker_->stopAndJoin();
  }

  void setPublisher(PublisherInterface* pub) { publisher_ = pub; }
  /** \brief Set the path tracker. */
  void setPathTracker(PathTrackerPtr pt) { path_tracker_ = pt; }

  LockType lockPipeline() {
    /// Lock so that no more data are passed into the pipeline
    LockType pipeline_lck(pipeline_mutex_);

    /// Pause the trackers/controllers but keep the current goal
    path_tracker::State pt_state;
    if (path_tracker_ && path_tracker_->isRunning()) {
      pt_state = path_tracker_->getState();
      if (pt_state != path_tracker::State::STOP) {
        CLOG(INFO, "tactic") << "Pausing path tracker thread";
      }
      path_tracker_->pause();
    }

    /// Waiting for unfinished pipeline job
    if (pipeline_thread_future_.valid()) pipeline_thread_future_.get();

    /// Lock so that no more data are passed into localization (during follow)
    std::lock_guard<std::mutex> loc_lck(localization_mutex_);
    /// Waiting for unfinished localization job
    if (localization_thread_future_.valid()) localization_thread_future_.get();

    /// Waiting for any unfinished jobs in pipeline
    pipeline_->wait();

    /// resume the trackers/controllers to their original state
    if (path_tracker_ && path_tracker_->isRunning()) {
      path_tracker_->setState(pt_state);
      CLOG(INFO, "tactic") << "Resuming path tracker thread to state "
                           << int(pt_state);
    }

    return pipeline_lck;
  }

  void setPipeline(const PipelineMode& pipeline_mode) override {
    CLOG(DEBUG, "tactic") << "[Lock Requested] setPipeline";
    auto lck = lockPipeline();
    CLOG(DEBUG, "tactic") << "[Lock Acquired] setPipeline";

    pipeline_mode_ = pipeline_mode;

    // Perform nessary resets upon entering a new pipeline mode
    switch (pipeline_mode_) {
      case PipelineMode::Idle:
        break;
      case PipelineMode::Branching:
        break;
      case PipelineMode::Merging:
        target_loc_.successes = 0;  // reset target localization success
        break;
      case PipelineMode::Searching:
        persistent_loc_.successes = 0;  // reset persistent localization success
        break;
      case PipelineMode::Following:
        break;
    }

    CLOG(DEBUG, "tactic") << "[Lock Released] setPipeline";
  };
  void runPipeline(QueryCache::Ptr qdata);

  void addRun(bool ephemeral = false, bool extend = false,
              bool save = true) override {
    /// \todo the following parameter are still needed?
    (void)ephemeral;
    (void)extend;
    (void)save;

    CLOG(DEBUG, "tactic") << "[Lock Requested] addRun";
    auto lck = lockPipeline();
    CLOG(DEBUG, "tactic") << "[Lock Acquired] addRun";

    /// \todo (yuchen) robot id is 0 for now
    graph_->addRun(0);

    // re-initialize the run
    first_frame_ = true;
    current_vertex_id_ = VertexId((uint64_t)-1);

    // re-initialize the localization chain (path will be added later)
    {
      CLOG(DEBUG, "tactic") << "[ChainLock Requested] addRun";
      const auto lock = chain_->guard();
      CLOG(DEBUG, "tactic") << "[ChainLock Acquired] addRun";
      chain_->reset();
      CLOG(DEBUG, "tactic") << "[ChainLock Released] addRun";
    }

    // re-initialize the pose records for visualization
    T_w_m_odo_ = lgmath::se3::TransformationWithCovariance(true);
    T_w_m_loc_ = lgmath::se3::TransformationWithCovariance(true);
    keyframe_poses_.clear();
    odometry_poses_.clear();
    keyframe_poses_.reserve(1000);
    odometry_poses_.reserve(5000);

    CLOG(DEBUG, "tactic") << "[Lock Released] addRun";
  }

 public:
  /// \todo a bunch of functions to be implemented for state machine
  const Localization& persistentLoc() const { return persistent_loc_; }
  const Localization& targetLoc() const { return target_loc_; }
  void setTrunk(const VertexId& v) override {
    CLOG(DEBUG, "tactic") << "[Lock Requested] setTrunk";
    auto lck = lockPipeline();
    CLOG(DEBUG, "tactic") << "[Lock Acquired] setTrunk";

    persistent_loc_ = Localization(v);
    target_loc_ = Localization();

    if (publisher_) publisher_->publishRobot(persistent_loc_);

    CLOG(DEBUG, "tactic") << "[Lock Released] setTrunk";
  }

  /**
   * \brief Returns whether the path following has completed
   * \todo currently this function only checks if the path tracker has stopped,
   * however, it should also check if all waypoints have been reached, which is
   * currently done in mission planner (follow.cpp)
   */
  bool pathFollowingDone() {
    if (path_tracker_) return !path_tracker_->isRunning();
    return true;
  }

  double distanceToSeqId(const uint64_t& seq_id) {
    LockType lck(pipeline_mutex_);
    double distance_to_seq_id = 0;
    {
      CLOG(DEBUG, "tactic") << "[ChainLock Requested] distanceToSeqId";
      const auto lock = chain_->guard();
      CLOG(DEBUG, "tactic") << "[ChainLock Acquired] distanceToSeqId";

      /// Clip the sequence ID to the max/min for the chain
      auto clip_seq = unsigned(std::min(seq_id, chain_->sequence().size() - 1));
      clip_seq = std::max(clip_seq, 0u);
      auto trunk_seq = unsigned(chain_->trunkSequenceId());

      /// if sequence is the same then we have arrived at this vertex.
      if (clip_seq == trunk_seq) {
        CLOG(DEBUG, "tactic") << "[ChainLock Released] distanceToSeqId";
        return 0;
      }

      ///
      unsigned start_seq = std::min(clip_seq, trunk_seq);
      unsigned end_seq = std::max(clip_seq, trunk_seq);
      // Compound raw distance along the path
      double dist = 0.;
      for (unsigned idx = start_seq; idx < end_seq; ++idx) {
        dist += (chain_->pose(idx) * chain_->pose(idx + 1).inverse())
                    .r_ab_inb()
                    .norm();
      }
      // Returns a negative value if we have passed that sequence already
      distance_to_seq_id =
          (clip_seq < chain_->trunkSequenceId()) ? -dist : dist;
      CLOG(DEBUG, "tactic") << "[ChainLock Released] distanceToSeqId";
    }
    return distance_to_seq_id;
  }
  TacticStatus status() const { return status_; }
  LocalizationStatus tfStatus(const pose_graph::RCEdge::TransformType&) const {
    return LocalizationStatus::Forced;
  }

  const VertexId& closestVertexID() const override {
    return chain_->trunkVertexId();
  }

  const VertexId& currentVertexID() const override {
    return current_vertex_id_;
  }

  bool canCloseLoop() const override {
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
      if (dx > config_->merge_threshold[0] ||
          dy > config_->merge_threshold[1] ||
          dt > config_->merge_threshold[2]) {
        reason += "offset from path is too large to merge; ";
        CLOG(WARNING, "tactic")
            << "Offset from path is too large to merge (x, y, th): " << dx
            << ", " << dy << " " << dt;
      }
    }
    if (!reason.empty()) {
      CLOG(WARNING, "tactic") << "Cannot merge because " << reason;
      return false;
    }
    return true;
  }

  void connectToTrunk(bool privileged, bool merge) override {
    CLOG(DEBUG, "tactic") << "[Lock Requested] connectToTrunk";
    auto lck = lockPipeline();
    CLOG(DEBUG, "tactic") << "[Lock Acquired] connectToTrunk";

    CLOG(DEBUG, "tactic") << "[ChainLock Requested] connectToTrunk";
    const auto lock = chain_->guard();
    CLOG(DEBUG, "tactic") << "[ChainLock Acquired] connectToTrunk";

    if (merge) {  /// For merging, i.e. loop closure
      CLOG(INFO, "tactic") << "Adding closure " << current_vertex_id_ << " --> "
                           << chain_->trunkVertexId();
      CLOG(DEBUG, "tactic") << "with transform:\n"
                            << chain_->T_petiole_trunk().inverse();
      graph_->addEdge(current_vertex_id_, chain_->trunkVertexId(),
                      chain_->T_petiole_trunk().inverse(), pose_graph::Spatial,
                      privileged);
    } else {  /// Upon successful metric localization.
      auto neighbours = graph_->at(current_vertex_id_)->spatialNeighbours();
      if (neighbours.size() == 0) {
        CLOG(INFO, "tactic")
            << "Adding connection " << current_vertex_id_ << " --> "
            << chain_->trunkVertexId() << ", privileged: " << privileged
            << ", with transform:"
            << chain_->T_petiole_trunk().inverse().vec().transpose();
        graph_->addEdge(current_vertex_id_, chain_->trunkVertexId(),
                        chain_->T_petiole_trunk().inverse(),
                        pose_graph::Spatial, privileged);
      } else if (neighbours.size() == 1) {
        CLOG(INFO, "tactic")
            << "Connection exists: " << current_vertex_id_ << " --> "
            << *neighbours.begin() << ", privileged set to: " << privileged;
        if (*neighbours.begin() != chain_->trunkVertexId()) {
          std::string err{"Not adding an edge connecting to trunk."};
          CLOG(ERROR, "tactic") << err;
          throw std::runtime_error{err};
        }
        graph_->at(current_vertex_id_, *neighbours.begin())
            ->setManual(privileged);
      } else {
        std::string err{"Should never reach here."};
        CLOG(ERROR, "tactic") << err;
        throw std::runtime_error{err};
      }
    }

    CLOG(DEBUG, "tactic") << "[ChainLock Released] connectToTrunk";

    CLOG(DEBUG, "tactic") << "[Lock Released] connectToTrunk";
  }

  void relaxGraph() { graph_->callbacks()->updateRelaxation(); }
  void saveGraph() {
    CLOG(DEBUG, "tactic") << "[Lock Requested] saveGraph";
    auto lck = lockPipeline();
    CLOG(DEBUG, "tactic") << "[Lock Acquired] saveGraph";
    graph_->save();
    CLOG(DEBUG, "tactic") << "[Lock Released] saveGraph";
  }

  void setPath(const VertexId::Vector& path, bool follow = false) {
    CLOG(DEBUG, "tactic") << "[Lock Requested] setPath";
    auto lck = lockPipeline();
    CLOG(DEBUG, "tactic") << "[Lock Acquired] setPath";

    /// Clear any existing path in UI
    if (publisher_) publisher_->clearPath();

    /// Set path and target localization
    /// \todo is it possible to put target loc into chain?
    CLOG(INFO, "tactic") << "Set path of size " << path.size();

    {
      CLOG(DEBUG, "tactic") << "[ChainLock Requested] setPath";
      const auto lock = chain_->guard();
      CLOG(DEBUG, "tactic") << "[ChainLock Acquired] setPath";

      chain_->setSequence(path);
      target_loc_ = Localization();

      if (path.size() > 0) {
        chain_->expand();
        publishPath(node_->now());
        if (follow && publisher_) {
          publisher_->publishPath(*chain_);
          startPathTracker();
        }
      } else {
        // make sure path tracker is stopped
        stopPathTracker();
      }
      CLOG(DEBUG, "tactic") << "[ChainLock Released] setPath";
    }

    CLOG(DEBUG, "tactic") << "[Lock Released] setPath";
  }

  VertexId closest_ = VertexId::Invalid();
  VertexId current_ = VertexId::Invalid();
  TacticStatus status_;
  Localization loc_;

 public:
  /// Internal data query functions for debugging
  const std::vector<lgmath::se3::TransformationWithCovariance>& odometryPoses()
      const {
    return odometry_poses_;
  }

 private:
  void addConnectedVertex(
      const TimeStampMsg& stamp,
      const lgmath::se3::TransformationWithCovariance& T_r_m,
      const bool manual) {
    /// Add the new vertex
    auto previous_vertex_id = current_vertex_id_;
    addDanglingVertex(stamp);

    /// Add connection
    (void)graph_->addEdge(previous_vertex_id, current_vertex_id_, T_r_m,
                          pose_graph::Temporal, manual);
  }
  void addDanglingVertex(const TimeStampMsg& stamp) {
    /// Add the new vertex
    auto vertex = graph_->addVertex(stamp);
    current_vertex_id_ = vertex->id();
  }

  void updatePersistentLoc(const VertexId& v, const EdgeTransform& T,
                           bool localized, bool reset_success = true) {
    // reset localization successes when switching to a new vertex
    persistent_loc_.v = v;
    persistent_loc_.T = T;
    persistent_loc_.localized = localized;
    if (reset_success) persistent_loc_.successes = 0;

    if (localized && !T.covarianceSet()) {
      CLOG(WARNING, "tactic")
          << "Attempted to set target loc without a covariance!";
    }
  }

  void updateTargetLoc(const VertexId& v, const EdgeTransform& T,
                       bool localized, bool reset_success = true) {
    // reset localization successes when switching to a new vertex
    target_loc_.v = v;
    target_loc_.T = T;
    target_loc_.localized = localized;
    if (reset_success) target_loc_.successes = 0;

    if (localized && !T.covarianceSet()) {
      CLOG(WARNING, "tactic")
          << "Attempted to set target loc without a covariance!";
    }
  }

 private:
  void startPathTracker() {
    if (!path_tracker_) {
      CLOG(WARNING, "tactic")
          << "Path tracker not set! Cannot start control loop.";
      return;
    }

    CLOG(INFO, "tactic") << "Starting path tracker.";
    path_tracker_->followPathAsync(path_tracker::State::RUN, chain_);
  }

  void stopPathTracker() {
    if (!path_tracker_) {
      CLOG(WARNING, "tactic")
          << "Path tracker not set! Cannot stop control loop.";
      return;
    }

    CLOG(INFO, "tactic") << "Stopping path tracker.";
    path_tracker_->stopAndJoin();
  }

 private:
  /** \brief Start running the pipeline (probably in a separate thread) */
  void runPipeline_(QueryCache::Ptr qdata);

  /** \brief Runs localization in follow (probably in a separate thread) */
  void runLocalizationInFollow_(QueryCache::Ptr qdata);
  /** \brief Runs localization in search (probably in a separate thread) */
  void runLocalizationInSearch_(QueryCache::Ptr qdata);
  /** \brief Runs localization in merge (probably in a separate thread) */
  void runLocalizationInMerge_(QueryCache::Ptr qdata);

  void updatePathTracker(QueryCache::Ptr qdata);

  void branch(QueryCache::Ptr qdata);
  void merge(QueryCache::Ptr qdata);
  void search(QueryCache::Ptr qdata);
  void follow(QueryCache::Ptr qdata);

  /** \brief Publishes odometry estimate in a global frame for visualization. */
  void publishOdometry(QueryCache::Ptr qdata);
  /** \brief Publishes the repeat path in a global frame for visualization. */
  void publishPath(rclcpp::Time rcl_stamp);
  /** \brief Publishes current frame localized against for visualization. */
  void publishLocalization(QueryCache::Ptr qdata);

 private:
  const rclcpp::Node::SharedPtr node_;

  Config::Ptr config_;
  PublisherInterface* publisher_;
  Graph::Ptr graph_;
  PathTrackerPtr path_tracker_;

  PipelineMode pipeline_mode_;
  BasePipeline::Ptr pipeline_;

  LocalizationChain::Ptr chain_;

  LiveMemoryManager live_mem_;
  MapMemoryManager map_mem_;

  std::recursive_timed_mutex pipeline_mutex_;
  std::future<void> pipeline_thread_future_;

  std::mutex localization_mutex_;
  std::future<void> localization_thread_future_;

  bool first_frame_ = true;

  /** \brief Vertex id of the latest keyframe, initialized to invalid */
  VertexId current_vertex_id_ = VertexId((uint64_t)-1);

  /** \brief Localization against the map, that persists across runs. */
  Localization persistent_loc_;
  /** \brief Localization against a target for merging. */
  Localization target_loc_;

  /** \brief Transformation from the latest keyframe to world frame */
  lgmath::se3::TransformationWithCovariance T_w_m_odo_ =
      lgmath::se3::TransformationWithCovariance(true);
  /** \brief Transformation from the localization keyframe to world frame */
  lgmath::se3::TransformationWithCovariance T_w_m_loc_ =
      lgmath::se3::TransformationWithCovariance(true);
  std::vector<PoseStampedMsg> keyframe_poses_;
  std::vector<lgmath::se3::TransformationWithCovariance> odometry_poses_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Publisher<ROSPathMsg>::SharedPtr odo_path_pub_;
  rclcpp::Publisher<ROSPathMsg>::SharedPtr loc_path_pub_;
};

}  // namespace tactic
}  // namespace vtr