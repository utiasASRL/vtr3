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
#include <vtr_tactic/pipelines/base_pipeline.hpp>
#include <vtr_tactic/publisher_interface.hpp>
#include <vtr_tactic/task_queues/async_task_queue.hpp>
#include <vtr_tactic/types.hpp>

using OdometryMsg = nav_msgs::msg::Odometry;
using ROSPathMsg = nav_msgs::msg::Path;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

/// \todo define PathTracker::Ptr in Base
using PathTrackerPtr = std::shared_ptr<vtr::path_tracker::Base>;

namespace vtr {
namespace tactic {

class Tactic : public StateMachineInterface {
 public:
  using Ptr = std::shared_ptr<Tactic>;

  using ChainLockType = std::lock_guard<std::recursive_mutex>;

  struct Config {
    using Ptr = std::shared_ptr<Config>;
    /** \brief Configuration for the localization chain */
    LocalizationChain::Config chain_config;

    /** \brief Number of threads for the async task queue */
    int task_queue_num_threads = 1;

    /** \brief Maximum number of queued tasks in task queue */
    int task_queue_size = -1;

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
         BasePipeline::Ptr pipeline, Graph::Ptr graph);

  virtual ~Tactic();

  void setPublisher(PublisherInterface* pub) { publisher_ = pub; }
  void setPathTracker(PathTrackerPtr pt) { path_tracker_ = pt; }

  const LocalizationChain::Ptr& chain() { return chain_; }
  const Graph::Ptr& graph() { return graph_; }

  LockType lockPipeline();
  void setPipeline(const PipelineMode& pipeline_mode) override;
  void runPipeline(QueryCache::Ptr qdata);

 public:
  // clang-format off
  /** \brief Set the path being followed */
  void setPath(const VertexId::Vector& path, bool follow = false);
  /** \brief Set the current privileged vertex (topological localization) */
  void setTrunk(const VertexId& v) override;
  /** \brief Get distance between the current loc. chain to the target vertex */
  double distanceToSeqId(const uint64_t& seq_id) override;
  void addRun(bool ephemeral = false) override;
  /** \brief Returns whether the path following has completed */
  /// \todo currently this function only checks if the path tracker has stopped,
  /// however, it should also check if all waypoints have been reached, which is
  /// currently done in mission planner (follow.cpp)
  bool pathFollowingDone() { return path_tracker_ ? !path_tracker_->isRunning() : true; }
  bool canCloseLoop() const override;
  void connectToTrunk(bool privileged, bool merge) override;
  TacticStatus status() const { return status_; }
  LocalizationStatus tfStatus(const EdgeTransform&) const { return LocalizationStatus::Forced; }
  const Localization& persistentLoc() const { return persistent_loc_; }
  const Localization& targetLoc() const { return target_loc_; }
  const VertexId& closestVertexID() const override { return chain_->trunkVertexId(); }
  const VertexId& currentVertexID() const override { return current_vertex_id_; }
  void relaxGraph() { graph_->callback()->updateRelaxation(); }
  void saveGraph();
  // clang-format on

  VertexId closest_ = VertexId::Invalid();
  VertexId current_ = VertexId::Invalid();
  TacticStatus status_;
  Localization loc_;

 public:
  /// Internal data query functions for debugging
  const std::vector<EdgeTransform>& odometryPoses() const {
    return odometry_poses_;
  }

 private:
  void addDanglingVertex(const storage::Timestamp& stamp);
  void addConnectedVertex(const storage::Timestamp& stamp,
                          const EdgeTransform& T_r_m, const bool manual);
  void updatePersistentLoc(const VertexId& v, const EdgeTransform& T,
                           bool localized, bool reset_success = true);
  void updateTargetLoc(const VertexId& v, const EdgeTransform& T,
                       bool localized, bool reset_success = true);

 private:
  void startPathTracker();
  void stopPathTracker();

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
  Graph::Ptr graph_;
  PathTrackerPtr path_tracker_;
  PublisherInterface* publisher_ = nullptr;

  PipelineMode pipeline_mode_;
  BasePipeline::Ptr pipeline_;

  LocalizationChain::Ptr chain_;

  TaskExecutor::Ptr task_queue_;

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
  EdgeTransform T_w_m_odo_ = EdgeTransform(true);
  /** \brief Transformation from the localization keyframe to world frame */
  EdgeTransform T_w_m_loc_ = EdgeTransform(true);
  std::vector<PoseStampedMsg> keyframe_poses_;
  std::vector<EdgeTransform> odometry_poses_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Publisher<ROSPathMsg>::SharedPtr odo_path_pub_;
  rclcpp::Publisher<ROSPathMsg>::SharedPtr loc_path_pub_;
};

}  // namespace tactic
}  // namespace vtr