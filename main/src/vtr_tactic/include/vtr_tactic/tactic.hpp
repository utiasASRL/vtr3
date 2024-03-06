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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/pipeline_interface.hpp"
#include "vtr_tactic/pipelines/base_pipeline.hpp"
#include "vtr_tactic/tactic_callback_interface.hpp"
#include "vtr_tactic/tactic_interface.hpp"
#include "vtr_tactic/task_queue.hpp"
#include "vtr_tactic/types.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace vtr {
namespace tactic {

class Tactic : public PipelineInterface, public TacticInterface {
 public:
  PTR_TYPEDEFS(Tactic);
  using Callback = TacticCallbackInterface;
  using TaskQueueCallback = PipelineInterface::TaskQueueCallback;

  using RobotStateMutex = std::mutex;
  using RobotStateLock = std::unique_lock<RobotStateMutex>;
  using RobotStateGuard = std::lock_guard<RobotStateMutex>;

  struct Config {
    PTR_TYPEDEFS(Config);

    bool enable_parallelization = false;
    bool preprocessing_skippable = false;
    bool odometry_mapping_skippable = false;
    bool localization_skippable = false;

    /** \brief Number of threads for the async task queue */
    int task_queue_num_threads = 1;
    /** \brief Maximum number of queued tasks in task queue */
    int task_queue_size = -1;

    /** \brief */
    double route_completion_translation_threshold = 0.5;

    /** \brief Configuration for the localization chain */
    LocalizationChain::Config chain_config;

    bool save_odometry_result = false;
    bool save_odometry_vel_result = false;
    bool save_localization_result = false;
    /** \brief Visualize odometry and localization via Rviz. */
    bool visualize = false;

    static UniquePtr fromROS(const rclcpp::Node::SharedPtr& node,
                             const std::string& prefix = "tactic");
  };

  Tactic(Config::UniquePtr config, const BasePipeline::Ptr& pipeline,
         const OutputCache::Ptr& output, const Graph::Ptr& graph,
         const Callback::Ptr& callback = std::make_shared<Callback>(),
         const TaskQueueCallback::Ptr& task_queue_callback =
             std::make_shared<TaskQueueCallback>());

  ~Tactic() { join(); }

  /// tactic interface, interacts with the state machine
 public:
  TacticInterface::PipelineLock lockPipeline() override;
  /// \note following functions must be called with pipeline locked
  void setPipeline(const PipelineMode& pipeline_mode) override;
  void addRun(const bool ephemeral = false) override;
  void finishRun() override;
  void setPath(const VertexId::Vector& path, const unsigned& trunk_sid = 0,
               const EdgeTransform& T_twig_branch = EdgeTransform(true),
               const bool publish = false) override;
  void setTrunk(const VertexId& v = VertexId::Invalid()) override;
  void connectToTrunk(const bool privileged = false) override;
  /// \note following queries can be called without pipeline locked
  Localization getPersistentLoc() const override;
  bool isLocalized() const override;
  bool passedSeqId(const uint64_t& sid) const override;
  bool routeCompleted() const override;

 private:
  /** \brief Performs the actual preprocessing task */
  bool input_(const QueryCache::Ptr& qdata) override;

  /** \brief Performs the actual preprocessing task */
  bool preprocess_(const QueryCache::Ptr& qdata) override;

  /** \brief Performs the actual odometry mapping task */
  bool runOdometryMapping_(const QueryCache::Ptr& qdata) override;
  bool teachMetricLocOdometryMapping(const QueryCache::Ptr& qdata);
  bool teachBranchOdometryMapping(const QueryCache::Ptr& qdata);
  bool teachMergeOdometryMapping(const QueryCache::Ptr& qdata);
  bool repeatMetricLocOdometryMapping(const QueryCache::Ptr& qdata);
  bool repeatFollowOdometryMapping(const QueryCache::Ptr& qdata);

  /** \brief Performs the actual localization task */
  bool runLocalization_(const QueryCache::Ptr& qdata) override;
  bool teachMetricLocLocalization(const QueryCache::Ptr& qdata);
  bool teachBranchLocalization(const QueryCache::Ptr& qdata);
  bool teachMergeLocalization(const QueryCache::Ptr& qdata);
  bool repeatMetricLocLocalization(const QueryCache::Ptr& qdata);
  bool repeatFollowLocalization(const QueryCache::Ptr& qdata);

 private:
  /// pipeline helper functions and states
  void addVertexEdge(const Timestamp& stamp, const EdgeTransform& T_r_v,
                     const bool manual, const EnvInfo& env_info);

  /**
   * \brief Whether this is the first frame of this run, only used by
   * preprocessing thread.
   * \note Only change this variable when pipeline is locked.
   */
  bool first_frame_ = true;

  /**
   * \brief Current vertex id for odometry, only used by odometry thread
   * \note Only change this when pipeline is locked
   */
  VertexId current_vertex_id_ = VertexId::Invalid();

  /**
   * \brief used to determine what pipeline to use
   * \note Only change this when pipeline is locked
   */
  PipelineMode pipeline_mode_ = PipelineMode::Idle;

 private:
  Config::UniquePtr config_;
  const BasePipeline::Ptr pipeline_;
  const OutputCache::Ptr output_;
  const LocalizationChain::Ptr chain_;
  const Graph::Ptr graph_;

  /// robot status update related
 private:
  /**
   * \brief Called in odometry thread, also updated by state machine when user
   * sets trunk vertex
   */
  void updatePersistentLoc(const Timestamp& t, const VertexId& v,
                           const EdgeTransform& T_r_v, const bool localized);
  /**
   * \brief Called in odometry thread, also updated by state machine when user
   * sets trunk vertex
   */
  void updateTargetLoc(const Timestamp& t, const VertexId& v,
                       const EdgeTransform& T_r_v, const bool localized);

  /** \brief Protects: persistent_loc_, target_loc_ */
  mutable RobotStateMutex robot_state_mutex_;
  /** \brief Localization against the map, that persists across runs. */
  Localization persistent_loc_;
  /** \brief Localization against a target for merging. */
  Localization target_loc_;
  /** \brief callback on robot state update */
  const Callback::Ptr callback_;

  /// \note updates to these variables are protected by the tactic mutex.
  /** \brief Transformation from the odometry vertex frame to world frame */
  EdgeTransform T_w_v_odo_ = EdgeTransform(true);
  /** \brief Transformation from the localization vertex frame to world frame */
  EdgeTransform T_w_v_loc_ = EdgeTransform(true);

  friend class TacticCallbackInterface;
};

}  // namespace tactic
}  // namespace vtr