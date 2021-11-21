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

#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/pipelines/base_pipeline.hpp"
#include "vtr_tactic/task_queues/async_task_queue.hpp"
#include "vtr_tactic/types.hpp"

using OdometryMsg = nav_msgs::msg::Odometry;
using ROSPathMsg = nav_msgs::msg::Path;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

namespace vtr {
namespace tactic {

/**
 * \brief Bounded buffer for the producer and consumer problem across
 * preprocessing, odometry&mapping and localization threads.
 * \details Data can be added as discardable and non-discardable. When the size
 * of the buffer is full, the oldest discardable data is removed when more data
 * are added. When no discardable data presents, push is blocked.
 */
template <typename T>
class QueryBuffer {
 public:
  using LockGuard = std::lock_guard<std::mutex>;
  using UniqueLock = std::unique_lock<std::mutex>;

  QueryBuffer(const size_t size = 1) : size_{size} {}

  bool push(const T& qdata, const bool discardable = true) {
    UniqueLock lock(mutex_);

    // a consistency check
    if (curr_size_ != (queries_.size() + nondiscardable_queries_.size()))
      throw std::runtime_error("QueryBuffer: inconsistent size");
    if (curr_size_ > size_)
      throw std::runtime_error("QueryBuffer: size exceeded");

    bool discarded = false;

    // move all non-discardable data to the nondiscardable queue
    while ((!queries_.empty()) && queries_.front().second == false) {
      nondiscardable_queries_.push(queries_.front().first);
      queries_.pop();
    }

    // see if we can add this in by discarding the oldest discardable data
    if (curr_size_ == size_) {
      // we have no space left for discardable data
      if (nondiscardable_queries_.size() == size_) {
        if (discardable) {
          discarded = true;
        } else {
          while (curr_size_ == size_) cv_not_full_.wait(lock);
          queries_.push(std::make_pair(qdata, discardable));
          curr_size_++;
        }
      }
      // we can discard the oldest discardable data
      else {
        queries_.push(std::make_pair(qdata, discardable));
        queries_.pop();
        discarded = true;
      }
    }
    // add directly since the buffer is not full
    else {
      queries_.push(std::make_pair(qdata, discardable));
      curr_size_++;
    }
    cv_not_empty_.notify_one();
    cv_size_changed_.notify_all();
    return discarded;
  }

  T pop() {
    UniqueLock lock(mutex_);
    while (curr_size_ == 0) cv_not_empty_.wait(lock);
    // if there are nondiscardable queries, pop from nondiscardable queries
    auto query = [&]() {
      if (!nondiscardable_queries_.empty()) {
        auto query = nondiscardable_queries_.front();
        nondiscardable_queries_.pop();
        return query;
      } else {
        auto query = queries_.front().first;
        queries_.pop();
        return query;
      }
    }();
    --curr_size_;
    cv_not_full_.notify_one();
    cv_size_changed_.notify_all();
    return query;
  }

  void wait(const size_t size = 0) {
    UniqueLock lock(mutex_);
    while (curr_size_ != size) cv_size_changed_.wait(lock);
  }

 private:
  /** \brief Buffer maximum size */
  const size_t size_;

  /** \brief Protects the queries_ queue */
  std::mutex mutex_;
  /** \brief Condition to wait when queue is full */
  std::condition_variable cv_not_full_;
  /** \brief Condition to wait when queue is empty */
  std::condition_variable cv_not_empty_;
  /** \brief Condition to wait until the size of buffer changes */
  std::condition_variable cv_size_changed_;

  /** \brief Current queue size */
  size_t curr_size_ = 0;
  /** \brief Queue of discardable + nondiscardable queries */
  std::queue<std::pair<T, bool>> queries_;
  /** \brief Queue of nondiscardable queries */
  std::queue<T> nondiscardable_queries_;
};

/**
 * \brief State estimation pipeline execution model. Can inherit from this class
 * for testing and debugging concurrency problems.
 */
class PipelineInterface {
 public:
  using PipelineMutex = std::recursive_timed_mutex;
  using PipelineLock = std::unique_lock<PipelineMutex>;

  PipelineInterface(const Graph::Ptr& graph, const size_t& num_async_threads,
                    const size_t& async_queue_size);

  /** \brief Subclass must call join due to inheritance. */
  virtual ~PipelineInterface() { join(); }

  void join();

  /**
   * \brief Stops passing query data into the pipeline immediately and then
   * waits until all pipeline threads finishes.
   */
  PipelineLock lockPipeline();

  /** \brief Pipline entrypoint, gets query input from navigator */
  void input(const QueryCache::Ptr& qdata);

 private:
  /** \brief Data preprocessing thread, input->preprocess->odo&mapping */
  void preprocess();
  /** \brief Odometry & mapping thread, preprocess->odo&mapping->localization */
  void runOdometryMapping();
  /** \brief Localization thread, odomtry&mapping->localization */
  void runLocalization();

  /** \brief Accepts the input data */
  virtual bool input_(const QueryCache::Ptr& qdata) = 0;
  /** \brief Performs the actual preprocessing task */
  virtual bool preprocess_(const QueryCache::Ptr& qdata) = 0;
  /** \brief Performs the actual odometry mapping task */
  virtual bool runOdometryMapping_(const QueryCache::Ptr& qdata) = 0;
  /** \brief Performs the actual localization task */
  virtual bool runLocalization_(const QueryCache::Ptr& qdata) = 0;

 protected:
  AsyncTaskExecutor::Ptr task_queue_;

 private:
  PipelineMutex pipeline_mutex_;
  common::joinable_semaphore pipeline_semaphore_{0};

  QueryBuffer<QueryCache::Ptr> preprocessing_buffer_{1};
  QueryBuffer<QueryCache::Ptr> odometry_mapping_buffer_{1};
  QueryBuffer<QueryCache::Ptr> localization_buffer_{1};

  std::thread preprocessing_thread_;
  std::thread odometry_mapping_thread_;
  std::thread localization_thread_;
};

class TacticCallbackInterface;

class TacticV2 : public PipelineInterface {
 public:
  struct Config {
    using UniquePtr = std::unique_ptr<Config>;

    /** \brief Configuration for the localization chain */
    LocalizationChain::Config chain_config;

    /** \brief Number of threads for the async task queue */
    int task_queue_num_threads = 1;
    /** \brief Maximum number of queued tasks in task queue */
    int task_queue_size = -1;

    bool preprocessing_skippable = false;
    bool odometry_mapping_skippable = false;

    /** \brief Whether localization is skippable */
    bool localization_skippable = true;
    /** \brief Whether to perform localization only on keyframe data */
    bool localization_only_keyframe = false;
    /** \brief Default localization covariance when chain is not localized. */
    Eigen::Matrix<double, 6, 6> default_loc_cov =
        Eigen::Matrix<double, 6, 6>::Zero();

    /** \brief Whether to extrapolate using STEAM trajectory for path tracker */
    bool extrapolate_odometry = false;

    /** \brief Threshold for merging <x, y, theta> */
    std::vector<double> merge_threshold{0.5, 0.25, 0.2};

    /** \brief Visualize odometry and localization via Rviz. */
    bool visualize = false;
    Eigen::Matrix<double, 3, 1> vis_loc_path_offset =
        Eigen::Matrix<double, 3, 1>::Zero();

    static UniquePtr fromROS(const rclcpp::Node::SharedPtr& node,
                             const std::string& prefix = "tactic");
  };

  TacticV2(Config::UniquePtr config, const BasePipeline::Ptr& pipeline,
           const Graph::Ptr& graph,
           const std::shared_ptr<TacticCallbackInterface>& callback =
               std::make_shared<TacticCallbackInterface>());

  ~TacticV2() { join(); }

 public:
  /** \brief Changes the pipeine behavior based on current operation mode */
  void setPipeline(const PipelineMode& pipeline_mode);

  void addRun(const bool ephemeral = false);

  void setPath(const VertexId::Vector& path, bool follow);

 private:
  /** \brief Performs the actual preprocessing task */
  bool input_(const QueryCache::Ptr& qdata) override;

  /** \brief Performs the actual preprocessing task */
  bool preprocess_(const QueryCache::Ptr& qdata) override;

  /** \brief Performs the actual odometry mapping task */
  bool runOdometryMapping_(const QueryCache::Ptr& qdata) override;
  bool branchOdometryMapping(const QueryCache::Ptr& qdata);
  bool mergeOdometryMapping(const QueryCache::Ptr& qdata);
  bool searchOdometryMapping(const QueryCache::Ptr& qdata);
  bool followOdometryMapping(const QueryCache::Ptr& qdata);

  /** \brief Performs the actual localization task */
  bool runLocalization_(const QueryCache::Ptr& qdata) override;
  bool branchLocalization(const QueryCache::Ptr& qdata);
  bool mergeLocalization(const QueryCache::Ptr& qdata);
  bool searchLocalization(const QueryCache::Ptr& qdata);
  bool followLocalization(const QueryCache::Ptr& qdata);

 private:
  /// pipeline helper functions and states
  void addDanglingVertex(const storage::Timestamp& stamp);
  void addConnectedVertex(const storage::Timestamp& stamp,
                          const EdgeTransform& T_r_m, const bool manual);

  /**
   * \brief Whether this is the first frame of this run, only used by
   * preprocessing thread.
   * \note Only change this variable when pipeline is locked.
   */
  bool first_frame_ = true;

  /**
   * \brief Vertex id of the latest keyframe, only used by odometry thread
   * \note initialized to invalid, Only change this when pipeline is locked
   */
  VertexId current_vertex_id_ = VertexId((uint64_t)-1);

 private:
  Config::UniquePtr config_;

  PipelineMode pipeline_mode_;
  BasePipeline::Ptr pipeline_;
  LocalizationChain::Ptr chain_;
  Graph::Ptr graph_;

 private:
  /// robot status update related
  void updatePersistentLoc(const storage::Timestamp& t, const VertexId& v,
                           const EdgeTransform& T_r_v, bool localized,
                           bool reset_success = true);
  void updateTargetLoc(const storage::Timestamp& t, const VertexId& v,
                       const EdgeTransform& T_r_v, bool localized,
                       bool reset_success = true);

  std::shared_ptr<TacticCallbackInterface> callback_;

  /** \brief Protects: persistent_loc_, target_loc_ */
  std::mutex robot_statue_mutex_;
  /** \brief Localization against the map, that persists across runs. */
  Localization persistent_loc_;
  /** \brief Localization against a target for merging. */
  Localization target_loc_;

  /// \note updates to these variables are protected by the tactic mutex.
  /** \brief Transformation from the latest keyframe to world frame */
  EdgeTransform T_w_m_odo_ = EdgeTransform(true);
  /** \brief Transformation from the localization keyframe to world frame */
  EdgeTransform T_w_m_loc_ = EdgeTransform(true);
  std::vector<PoseStampedMsg> keyframe_poses_;
  std::vector<EdgeTransform> odometry_poses_;

  friend class TacticCallbackInterface;
  friend class TacticCallback;
};

class TacticCallbackInterface {
 public:
  using Ptr = std::shared_ptr<TacticCallbackInterface>;

  virtual void publishRobotUI(const TacticV2&) {}
  virtual void publishPathUI(const TacticV2&) {}
  virtual void clearPathUI(const TacticV2&) {}

  virtual void publishOdometryRviz(const TacticV2&, const QueryCache&) {}
  virtual void publishPathRviz(const TacticV2&) {}
  virtual void publishLocalizationRviz(const TacticV2&, const QueryCache&) {}
};

}  // namespace tactic
}  // namespace vtr