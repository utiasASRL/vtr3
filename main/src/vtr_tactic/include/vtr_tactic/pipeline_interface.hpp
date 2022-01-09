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
 * \file pipeline_interface.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/task_queue.hpp"
#include "vtr_tactic/types.hpp"

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

  QueryBuffer(const size_t size)
      : size_(size == 0 ? std::numeric_limits<size_t>::max() : size),
        require_immediate_pop_(size == 0 ? true : false) {}

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

    if (require_immediate_pop_) {
      if (waiting_count_ < 0)
        throw std::runtime_error("QueryBuffer: waiting count smaller than 0");
      if (waiting_count_ == 0) {
        if (discardable) {
          discarded = true;
        } else {
          cv_has_waiting_.wait(lock, [&] { return waiting_count_ > 0; });
          queries_.push(std::make_pair(qdata, discardable));
          curr_size_++;
          waiting_count_--;
          CLOG(DEBUG, "tactic")
              << "In push nondirect, current number of waiting "
              << waiting_count_;
          cv_size_changed_.notify_all();
          cv_not_empty_.notify_one();
        }
      } else {
        queries_.push(std::make_pair(qdata, discardable));
        curr_size_++;
        waiting_count_--;
        CLOG(DEBUG, "tactic")
            << "In push direct, current number of waiting " << waiting_count_;
        cv_size_changed_.notify_all();
        cv_not_empty_.notify_one();
      }
    } else {
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
            cv_size_changed_.notify_all();
            cv_not_empty_.notify_one();
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
        cv_size_changed_.notify_all();
        cv_not_empty_.notify_one();
      }
    }
    return discarded;
  }

  T pop() {
    UniqueLock lock(mutex_);
    while (curr_size_ == 0) {
      if (require_immediate_pop_) {
        if (waiting_count_ < 0)
          throw std::runtime_error("QueryBuffer: waiting count smaller than 0");
        waiting_count_++;
        CLOG(DEBUG, "tactic")
            << "In pop, current number of waiting " << waiting_count_;
        cv_has_waiting_.notify_one();
      }
      cv_not_empty_.wait(lock);
    }
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
  const bool require_immediate_pop_;

  /** \brief Protects all members below, cv should release this mutex */
  std::mutex mutex_;
  /** \brief Wait until some thread is trying to pop but the queue is empty */
  std::condition_variable cv_has_waiting_;
  /** \brief Wait until the queue is not full */
  std::condition_variable cv_not_full_;
  /** \brief Wait until the queue is not empty */
  std::condition_variable cv_not_empty_;
  /** \brief Wait until the size of buffer changes */
  std::condition_variable cv_size_changed_;

  /** \brief Current queue size */
  size_t curr_size_ = 0;
  /** \brief Current number of threads waiting for data */
  int waiting_count_ = 0;
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

  using TaskQueueCallback = TaskExecutor::Callback;

  PipelineInterface(const bool& enable_parallelization,
                    const OutputCache::Ptr& output, const Graph::Ptr& graph,
                    const size_t& num_async_threads,
                    const size_t& async_queue_size,
                    const TaskQueueCallback::Ptr& task_queue_callback =
                        std::make_shared<TaskQueueCallback>());

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
  void inputSequential(const QueryCache::Ptr& qdata);
  void inputParallel(const QueryCache::Ptr& qdata);

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
  TaskExecutor::Ptr task_queue_;

 private:
  const bool enable_parallelization_;

  PipelineMutex pipeline_mutex_;
  common::joinable_semaphore pipeline_semaphore_{0};

  QueryBuffer<QueryCache::Ptr> preprocessing_buffer_{0};
  QueryBuffer<QueryCache::Ptr> odometry_mapping_buffer_{0};
  QueryBuffer<QueryCache::Ptr> localization_buffer_{0};

  std::thread preprocessing_thread_;
  std::thread odometry_mapping_thread_;
  std::thread localization_thread_;
};

}  // namespace tactic
}  // namespace vtr