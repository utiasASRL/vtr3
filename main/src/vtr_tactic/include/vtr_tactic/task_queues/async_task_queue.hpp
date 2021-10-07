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
 * \file async_task_queue.hpp
 * \brief AsyncTaskExecutor and BaseTask class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <list>
#include <mutex>
#include <queue>
#include <stdexcept>

#include "vtr_common/utils/semaphore.hpp"
#include "vtr_common/utils/type_traits.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace tactic {

class AsyncTaskExecutor;

class BaseTask : public std::enable_shared_from_this<BaseTask> {
 public:
  using Ptr = std::shared_ptr<BaseTask>;

  BaseTask(const unsigned& priority0 = 0,
           const VertexId& vid0 = VertexId::Invalid())
      : priority(priority0), vid(vid0) {}

  virtual ~BaseTask() = default;

  /**
   * \brief Executes the task
   * \note If the task depends on any other task, then follow this convention:
   *  - check if the dependency is met, if not, instantiate the dependency and
   * add it to the queue with this->priority+1, then add *this task to the queue
   * with the same priority
   * \note It is possible that the same task will be launched twice due to
   * inter-dependency. Say that task B depends on task A and they are launched
   * at the same time, it is possible that A and B starts at the same time as
   * well, so that (following the above convention) B adds another A to the
   * queue and then B itself to the queue. Therefore, either use a mutex to lock
   * resources, preventing B from running while A runs, or
   */
  virtual void run(const std::shared_ptr<AsyncTaskExecutor>& executor,
                   const Graph::Ptr& graph) = 0;

 protected:
  const unsigned priority;
  const VertexId vid;

  friend class AsyncTaskExecutor;
};

/**
 * \brief An executor that executes tasks asynchronously. Priority can be
 * specified in range [0, infty), with 0 being the lowest.
 */
class AsyncTaskExecutor
    : public std::enable_shared_from_this<AsyncTaskExecutor> {
  using ulock_t = std::unique_lock<std::mutex>;
  using lockg_t = std::lock_guard<std::mutex>;
  using cv_t = std::condition_variable;
  using sem_t = common::bounded_joinable_semaphore;
  using sem_guard_t = common::semaphore_guard<sem_t>;

 public:
  using Ptr = std::shared_ptr<AsyncTaskExecutor>;

  using ValueType = std::pair<unsigned, BaseTask::Ptr>;
  /// A customized comparator
  struct Compare {
    constexpr bool operator()(const ValueType& lhs,
                              const ValueType& rhs) const {
      return lhs.first < rhs.first;
    }
  };

  using QueueType =
      std::priority_queue<ValueType, std::vector<ValueType>, Compare>;

  /**
   * \brief spawn the threads
   * \param[in] num_threads the number of threads in the pool
   * \param[in] queue_length the maximum number of queued jobs
   * \param[in] graph pose graph pointer for data read/write
   */
  AsyncTaskExecutor(const Graph::Ptr& graph, unsigned num_threads,
                    size_t queue_length = 0);

  /** \brief \note subclass should call join() to clean up the threads */
  virtual ~AsyncTaskExecutor() { join(); };

  /** \brief starts all threads */
  void start();

  /** \brief clears all the pending jobs, but let the threads keep running */
  void clear();

  /** \brief removes all pending jobs and stops all threads (non-blocking). */
  void stop();

  /** \brief removes all pending jobs, and join all threads (blocking). */
  void join();

  /** \brief waits for all jobs to finish */
  void wait() { job_count_.wait(0); }

  /** \brief returns the (approximate) idle state of the pool (non-blocking) */
  inline bool isIdleApprox() const { return job_count_.get_value() == 0; }

  /** \brief returns the idle state of the pool */
  inline bool isIdle() {
    ulock_t lck(mutex_);
    return job_count_.get_value() == 0;
  }

  /** \brief returns the (approximate) number of pending jobs (non-blocking) */
  inline size_t pendingApprox() const { return job_count_.get_value(); }

  /** \brief returns the number of pending jobs */
  inline size_t pending() {
    ulock_t lck(mutex_);
    return job_count_.get_value();
  }

  /** \brief checks to see if the threads are running */
  bool joinable() {
    ulock_t ulock(mutex_);
    return !threads_.empty();
  }

  void dispatch(const BaseTask::Ptr& task);

  void tryDispatch(const BaseTask::Ptr& task);

 private:
  void _dispatch(const BaseTask::Ptr& task);

  /** \brief This is what the thread actually runs */
  void doWork();

  /// pointer to the pose graph for data reading/writing
  const Graph::Ptr graph_;
  /// number of threads allowed in the pool
  const unsigned num_threads_;
  /// counts the number of jobs (pending or in the queue)
  sem_t job_count_;
  /// stop flag for the threads to commit suicide
  bool stop_ = true;
  /// condition for all the unemployed threads to sleep on
  std::condition_variable sleeping_;
  /// the threads!!!
  std::list<std::thread> threads_;
  /// protects: jobs_, stop_, job_count_
  std::mutex mutex_;
  /// pending jobs that await a worker
  QueueType jobs_;
};

}  // namespace tactic
}  // namespace vtr
