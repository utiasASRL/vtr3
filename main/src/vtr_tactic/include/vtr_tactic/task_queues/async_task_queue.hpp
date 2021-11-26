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
 * \brief TaskExecutor and Task class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <limits>
#include <list>
#include <mutex>
#include <queue>
#include <stdexcept>

#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

#include "vtr_common/utils/semaphore.hpp"
#include "vtr_common/utils/type_traits.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace tactic {

class TaskExecutor;

class Task {
 public:
  using Id = unsigned long;
  using Priority = size_t;
  using DepId = boost::uuids::uuid;
  using DepIdHash = boost::hash<boost::uuids::uuid>;
  using DepIdSet = std::unordered_set<DepId, DepIdHash>;

  using Ptr = std::shared_ptr<Task>;

  /** \brief Thread safe unique id generator */
  static Id getId() {
    static Id id = 0;
    static std::mutex mutex;
    std::lock_guard<std::mutex> guard(mutex);
    return ++id;
  }

  Task(const BaseModule::Ptr& module, const QueryCache::Ptr& qdata,
       const unsigned& priority0 = 0,
       const DepId& dep_id0 = boost::uuids::random_generator()(),
       const DepIdSet& dependencies0 = {},
       const VertexId& vid0 = VertexId::Invalid())
      : module_(module),
        qdata_(qdata),
        priority(priority0),
        dep_id(dep_id0),
        dependencies(dependencies0),
        vid(vid0) {}

  virtual ~Task() = default;

  /**
   * \brief Executes the task
   * \note If the task depends on any other task, then follow this convention:
   *  - check if the dependency is met, if not, instantiate the dependency and
   * add it to the queue with this->priority+1, then add *this task to the queue
   * with the same priority and dep_id
   * \note It is possible that the same task will be launched twice due to
   * inter-dependency. Say that task B depends on task A and they are launched
   * at the same time, it is possible that A and B starts at the same time as
   * well, so that (following the above convention) B adds another A to the
   * queue and then B itself to the queue. Therefore, either use a mutex to lock
   * resources, preventing B from running while A runs, or
   * \note when there is a dependency, user should make sure that the dependency
   * is added to the task queue before the task itself.
   */
  void run(const std::shared_ptr<TaskExecutor>& executor,
           const Graph::Ptr& graph) {
    module_->runAsync(*qdata_, graph, executor, priority, dep_id);
  }

  void refreshId() { id = getId(); }

  void adjustPriority(const int& delta) {
    priority = int(priority) + delta > 0 ? int(priority) + delta : 0;
  }

 private:
  const BaseModule::Ptr module_;
  const QueryCache::Ptr qdata_;

 public:
  Id id = getId();  // keep track of order of task launch
  Priority priority;
  const DepId dep_id;  // keep tracker of dependent tasks
  const std::unordered_set<DepId, DepIdHash> dependencies;
  const VertexId vid;
};

class TaskQueue {
 public:
  using LockGuard = std::lock_guard<std::mutex>;
  using UniqueLock = std::unique_lock<std::mutex>;

  TaskQueue(const size_t& size = std::numeric_limits<std::size_t>::max())
      : size_(size) {}

  bool push(const Task::Ptr& task) {
    UniqueLock lock(mutex_);

    if (tasks_.size() > size_)
      throw std::runtime_error("TaskQueue: size exceeded");

    // insert the task into the queue
    auto iter = queue_.try_emplace(task->priority);
    auto success = iter.first->second.emplace(task->id).second;
    success |= tasks_.emplace(task->id, task).second;
    if (!success)
      throw std::runtime_error(
          "TaskQueue: inserting a task that already exists");
    // insert the task into the dependency map
    auto deps_iter = deps_.try_emplace(task->dep_id, 1);
    if (!deps_iter.second) ++deps_.at(task->dep_id);
#if false
    CLOG(DEBUG, "tactic.async_task")
        << "Added task of id: " << task->id << ", priority: " << task->priority
        << ", dep id: " << task->dep_id << "(count:" << deps_.at(task->dep_id)
        << ") to queue.";
#endif
    bool discarded = false;
    if (tasks_.size() > size_) {
      // discard the task with the lowest priority and added earliest
      const auto iter = queue_.begin();
      const auto priority = iter->first;
      const auto id = *(iter->second.begin());
      const auto dep_id = tasks_.at(id)->dep_id;
      if ((--deps_.at(dep_id)) == 0) deps_.erase(dep_id);
      tasks_.erase(id);
      iter->second.erase(id);
      if (queue_.at(priority).empty()) queue_.erase(priority);
      CLOG(DEBUG, "tactic.async_task")
          << "Discarded task of id: " << id << ", priority: " << priority
          << ", dep id: " << dep_id
          << "(count:" << (deps_.count(dep_id) ? deps_.at(dep_id) : 0)
          << ") from queue.";

      discarded = true;
    }

    return discarded;
  }

  Task::Ptr pop() {
    UniqueLock lock(mutex_);

    if (tasks_.empty())
      throw std::runtime_error("TaskQueue: task queue is empty");

    const auto iter = queue_.rbegin();
    const auto priority = iter->first;
    const auto id = *(iter->second.begin());
    const auto task = tasks_.at(id);
    const auto dep_id = task->dep_id;
    if ((--deps_.at(dep_id)) == 0) deps_.erase(dep_id);
    tasks_.erase(id);
    iter->second.erase(id);
    if (queue_.at(priority).empty()) queue_.erase(priority);
#if false
    CLOG(DEBUG, "tactic.async_task")
        << "Popped task of id: " << task->id << ", priority: " << task->priority
        << ", dep id: " << task->dep_id
        << "(count:" << (deps_.count(task->dep_id) ? deps_.at(task->dep_id) : 0)
        << ") from queue.";
#endif
    return task;
  }

  bool empty() const {
    LockGuard lock(mutex_);
    return tasks_.empty();
  }

  size_t size() const {
    LockGuard lock(mutex_);
    return tasks_.size();
  }

  bool contains(const Task::DepId& id) const {
    LockGuard lock(mutex_);
    return deps_.find(id) != deps_.end();
  }

 private:
  const size_t size_;

  /** \brief Protects all members below, cv should release this mutex */
  mutable std::mutex mutex_;

  /** \brief The queue of tasks, ordered by priority and then job id */
  std::map<Task::Priority, std::set<Task::Id>> queue_;
  std::map<Task::Id, Task::Ptr> tasks_;
  /** \brief A id of every task for dependency tracking */
  std::unordered_map<Task::DepId, unsigned, Task::DepIdHash> deps_;
};

/**
 * \brief An executor that executes tasks asynchronously. Priority can be
 * specified in range [0, infty), with 0 being the lowest.
 * \note User ensures that start() and stop() do not run simutaneously.
 */
class TaskExecutor : public std::enable_shared_from_this<TaskExecutor> {
  using Mutex = std::mutex;
  using UniqueLock = std::unique_lock<Mutex>;
  using LockGuard = std::lock_guard<Mutex>;
  using Semaphore = common::bounded_joinable_semaphore;

 public:
  using Ptr = std::shared_ptr<TaskExecutor>;

  /**
   * \brief spawn the threads
   * \param[in] num_threads the number of threads in the pool
   * \param[in] queue_length the maximum number of queued jobs
   * \param[in] graph pose graph pointer for data read/write
   */
  TaskExecutor(const Graph::Ptr& graph, const unsigned num_threads,
               const size_t queue_length = 0);

  /** \brief \note subclass should call stop() to clean up the threads */
  ~TaskExecutor() { stop(); }

  /** \brief starts all threads */
  void start();

  /** \brief removes all pending jobs and stops all threads (non-blocking). */
  void stop();

  /** \brief wait until all job finishes */
  void wait() {
    UniqueLock lock(mutex_);
    while (job_count_.get_value() > 0) cv_job_empty_.wait(lock);
  }

  /** \brief returns the (approximate) idle state of the pool (non-blocking) */
  bool isIdleApprox() const { return job_count_.get_value() == 0; }

  /** \brief returns the idle state of the pool */
  bool isIdle() {
    UniqueLock lck(mutex_);
    return job_count_.get_value() == 0;
  }

  /** \brief returns the (approximate) number of pending jobs (non-blocking) */
  size_t pendingApprox() const { return job_count_.get_value(); }

  /** \brief returns the number of pending or running jobs */
  size_t pending() {
    UniqueLock lck(mutex_);
    return job_count_.get_value();
  }

  void dispatch(const Task::Ptr& task);

 private:
  /** \brief This is what the thread actually runs */
  void doWork();

  /** \brief pointer to the pose graph for data reading/writing */
  const Graph::Ptr graph_;
  /** \brief number of threads allowed in the pool */
  const unsigned num_threads_;

  /**
   * \brief protects: stop_, thread_count_, threads_, task_queue_, job_count_
   * and running_ids_, condition variable wait on this mutex
   */
  Mutex mutex_;
  /** \brief wait until the task queue is not empty */
  std::condition_variable cv_stop_or_queue_not_empty_;
  /** \brief wait until a worker thread has finished (returned) */
  std::condition_variable cv_thread_finish_;
  /** \brief wait until there is no job running or in queue */
  std::condition_variable cv_job_empty_;

  /** \brief stop flag for the threads to commit suicide */
  bool stop_ = true;
  /** \brief current threads that are running jobs */
  size_t thread_count_ = 0;
  /** \brief the threads!!! */
  std::list<std::thread> threads_;
  /** \brief current running job ids */
  std::unordered_map<Task::DepId, unsigned, Task::DepIdHash> running_deps_;
  /** \brief pending jobs that await a worker */
  TaskQueue task_queue_;
  /** \brief counts the number of jobs (pending or in the queue) */
  Semaphore job_count_;
};

}  // namespace tactic
}  // namespace vtr
