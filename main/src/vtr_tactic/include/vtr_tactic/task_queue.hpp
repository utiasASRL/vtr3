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
 * \file task_queue.hpp
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

namespace vtr {
namespace tactic {

class TaskExecutor;

class Task {
 public:
  using Id = unsigned;
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
       const unsigned& priority0 = 0, const DepIdSet& dependencies0 = {},
       const DepId& dep_id0 = boost::uuids::random_generator()(),
       const VertexId& vid0 = VertexId::Invalid())
      : module_(module),
        qdata_(qdata),
        priority(priority0),
        dep_id(dep_id0),
        dependencies(dependencies0),
        vid(vid0) {}

  /**
   * \brief Executes the task
   * \note when there is a dependency, user MUST make sure that the dependency
   * is added to the task queue before the task itself.
   */
  void run(const std::shared_ptr<TaskExecutor>& executor,
           const OutputCache::Ptr& output, const Graph::Ptr& graph) {
    module_->runAsync(*qdata_, *output, graph, executor, priority, dep_id);
  }

 private:
  const BaseModule::Ptr module_;
  const QueryCache::Ptr qdata_;

 public:
  /** \brief a unique id for this task guaranteed to be incremental */
  Id id = getId();  // keep track of order of task launch
  /** \brief the priority of this task */
  const Priority priority;
  /** \brief the dependency id of this task (no necessarily unique) */
  const DepId dep_id;
  /** \brief the set of dependencies for this task, updated by the task queue */
  std::unordered_set<DepId, DepIdHash> dependencies;
  /** \brief for GUI visualization only */
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

    if (id2task_map_.size() > size_)
      throw std::runtime_error("TaskQueue: size exceeded");

    {
      auto success = id2task_map_.try_emplace(task->id, task).second;
      auto iter = complete_queue_.try_emplace(task->priority);
      success |= iter.first->second.emplace(task->id).second;
      if (!success)
        throw std::runtime_error(
            "TaskQueue: inserting a task that already exists");
    }

    {
      // insert the task into the dependency map for tasks that depend on it
      auto iter = deps2count_map_.try_emplace(task->dep_id, 1);
      if (!iter.second) ++deps2count_map_.at(task->dep_id);
      //
      depid2ids_map_.try_emplace(task->dep_id);
    }

    // keep track of the dependencies of this task
    const auto deps = task->dependencies;
    for (const auto& dep : deps) {
      if (depid2ids_map_.count(dep) == 0) {
        // dep not in the queue means it has already finished
        // this is why dependencies must be added first!
        task->dependencies.erase(dep);
      } else {
        depid2ids_map_.at(dep).emplace(task->id);
      }
    }

    // add this task to the executable queue if it does not have any dependency
    // left
    if (task->dependencies.empty()) {
      CLOG(DEBUG, "tactic.async_task")
          << "Task of id: " << task->id << ", priority: " << task->priority
          << ", dep id: " << task->dep_id
          << " has all dependency met, added to executable queue.";
      auto iter = executable_queue_.try_emplace(task->priority);
      const auto success = iter.first->second.emplace(task->id).second;
      if (!success)
        throw std::runtime_error(
            "TaskQueue: inserting a task that already exists");
    }

    CLOG(DEBUG, "tactic.async_task")
        << "Added task of id: " << task->id << ", priority: " << task->priority
        << ", dep id: " << task->dep_id << " to queue.";

    bool discarded = false;
    if (id2task_map_.size() > size_) {
      // discard the task with the lowest priority and added earliest or the
      // task that recursively depends it if exists
      removeLeaf(*(complete_queue_.begin()->second.begin()));
      discarded = true;
    }

    return discarded;
  }

  Task::Ptr pop() {
    UniqueLock lock(mutex_);

    if (executable_queue_.empty())
      throw std::runtime_error("TaskQueue: executable queue is empty when pop");

    const auto id = *(executable_queue_.rbegin()->second.begin());
    const auto task = id2task_map_.at(id);
    const auto priority = task->priority;

    // remove this task from the executable queue
    executable_queue_.at(priority).erase(id);
    if (executable_queue_.at(priority).empty())
      executable_queue_.erase(priority);

    // remove this task from the complete queue
    complete_queue_.at(priority).erase(id);
    if (complete_queue_.at(priority).empty()) complete_queue_.erase(priority);

    //
    id2task_map_.erase(id);

    CLOG(DEBUG, "tactic.async_task")
        << "Popped task of id: " << task->id << ", priority: " << task->priority
        << ", dep id: " << task->dep_id << " from queue.";

    return task;
  }

  void updateDeps(const Task::Ptr& task) {
    if ((--deps2count_map_.at(task->dep_id)) != 0) return;

    // if count of this dep id goes to zero then add its dependent tasks back
    for (const auto& id : depid2ids_map_.at(task->dep_id)) {
      auto curr_task = id2task_map_.at(id);
      curr_task->dependencies.erase(task->dep_id);
      if (curr_task->dependencies.empty()) {
        CLOG(DEBUG, "tactic.async_task")
            << "Task of id: " << curr_task->id
            << ", priority: " << curr_task->priority
            << ", dep id: " << curr_task->dep_id
            << " has all dependency met, added to executable queue.";
        auto iter = executable_queue_.try_emplace(curr_task->priority);
        auto success = iter.first->second.emplace(curr_task->id).second;
        if (!success)
          throw std::runtime_error(
              "TaskQueue: inserting a task that already exists");
      }
    }
    deps2count_map_.erase(task->dep_id);
    depid2ids_map_.erase(task->dep_id);
  }

  bool hasNext() const {
    UniqueLock lock(mutex_);
    return !executable_queue_.empty();
  }

  void clear() {
    LockGuard lock(mutex_);
    // remove all tasks in queue
    while (!complete_queue_.empty())
      removeLeaf(*(complete_queue_.begin()->second.begin()));
    if (!id2task_map_.empty() || !executable_queue_.empty())
      throw std::runtime_error(
          "TaskQueue: id2task_map_ is not empty, task queue inconsistent");
  }

  /** \brief This function is for consistency check only. */
  bool empty() const {
    LockGuard lock(mutex_);
    if (id2task_map_.empty()) {
      if (!(complete_queue_.empty() && deps2count_map_.empty() &&
            depid2ids_map_.empty() && executable_queue_.empty()))
        throw std::runtime_error("TaskQueue: inconsistent state");
      return true;
    }
    return false;
  }

  size_t size() const {
    LockGuard lock(mutex_);
    return id2task_map_.size();
  }

 private:
  /** \brief Remove task with this id or a task depends on it */
  void removeLeaf(const Task::Id id) {
    if (!id2task_map_.count(id))
      throw std::runtime_error("TaskQueue: id not found when removing leaf");

    const auto dep_id = id2task_map_.at(id)->dep_id;
    // remove one of the task that depends on this task if exists
    if (!depid2ids_map_.at(dep_id).empty()) {
      removeLeaf(*(depid2ids_map_.at(dep_id).begin()));
      return;
    }
    // otherwise remove this task

    // remove this task from the executable queue (if it is in there)
    const auto priority = id2task_map_.at(id)->priority;
    const auto dependencies = id2task_map_.at(id)->dependencies;

    if (executable_queue_.count(priority) &&
        executable_queue_.at(priority).erase(id)) {
      if (executable_queue_.at(priority).empty())
        executable_queue_.erase(priority);
    }

    // remove this task from the deps tracking maps
    if ((--deps2count_map_.at(dep_id)) == 0) {
      deps2count_map_.erase(dep_id);
      depid2ids_map_.erase(dep_id);
    }

    for (const auto& dep : dependencies) depid2ids_map_.at(dep).erase(id);

    // remove this task from the complete queue
    complete_queue_.at(priority).erase(id);
    if (complete_queue_.at(priority).empty()) complete_queue_.erase(priority);

    //
    id2task_map_.erase(id);

    CLOG(DEBUG, "tactic.async_task")
        << "Discarded task of id: " << id << ", priority: " << priority
        << ", dep id: " << dep_id << " from queue.";
  }

 private:
  const size_t size_;

  /** \brief Protects all members below, cv should release this mutex */
  mutable std::mutex mutex_;

  /** \brief Keep track of all tasks */
  std::unordered_map<Task::Id, Task::Ptr> id2task_map_;

  /**
   * \brief Ordered queue with all task ids, used for discarding tasks when this
   * TaskQueue if full
   */
  std::map<Task::Priority, std::set<Task::Id>> complete_queue_;

  /**
   * \brief Dep Id of unfinished tasks and their count
   * \note We allow multiple tasks with the same dep id, a dependency is
   * considered satisfied only when this count goes to zero - this is
   * essentially for an async task to relaunch itself if it has dependencies,
   * and after the dependencies are launched
   */
  std::unordered_map<Task::DepId, unsigned, Task::DepIdHash> deps2count_map_;

  /** \brief Keep track of the dependent tasks of each task */
  std::unordered_map<Task::DepId, std::set<Task::Id>, Task::DepIdHash>
      depid2ids_map_;

  /**
   * \brief Ordered queue with only dependency met task ids, used for sending
   * out the next executable task
   */
  std::map<Task::Priority, std::set<Task::Id>> executable_queue_;
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
   * \param[in] output output cache pointer for data read/write
   * \param[in] graph pose graph pointer for data read/write
   * \param[in] num_threads the number of threads in the pool
   * \param[in] queue_length the maximum number of queued jobs
   */
  TaskExecutor(const OutputCache::Ptr& output, const Graph::Ptr& graph,
               const unsigned num_threads, const size_t queue_length = 0);

  /** \brief \note subclass should call stop() to clean up the threads */
  ~TaskExecutor() { stop(); }

  /** \brief starts all threads */
  void start();

  /** \brief removes all pending jobs and stops all threads (non-blocking). */
  void stop();

  /** \brief wait until all job finishes */
  void wait() {
    UniqueLock lock(mutex_);
    while (job_count_.get_value() > 0) cv_job_maybe_empty_.wait(lock);
    // consistency check
    if (!task_queue_.empty())
      throw std::runtime_error(
          "TaskQueue: not empty or task queue is in an inconsistent state");
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

  /** \brief Pointer to the output cache (localization chain etc) */
  const OutputCache::Ptr output_;
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
  std::condition_variable cv_stop_or_queue_has_next_;
  /** \brief wait until there is no job running or in queue */
  std::condition_variable cv_job_maybe_empty_;
  /** \brief wait until a worker thread has finished (returned) */
  std::condition_variable cv_thread_finish_;

  /** \brief stop flag for the threads to commit suicide */
  bool stop_ = true;
  /** \brief current threads that are running jobs */
  size_t thread_count_ = 0;
  /** \brief the threads!!! */
  std::list<std::thread> threads_;
  /** \brief pending jobs that await a worker */
  TaskQueue task_queue_;
  /** \brief counts the number of jobs (pending or in the queue) */
  Semaphore job_count_;
};

}  // namespace tactic
}  // namespace vtr
