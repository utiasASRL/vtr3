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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 * \brief TaskExecutor and Task class definition
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
       const std::string& name0 = "anonymous",
       const VertexId& vid0 = VertexId::Invalid())
      : module_(module),
        qdata_(qdata),
        priority(priority0),
        dep_id(dep_id0),
        dependencies(dependencies0),
        name(name0),
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
  const std::string name;
  const VertexId vid;
};

class TaskQueue {
 public:
  using LockGuard = std::lock_guard<std::mutex>;
  using UniqueLock = std::unique_lock<std::mutex>;

  TaskQueue(const size_t& size = std::numeric_limits<std::size_t>::max())
      : size_(size) {}

  std::tuple<bool, Task::Id> push(const Task::Ptr& task);
  Task::Ptr pop();
  void updateDeps(const Task::Ptr& task);

  void clear();

  bool hasNext() const;
  bool empty() const;
  size_t size() const;

 private:
  /** \brief Remove task with this id or a task depends on it */
  Task::Id removeLeaf(const Task::Id id);

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

class TaskExecutorCallbackInterface {
 public:
  using Ptr = std::shared_ptr<TaskExecutorCallbackInterface>;

  virtual ~TaskExecutorCallbackInterface() = default;

  virtual void taskAdded(const Task::Ptr& /* task */) {}
  virtual void taskRemoved(const Task::Id& /* id */,
                           const bool /* completed */) {}
};

/**
 * \brief An executor that executes tasks asynchronously. Priority can be
 * specified in range [0, infty), with 0 being the lowest.
 * \note User ensures that start() and stop() do not run simutaneously.
 */
class TaskExecutor : public std::enable_shared_from_this<TaskExecutor> {
 public:
  using Ptr = std::shared_ptr<TaskExecutor>;

  using Mutex = std::mutex;
  using UniqueLock = std::unique_lock<Mutex>;
  using LockGuard = std::lock_guard<Mutex>;
  using Semaphore = common::bounded_joinable_semaphore;

  using Callback = TaskExecutorCallbackInterface;

  /**
   * \brief spawn the threads
   * \param[in] output output cache pointer for data read/write
   * \param[in] graph pose graph pointer for data read/write
   * \param[in] num_threads the number of threads in the pool
   * \param[in] queue_length the maximum number of queued jobs
   */
  TaskExecutor(const OutputCache::Ptr& output, const Graph::Ptr& graph,
               const unsigned num_threads, const size_t queue_length = 0,
               const Callback::Ptr& callback = std::make_shared<Callback>());

  /** \brief \note subclass should call stop() to clean up the threads */
  ~TaskExecutor() { stop(); }

  /** \brief starts all threads */
  void start();

  /** \brief removes all pending jobs and stops all threads (non-blocking). */
  void stop();

  /** \brief wait until all job finishes */
  void wait() const;

  /** \brief returns the idle state of the pool */
  bool isIdle() const;

  /** \brief returns the number of pending or running jobs */
  size_t pending() const;

  void dispatch(const Task::Ptr& task);

 private:
  /** \brief This is what the thread actually runs */
  void doWork(const size_t& thread_id);

  /** \brief Pointer to the output cache (localization chain etc) */
  const OutputCache::Ptr output_;
  /** \brief pointer to the pose graph for data reading/writing */
  const Graph::Ptr graph_;
  /** \brief number of threads allowed in the pool */
  const unsigned num_threads_;

  /**
   * \brief protects: stop_, thread_count_, threads_, task_queue_, job_count_,
   * condition variable wait on this mutex
   */
  mutable Mutex mutex_;
  /** \brief wait until the task queue is not empty */
  mutable std::condition_variable cv_stop_or_queue_has_next_;
  /** \brief wait until there is no job running or in queue */
  mutable std::condition_variable cv_job_maybe_empty_;
  /** \brief wait until a worker thread has finished (returned) */
  mutable std::condition_variable cv_thread_finish_;

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

  /** \brief callback on task queue/executor update */
  const Callback::Ptr callback_;
};

}  // namespace tactic
}  // namespace vtr
