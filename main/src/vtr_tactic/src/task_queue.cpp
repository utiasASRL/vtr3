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
 * \file task_queue.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 * \brief TaskExecutor class methods definition
 */
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace tactic {

std::tuple<bool, Task::Id> TaskQueue::push(const Task::Ptr& task) {
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
  Task::Id discarded_id;
  if (id2task_map_.size() > size_) {
    // discard the task with the lowest priority and added earliest or the
    // task that recursively depends it if exists
    discarded_id = removeLeaf(*(complete_queue_.begin()->second.begin()));
    discarded = true;
  }

  return std::make_tuple(discarded, discarded_id);
}

Task::Ptr TaskQueue::pop() {
  UniqueLock lock(mutex_);

  if (executable_queue_.empty())
    throw std::runtime_error("TaskQueue: executable queue is empty when pop");

  const auto id = *(executable_queue_.rbegin()->second.begin());
  const auto task = id2task_map_.at(id);
  const auto priority = task->priority;

  // remove this task from the executable queue
  executable_queue_.at(priority).erase(id);
  if (executable_queue_.at(priority).empty()) executable_queue_.erase(priority);

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

void TaskQueue::updateDeps(const Task::Ptr& task) {
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

void TaskQueue::clear() {
  LockGuard lock(mutex_);
  // remove all tasks in queue
  while (!complete_queue_.empty())
    removeLeaf(*(complete_queue_.begin()->second.begin()));
  if (!id2task_map_.empty() || !executable_queue_.empty())
    throw std::runtime_error(
        "TaskQueue: id2task_map_ is not empty, task queue inconsistent");
}

bool TaskQueue::hasNext() const {
  UniqueLock lock(mutex_);
  return !executable_queue_.empty();
}

bool TaskQueue::empty() const {
  LockGuard lock(mutex_);
  if (id2task_map_.empty()) {
    if (!(complete_queue_.empty() && deps2count_map_.empty() &&
          depid2ids_map_.empty() && executable_queue_.empty()))
      throw std::runtime_error("TaskQueue: inconsistent state");
    return true;
  }
  return false;
}

size_t TaskQueue::size() const {
  LockGuard lock(mutex_);
  return id2task_map_.size();
}

Task::Id TaskQueue::removeLeaf(const Task::Id id) {
  if (!id2task_map_.count(id))
    throw std::runtime_error("TaskQueue: id not found when removing leaf");

  const auto dep_id = id2task_map_.at(id)->dep_id;
  // remove one of the task that depends on this task if exists
  if (!depid2ids_map_.at(dep_id).empty())
    return removeLeaf(*(depid2ids_map_.at(dep_id).begin()));

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
  return id;
}

TaskExecutor::TaskExecutor(const OutputCache::Ptr& output,
                           const Graph::Ptr& graph, const unsigned num_threads,
                           const size_t queue_length,
                           const Callback::Ptr& callback)
    : output_(output),
      graph_(graph),
      num_threads_(num_threads),
      task_queue_(queue_length),
      job_count_(0, queue_length == size_t(-1)
                        ? std::numeric_limits<std::size_t>::max()
                        : queue_length + num_threads),
      callback_(callback) {
  // start the threads!
  start();
}

void TaskExecutor::start() {
  LockGuard lock(mutex_);
  stop_ = false;
  thread_count_ = num_threads_;
  // make sure we have enough threads running
  while (threads_.size() < num_threads_)
    threads_.emplace_back(&TaskExecutor::doWork, this, threads_.size());
}

void TaskExecutor::stop() {
  UniqueLock lock(mutex_);

  // wake up and tell the threads to stop
  stop_ = true;
  cv_stop_or_queue_has_next_.notify_all();

  // clear all the pending jobs, empty the job queue
  const auto queue_size = task_queue_.size();
  task_queue_.clear();
  if (queue_size > job_count_.get_value())
    throw std::runtime_error(
        "TaskExecutor::stop(): pending job is greater than job count");
  for (size_t i = 0; i < queue_size; ++i) job_count_.acquire();
  cv_job_maybe_empty_.notify_all();

  // wait for the threads to stop, and destroy them
  while (thread_count_ > 0) cv_thread_finish_.wait(lock);
  while (!threads_.empty()) {
    threads_.front().join();
    threads_.pop_front();
  }

  // consistency check: make sure the queue is empty
  if (!task_queue_.empty())
    throw std::runtime_error(
        "TaskQueue: not empty or task queue is in an inconsistent state");
}

void TaskExecutor::wait() const {
  UniqueLock lock(mutex_);
  while (job_count_.get_value() > 0) cv_job_maybe_empty_.wait(lock);
  // consistency check
  if (!task_queue_.empty())
    throw std::runtime_error(
        "TaskQueue: not empty or task queue is in an inconsistent state");
}

bool TaskExecutor::isIdle() const {
  UniqueLock lck(mutex_);
  return job_count_.get_value() == 0;
}

size_t TaskExecutor::pending() const {
  UniqueLock lck(mutex_);
  return job_count_.get_value();
}

void TaskExecutor::dispatch(const Task::Ptr& task) {
  LockGuard lock(mutex_);
  // make sure we aren't stopped
  if (stop_) return;
  // The pool has been shut down
  if (threads_.size() == 0) return;
  // add the job to the queue (discard one job is the queue is full)
  const auto [discarded, discarded_id] = task_queue_.push(task);
  // update job count without blocking
  if (!discarded) {
    const auto success = job_count_.try_release();
    // the task_queue_ automatically discard old data when it is full, so
    // release must succeed
    if (!success)
      throw std::runtime_error("TaskExecutor::dispatch: job_count_ is full");
  }
  // tell any sleeping threads that there is a job ready
  // notify_one is ok because discarding a task won't make any other task
  // available to run
  cv_stop_or_queue_has_next_.notify_one();

  CLOG(DEBUG, "tactic.async_task")
      << "Added task of id: " << task->id << ", priority: " << task->priority
      << ", dep id: " << task->dep_id
      << " to queue; current job count (running or in queue): "
      << job_count_.get_value();
  callback_->taskAdded(task);
  if (discarded) {
    CLOG(DEBUG, "tactic.async_task")
        << "Discarded task of id: " << discarded_id << " from queue.";
    callback_->taskRemoved(discarded_id, /* completed */ false);
  }
}

void TaskExecutor::doWork(const size_t& thread_id) {
  el::Helpers::setThreadName("tactic.async_task_thread_" +
                             std::to_string(thread_id));
  while (true) {
    UniqueLock lock(mutex_);

    // while there are no jobs, sleep
    while (!stop_ && !task_queue_.hasNext())
      cv_stop_or_queue_has_next_.wait(lock);

    // if we need to stop, then stop
    if (stop_) {
      --thread_count_;
      cv_thread_finish_.notify_all();
      return;
    }

    // grab a job to do
    auto task = task_queue_.pop();

    lock.unlock();

    // do the task
    task->run(shared_from_this(), output_, graph_);

    // Decrement the task counter
    lock.lock();

    // remove the task from the dependency list
    task_queue_.updateDeps(task);

    job_count_.acquire();
    // finishing a task make any other task depending on it available to run, so
    // notify all
    cv_stop_or_queue_has_next_.notify_all();
    cv_job_maybe_empty_.notify_all();

    CLOG(DEBUG, "tactic.async_task")
        << "Removed task of id: " << task->id
        << ", priority: " << task->priority << ", dep id: " << task->dep_id
        << " from queue; current job count (running or in queue): "
        << job_count_.get_value();
    callback_->taskRemoved(task->id, /* completed */ true);
  }
}

}  // namespace tactic
}  // namespace vtr
