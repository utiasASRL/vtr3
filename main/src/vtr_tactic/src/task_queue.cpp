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
 * \file async_task_queue.cpp
 * \brief TaskExecutor class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace tactic {

TaskExecutor::TaskExecutor(const Graph::Ptr& graph, const unsigned num_threads,
                           const size_t queue_length)
    : graph_(graph),
      num_threads_(num_threads),
      task_queue_(queue_length),
      job_count_(0, queue_length == size_t(-1)
                        ? std::numeric_limits<std::size_t>::max()
                        : queue_length + num_threads) {
  // start the threads!
  start();
}

void TaskExecutor::start() {
  LockGuard lock(mutex_);
  stop_ = false;
  thread_count_ = num_threads_;
  // make sure we have enough threads running
  while (threads_.size() < num_threads_)
    threads_.emplace_back(&TaskExecutor::doWork, this);
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

void TaskExecutor::dispatch(const Task::Ptr& task) {
  LockGuard lock(mutex_);
  // make sure we aren't stopped
  if (stop_) return;
  // The pool has been shut down
  if (threads_.size() == 0) return;
  // add the job to the queue (discard one job is the queue is full)
  auto discarded = task_queue_.push(task);
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
}

void TaskExecutor::doWork() {
  // Forever wait for work :)
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
    task->run(shared_from_this(), graph_);

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
  }
}

}  // namespace tactic
}  // namespace vtr
