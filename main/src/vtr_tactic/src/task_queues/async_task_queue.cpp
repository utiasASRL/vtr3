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
 * \brief AsyncTaskExecutor class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_tactic/task_queues/async_task_queue.hpp>

namespace vtr {
namespace tactic {

AsyncTaskExecutor::AsyncTaskExecutor(const Graph::Ptr& graph,
                                     unsigned num_threads, size_t queue_length)
    : graph_(graph),
      num_threads_(num_threads),
      job_count_(0, queue_length == size_t(-1) ? size_t(-1)
                                               : queue_length + num_threads) {
  // start the threads!
  start();
}

void AsyncTaskExecutor::start() {
  LockGuard lock(mutex_);
  stop_ = false;
  thread_count_ = num_threads_;
  // make sure we have enough threads running
  while (threads_.size() < num_threads_)
    threads_.emplace_back(&AsyncTaskExecutor::doWork, this);
}

void AsyncTaskExecutor::stop() {
  UniqueLock lock(mutex_);

  // wake up and tell the threads to stop
  stop_ = true;
  cv_queue_not_empty_.notify_all();

  // clear all the pending jobs, empty the job queue
  while (!jobs_.empty()) {
    job_count_.acquire();
    jobs_.pop();
  }
  cv_job_empty_.notify_all();

  // wait for the threads to stop, and destroy them
  while (thread_count_ > 0) cv_thread_finish_.wait(lock);
  while (!threads_.empty()) {
    threads_.front().join();
    threads_.pop_front();
  }
}

void AsyncTaskExecutor::dispatch(const BaseTask::Ptr& task) {
  UniqueLock lock(mutex_);
  // make sure we aren't stopped
  if (stop_) throw std::runtime_error("Async task executor already stopped!");
  // The poll has been shut down
  if (threads_.size() == 0) throw std::runtime_error("No threads available!");
  // The queue is full
  while (!job_count_.try_release()) {
    cv_queue_not_full_.wait(lock);
    if (stop_) return;
    if (threads_.size() == 0) return;
  }
  // add the job to the queue
  jobs_.emplace(task->priority, task);
  // tell any sleeping threads that there is a job ready
  cv_queue_not_empty_.notify_one();
}

void AsyncTaskExecutor::tryDispatch(const BaseTask::Ptr& task) {
  LockGuard lock(mutex_);
  // make sure we aren't stopped
  if (stop_) return;
  // The pool has been shut down
  if (threads_.size() == 0) return;
  // The queue is full
  if (!job_count_.try_release()) return;
  // add the job to the queue
  jobs_.emplace(task->priority, task);
  // tell any sleeping threads that there is a job ready
  cv_queue_not_empty_.notify_one();
}

void AsyncTaskExecutor::doWork() {
  // Forever wait for work :)
  while (true) {
    UniqueLock lock(mutex_);
    // while there are no jobs, sleep
    while (!stop_ && jobs_.empty()) cv_queue_not_empty_.wait(lock);
    // if we need to stop, then stop
    if (stop_) {
      --thread_count_;
      cv_thread_finish_.notify_one();
      return;
    }

    // grab a job to do
    auto job = jobs_.top().second;
    jobs_.pop();
    lock.unlock();

    // do the job
    job->run(shared_from_this(), graph_);

    // Decrement the job counter
    lock.lock();
    job_count_.acquire();
    cv_queue_not_full_.notify_one();
    cv_job_empty_.notify_all();
  }
}

}  // namespace tactic
}  // namespace vtr
