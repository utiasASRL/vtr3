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
  lockg_t lock(mutex_);
  stop_ = false;
  // make sure we have enough threads running
  while (threads_.size() < num_threads_)
    threads_.emplace_back(&AsyncTaskExecutor::doWork, this);
}

void AsyncTaskExecutor::clear() {
  lockg_t lock(mutex_);
  // reduces the job counter to zero
  for (size_t i = 0; i < jobs_.size(); ++i) job_count_.acquire();
  // empty the job queue
  while (!jobs_.empty()) jobs_.pop();
}

void AsyncTaskExecutor::stop() {
  // clear all the pending jobs
  clear();
  // tell the threads to stop
  lockg_t lock(mutex_);
  stop_ = true;
  // tell the threads to wake up
  sleeping_.notify_all();
}

void AsyncTaskExecutor::join() {
  // tell the threads to stop
  stop();
  // wait for the threads to stop, and destroy them
  while (!threads_.empty()) {
    if (threads_.front().joinable()) threads_.front().join();
    threads_.pop_front();
  }
}

void AsyncTaskExecutor::dispatch(const BaseTask::Ptr& task) {
  job_count_.release();
  _dispatch(task);
}

void AsyncTaskExecutor::tryDispatch(const BaseTask::Ptr& task) {
  // NOTE: job_count will not be released if there are no threads, due to early
  // termination of boolean expressions
  if (threads_.size() == 0 || !job_count_.try_release())
    // The pool has been shut down, or the queue was full
    return;

  // Dispatch the job if we can release it
  _dispatch(task);
}

void AsyncTaskExecutor::_dispatch(const BaseTask::Ptr& task) {
  lockg_t lock(mutex_);
  // make sure we aren't stopped, why would you be adding jobs!
  if (stop_) throw std::runtime_error("pool is already stopped.");
  // add the job to the queue
  jobs_.emplace(task->priority, task);
  // tell any sleeping threads that there is a job ready
  sleeping_.notify_one();
}

void AsyncTaskExecutor::doWork() {
  // Forever wait for work :)
  while (true) {
    ulock_t lock(mutex_);
    // while there are no jobs, sleep
    while (!stop_ && jobs_.empty()) sleeping_.wait(lock);
    // if we need to stop, then stop
    if (stop_) return;
    // grab a job to do
    auto job = jobs_.top().second;
    jobs_.pop();
    lock.unlock();

    // do the job
    job->run(shared_from_this(), graph_);

    // Decrement the job counter
    job_count_.acquire();
  }
}

}  // namespace tactic
}  // namespace vtr
