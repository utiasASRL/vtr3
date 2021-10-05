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
 * \file thread_pool.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_tactic/task_queues/task_queue_base.hpp>

namespace vtr {
namespace tactic {

TaskQueueBase::TaskQueueBase(unsigned num_threads, size_t queue_length)
    : num_threads_(num_threads),
      job_count_(0, queue_length == size_t(-1) ? size_t(-1)
                                               : queue_length + num_threads) {
  // start the threads!
  start();
}

void TaskQueueBase::start() {
  lockg_t lock(mutex_);
  stop_ = false;
  // make sure we have enough threads running
  while (threads_.size() < num_threads_)
    threads_.emplace_back(&TaskQueueBase::doWork, this);
}

void TaskQueueBase::clear() {
  lockg_t lock(mutex_);
  // reduces the job counter to zero
  for (size_t i = 0; i < getQueueSize(); ++i) job_count_.acquire();
  // empty the job queue
  emptyQueue();
}

void TaskQueueBase::stop() {
  // clear all the pending jobs
  clear();
  // tell the threads to stop
  lockg_t lock(mutex_);
  stop_ = true;
  // tell the threads to wake up
  sleeping_.notify_all();
}

void TaskQueueBase::join() {
  // tell the threads to stop
  stop();
  // wait for the threads to stop, and destroy them
  while (!threads_.empty()) {
    if (threads_.front().joinable()) threads_.front().join();
    threads_.pop_front();
  }
}

void TaskQueueBase::doWork() {
  // Forever wait for work :)
  while (true) {
    ulock_t lock(mutex_);
    // while there are no jobs, sleep
    while (!stop_ && isQueueEmpty()) sleeping_.wait(lock);
    // if we need to stop, then stop
    if (stop_) return;
    // grab a job to do
    auto job = getNextFromQueue();
    lock.unlock();

    // do the job
    job();

    // Decrement the job counter
    job_count_.acquire();
  }
}

}  // namespace tactic
}  // namespace vtr
