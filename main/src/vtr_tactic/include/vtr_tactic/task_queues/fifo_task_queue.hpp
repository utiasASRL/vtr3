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
 * \file fifo_task_queue.hpp
 * \brief FIFOTaskQueue class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <queue>

#include "vtr_tactic/task_queues/task_queue_base.hpp"

namespace vtr {
namespace tactic {

class FIFOTaskQueue : public TaskQueueBase {
 public:
  using Ptr = std::shared_ptr<FIFOTaskQueue>;

  using QueueType = std::queue<std::function<void()>>;

  FIFOTaskQueue(unsigned num_threads, size_t queue_length = 0)
      : TaskQueueBase(num_threads, queue_length) {}

  ~FIFOTaskQueue() override { join(); }

  size_t getQueueSize() const override { return jobs_.size(); }

  bool isQueueEmpty() const override { return jobs_.empty(); }

  void emptyQueue() override {
    while (!jobs_.empty()) jobs_.pop();
  }

  std::function<void()> getNextFromQueue() override {
    auto job = std::move(jobs_.front());
    jobs_.pop();
    return job;
  }

  /**
   * \brief adds a new job to the queue (like a call to std::async)
   * IMPORTANT: will block until there is room in the queue
   * \param[in] f a callable function that takes args
   * \param[in] args copyable arguments for the job, IMPORTANT: use
   * std::ref(arg) to pass by reference!
   */
  template <class Func, class... Args>
  std::future<typename std::result_of<Func(Args...)>::type> dispatch(
      Func f, Args &&... args);

  /**
   * \brief add a new job to the queue (like a call to std::async)
   * IMPORTANT: non-blocking; returns an invalid future on failure
   * \param[in] f a callable function that takes args
   * \param[in] args copyable arguments for the job, IMPORTANT: use
   * std::ref(arg) to pass by reference!
   */
  template <class Func, class... Args>
  std::future<typename std::result_of<Func(Args...)>::type> tryDispatch(
      Func f, Args &&... args);

 private:
  /** \brief Internal function for the actual dispatch */
  template <class Func, class... Args>
  std::future<typename std::result_of<Func(Args...)>::type> _dispatch(
      Func f, Args &&... args);

 private:
  /// pending jobs that await a worker
  QueueType jobs_;
};

template <class Func, class... Args>
std::future<typename std::result_of<Func(Args...)>::type>
FIFOTaskQueue::dispatch(Func f, Args &&... args) {
  job_count_.release();
  return _dispatch(f, std::forward<Args>(args)...);
}

template <class Func, class... Args>
std::future<typename std::result_of<Func(Args...)>::type>
FIFOTaskQueue::tryDispatch(Func f, Args &&... args) {
  // NOTE: job_count will not be released if there are no threads, due to early
  // termination of boolean expressions
  if (threads_.size() == 0 || !job_count_.try_release())
    // The pool has been shut down, or the queue was full
    return std::future<typename std::result_of<Func(Args...)>::type>();

  // Dispatch the job if we can release it
  return _dispatch(f, std::forward<Args>(args)...);
}

// the dispatch creates a packaged job with an empty signature.
template <class Func, class... Args>
std::future<typename std::result_of<Func(Args...)>::type>
FIFOTaskQueue::_dispatch(Func f, Args &&... args) {
  // this is the return type from the requested job
  typedef typename std::result_of<Func(Args...)>::type return_t;
  lockg_t lock(mutex_);
  // make sure we aren't stopped, why would you be adding jobs!
  if (stop_) throw std::runtime_error("pool is already stopped.");
  // make a promise, and get the return future
  auto promise_ptr = std::make_shared<std::promise<return_t>>();
  auto future_return = promise_ptr->get_future();
  // bind the job so that the promise will be set when the job is done
  std::function<void()> &&job =
      std::bind(TaskQueueBase::capture<Func, Args...>, promise_ptr, f,
                std::forward<Args>(args)...);
  // add the job to the queue
  jobs_.emplace(job);
  // tell any sleeping threads that there is a job ready
  sleeping_.notify_one();
  // return the future
  return future_return;
}

}  // namespace tactic
}  // namespace vtr
