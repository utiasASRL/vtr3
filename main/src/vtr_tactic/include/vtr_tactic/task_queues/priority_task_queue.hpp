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
 * \file priority_task_queue.hpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <queue>

#include "vtr_tactic/task_queues/task_queue_base.hpp"

namespace vtr {
namespace tactic {

/**
 * \brief Allows user to specify a priority of the job range [0, 9], 0 is the
 * highest.
 */
class PriorityTaskQueue : public TaskQueueBase {
 public:
  using Ptr = std::shared_ptr<PriorityTaskQueue>;

  using ValueType = std::pair<unsigned int, std::function<void()>>;

  /// A customized comparator
  struct Compare {
    constexpr bool operator()(const ValueType &lhs,
                              const ValueType &rhs) const {
      return lhs.first > rhs.first;
    }
  };
  using QueueType =
      std::priority_queue<ValueType, std::vector<ValueType>, Compare>;

  PriorityTaskQueue(unsigned num_threads, size_t queue_length = 0)
      : TaskQueueBase(num_threads, queue_length) {}

  ~PriorityTaskQueue() override { join(); }

  size_t getQueueSize() const override { return jobs_.size(); }

  bool isQueueEmpty() const override { return jobs_.empty(); }

  void emptyQueue() override {
    while (!jobs_.empty()) jobs_.pop();
  }

  std::function<void()> getNextFromQueue() override {
    auto job = std::move(jobs_.top().second);
    jobs_.pop();
    return job;
  }

  /**
   * \brief adds a new job to the queue (like a call to std::async)
   * IMPORTANT: will block until there is room in the queue
   * \param[in] priority priority of the task, will be clipped to [0, 9]
   * \param[in] f a callable function that takes args
   * \param[in] args copyable arguments for the job, IMPORTANT: use
   * std::ref(arg) to pass by reference!
   */
  template <class Func, class... Args>
  std::future<typename std::result_of<Func(Args...)>::type> dispatch(
      const unsigned int &priority, Func f, Args &&... args);

  /**
   * \brief add a new job to the queue (like a call to std::async)
   * IMPORTANT: non-blocking; returns an invalid future on failure
   * \param[in] priority priority of the task, will be clipped to [0, 9]
   * \param[in] f a callable function that takes args
   * \param[in] args copyable arguments for the job, IMPORTANT: use
   * std::ref(arg) to pass by reference!
   */
  template <class Func, class... Args>
  std::future<typename std::result_of<Func(Args...)>::type> tryDispatch(
      const unsigned int &priority, Func f, Args &&... args);

 private:
  /** \brief Internal function for the actual dispatch */
  template <class Func, class... Args>
  std::future<typename std::result_of<Func(Args...)>::type> _dispatch(
      const unsigned int &priority, Func f, Args &&... args);

 private:
  /// pending jobs that await a worker
  QueueType jobs_;
};

template <class Func, class... Args>
std::future<typename std::result_of<Func(Args...)>::type>
PriorityTaskQueue::dispatch(const unsigned int &priority, Func f,
                            Args &&... args) {
  job_count_.release();
  return _dispatch(priority, f, std::forward<Args>(args)...);
}

template <class Func, class... Args>
std::future<typename std::result_of<Func(Args...)>::type>
PriorityTaskQueue::tryDispatch(const unsigned int &priority, Func f,
                               Args &&... args) {
  // NOTE: job_count will not be released if there are no threads, due to early
  // termination of boolean expressions
  if (threads_.size() == 0 || !job_count_.try_release())
    // The pool has been shut down, or the queue was full
    return std::future<typename std::result_of<Func(Args...)>::type>();

  // Dispatch the job if we can release it
  return _dispatch(priority, f, std::forward<Args>(args)...);
}

// the dispatch creates a packaged job with an empty signature.
template <class Func, class... Args>
std::future<typename std::result_of<Func(Args...)>::type>
PriorityTaskQueue::_dispatch(const unsigned int &priority, Func f,
                             Args &&... args) {
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
  jobs_.emplace(std::min(priority, (unsigned int)9), job);
  // tell any sleeping threads that there is a job ready
  sleeping_.notify_one();
  // return the future
  return future_return;
}

}  // namespace tactic
}  // namespace vtr
