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
 * \file task_queue_base.hpp
 * \brief TaskQueueBase class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <list>
#include <mutex>
#include <stdexcept>

#include "vtr_common/utils/semaphore.hpp"
#include "vtr_common/utils/type_traits.hpp"

namespace vtr {
namespace tactic {

class TaskQueueBase {
 protected:
  using ulock_t = std::unique_lock<std::mutex>;
  using lockg_t = std::lock_guard<std::mutex>;
  using cv_t = std::condition_variable;
  using sem_t = common::bounded_joinable_semaphore;
  using sem_guard_t = common::semaphore_guard<sem_t>;

 public:
  using Ptr = std::shared_ptr<TaskQueueBase>;

  /**
   * \brief spawn the threads
   * \param[in] num_threads the number of threads in the pool
   * \param[in] queue_length the maximum number of queued jobs
   */
  TaskQueueBase(unsigned num_threads, size_t queue_length = 0);

  /** \brief \note subclass should call join() to clean up the threads */
  virtual ~TaskQueueBase() = default;

  /** \brief starts all threads */
  void start();

  /** \brief removes all pending jobs, and join all threads (blocking). */
  void join();

  /** \brief clears all the pending jobs, but let the threads keep running */
  void clear();

  /** \brief removes all pending jobs and stops all threads (non-blocking). */
  void stop();

  /** \brief waits for all jobs to finish */
  void wait() { job_count_.wait(0); }

  /** \brief returns the (approximate) idle state of the pool (non-blocking) */
  inline bool isIdleApprox() const { return job_count_.get_value() == 0; }

  /** \brief returns the idle state of the pool */
  inline bool isIdle() {
    ulock_t lck(mutex_);
    return job_count_.get_value() == 0;
  }

  /** \brief returns the (approximate) number of pending jobs (non-blocking) */
  inline size_t pendingApprox() const { return job_count_.get_value(); }

  /** \brief returns the number of pending jobs */
  inline size_t pending() {
    ulock_t lck(mutex_);
    return job_count_.get_value();
  }

  /** \brief checks to see if the threads are running */
  bool joinable() {
    ulock_t ulock(mutex_);
    return !threads_.empty();
  }

 protected:
  // This is a helper that lets bind capture the promise.
  // This one gets called for regular returns.
  template <class Func, class... Args>
  static typename std::enable_if<!common::returns_void<Func, Args...>::value,
                                 void>::type
  capture(std::shared_ptr<
              std::promise<typename std::result_of<Func(Args...)>::type>>
              promise_ptr,
          Func f, Args &&... args) {
    promise_ptr->set_value(f(std::forward<Args>(args)...));
  }
  // This is a helper that lets bind capture the promise.
  // This one gets called for void returns (special case).
  template <class Func, class... Args>
  static typename std::enable_if<common::returns_void<Func, Args...>::value,
                                 void>::type
  capture(std::shared_ptr<
              std::promise<typename std::result_of<Func(Args...)>::type>>
              promise_ptr,
          Func f, Args &&... args) {
    f(std::forward<Args>(args)...);
    promise_ptr->set_value();  // void promise needs an empty set_value!
  }

 private:
  /** \brief This is what the thread actually runs */
  void doWork();

  /** \brief Internal job queue interface */
  virtual size_t getQueueSize() const = 0;
  virtual bool isQueueEmpty() const = 0;
  virtual void emptyQueue() = 0;
  virtual std::function<void()> getNextFromQueue() = 0;

 protected:
  /// number of threads allowed in the pool
  const unsigned num_threads_;
  /// counts the number of jobs (pending or in the queue)
  sem_t job_count_;
  /// stop flag for the threads to commit suicide
  bool stop_ = true;
  /// condition for all the unemployed threads to sleep on
  std::condition_variable sleeping_;
  /// the threads!!!
  std::list<std::thread> threads_;
  /// makes this stuff thread safe!
  std::mutex mutex_;
};

}  // namespace tactic
}  // namespace vtr
