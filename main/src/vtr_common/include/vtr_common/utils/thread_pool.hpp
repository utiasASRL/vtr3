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
 * \file thread_pool.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <list>
#include <mutex>
#include <queue>
#include <stdexcept>

#include <vtr_common/utils/semaphore.hpp>
#include <vtr_common/utils/type_traits.hpp>

namespace vtr {
namespace common {

class thread_pool {
  typedef thread_pool my_t;
  typedef std::unique_lock<std::mutex> ulock_t;
  typedef std::lock_guard<std::mutex> lockg_t;
  typedef std::condition_variable cv_t;
  typedef bounded_joinable_semaphore sem_t;
  typedef semaphore_guard<sem_t> sem_guard_t;

 public:
  /// spawn the threads
  /// @param[in] num_threads the number of threads in the pool
  /// @param[in] queue_length the maximum number of queued jobs
  thread_pool(unsigned num_threads, size_t queue_length = 0);

  /// clean up the threads
  ~thread_pool() { join(); }

  /// start all of the threads
  void start();

  /// remove all pending jobs, and join all threads (blocking).
  void join();

  /// clear all the pending jobs, but let the threads keep running
  void clear();

  /// remove all pending jobs, and tell all threads to stop (non-blocking).
  void stop();

  /// waits for all jobs to finish
  void wait();

  /// returns the (approximate) idle state of the pool (non-blocking)
  inline bool isIdle_approx() const { return job_count_.get_value() == 0; }

  /// returns the idle state of the pool
  inline bool isIdle() {
    ulock_t lck(mutex_);
    return job_count_.get_value() == 0;
  }

  /// returns the (approximate) number of pending jobs (non-blocking)
  inline size_t pending_approx() const { return job_count_.get_value(); }

  /// returns the number of pending jobs
  inline size_t pending() {
    ulock_t lck(mutex_);
    return job_count_.get_value();
  }

  // TODO emergency_stop(), emergency_join() which sends signal to the threads

  /// check to see if the threads are running
  bool joinable() {
    ulock_t ulock(mutex_);
    return !threads_.empty();
  }

  /// add a new job to the queue (like a call to std::async)
  /// IMPORTANT: will block until there is room in the queue
  /// @param[in] f a callable function that takes args
  /// @param[in] args copyable arguments for the job, IMPORTANT:
  ///            use std::ref(arg) to pass by reference!
  template <class Func, class... Args>
  std::future<typename std::result_of<Func(Args...)>::type> dispatch(
      Func f, Args &&... args);

  /// add a new job to the queue (like a call to std::async)
  /// IMPORTANT: non-blocking; returns an invalid future on failure
  /// @param[in] f a callable function that takes args
  /// @param[in] args copyable arguments for the job, IMPORTANT:
  ///            use std::ref(arg) to pass by reference!
  template <class Func, class... Args>
  std::future<typename std::result_of<Func(Args...)>::type> try_dispatch(
      Func f, Args &&... args);

 private:
  /// Internal function for the actual dispatch
  template <class Func, class... Args>
  std::future<typename std::result_of<Func(Args...)>::type> _dispatch(
      Func f, Args &&... args);

  // This is what the thread actually runs
  void do_work();

  // This is a helper that lets bind capture the promise.
  // This one gets called for regular returns.
  template <class Func, class... Args>
  static
      typename std::enable_if<!returns_void<Func, Args...>::value, void>::type
      capture(std::shared_ptr<
                  std::promise<typename std::result_of<Func(Args...)>::type>>
                  promise_ptr,
              Func f, Args &&... args) {
    promise_ptr->set_value(f(std::forward<Args>(args)...));
  }
  // This is a helper that lets bind capture the promise.
  // This one gets called for void returns (special case).
  template <class Func, class... Args>
  static typename std::enable_if<returns_void<Func, Args...>::value, void>::type
  capture(std::shared_ptr<
              std::promise<typename std::result_of<Func(Args...)>::type>>
              promise_ptr,
          Func f, Args &&... args) {
    f(std::forward<Args>(args)...);
    promise_ptr->set_value();  // void promise needs an empty set_value!
  }

  /// stop flag for the threads to commit suicide
  bool stop_;
  /// number of threads allowed in the pool
  unsigned num_threads_;
  /// condition for all the unemployed threads to sleep on
  std::condition_variable sleeping_;
  /// the threads!!!
  std::list<std::thread> threads_;
  /// pending jobs that await a worker
  std::queue<std::function<void()>> jobs_;
  /// makes this stuff thread safe!
  std::mutex mutex_;
  /// counts the number of jobs (pending or in the queue)
  sem_t job_count_;
};

/// Blocking dispatch wrapper
template <class Func, class... Args>
std::future<typename std::result_of<Func(Args...)>::type> thread_pool::dispatch(
    Func f, Args &&... args) {
  job_count_.release();
  return _dispatch(f, std::forward<Args>(args)...);
}

/// Non-blocking dipatch wrapper
template <class Func, class... Args>
std::future<typename std::result_of<Func(Args...)>::type>
thread_pool::try_dispatch(Func f, Args &&... args) {
  // NOTE: job_count will not be released if there are no threads, due to early
  // termination of boolean expressions
  if (threads_.size() == 0 || !job_count_.try_release()) {
    // The pool has been shut down, or the queue was full
    return std::future<typename std::result_of<Func(Args...)>::type>();
  } else {
    // Dispatch the job if we can release it
    return _dispatch(f, std::forward<Args>(args)...);
  }
}

// the dispatch creates a packaged job with an empty signature.
template <class Func, class... Args>
std::future<typename std::result_of<Func(Args...)>::type>
thread_pool::_dispatch(Func f, Args &&... args) {
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
      std::bind(my_t::capture<Func, Args...>, promise_ptr, f,
                std::forward<Args>(args)...);
  // add the job to the queue
  jobs_.emplace(job);
  // tell any sleeping threads that there is a job ready
  sleeping_.notify_one();
  // return the future
  return future_return;
}

}  // namespace common
}  // namespace vtr
