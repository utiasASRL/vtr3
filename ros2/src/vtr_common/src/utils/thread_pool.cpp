#include <vtr_common/utils/thread_pool.hpp>

namespace vtr {
namespace common {

thread_pool::thread_pool(unsigned num_threads, size_t queue_length)
    : num_threads_(num_threads),
      job_count_(0, queue_length == size_t(-1) ? size_t(-1)
                                               : queue_length + num_threads) {
  // start the threads!
  start();
}

void thread_pool::start() {
  lockg_t lock(mutex_);
  stop_ = false;
  // make sure we have enough threads running
  while (threads_.size() < num_threads_)
    threads_.emplace_back(&my_t::do_work, this);
}

void thread_pool::clear() {
  lockg_t lock(mutex_);
  // clear all the pending jobs
  for (size_t i = 0; i < jobs_.size(); ++i) {
    job_count_.acquire();
  }
  while (!jobs_.empty()) {
    jobs_.pop();
  }
}

void thread_pool::stop() {
  // clear all the pending jobs
  clear();
  // tell the threads to stop
  lockg_t lock(mutex_);
  stop_ = true;
  // tell the threads to wake up
  sleeping_.notify_all();
}

void thread_pool::join() {
  // tell the threads to stop
  stop();
  // wait for the threads to stop, and destroy them
  while (!threads_.empty()) {
    if (threads_.front().joinable()) threads_.front().join();
    threads_.pop_front();
  }
}

void thread_pool::wait() { job_count_.wait(0); }

void thread_pool::do_work() {
  // Forever wait for work :)
  while (true) {
    ulock_t lock(mutex_);
    // while there are no jobs, sleep
    while (!stop_ && jobs_.empty()) sleeping_.wait(lock);
    // if we need to stop, then stop
    if (stop_) return;
    // grab a job to do
    auto job = std::move(jobs_.front());
    jobs_.pop();
    lock.unlock();
    // do the job
    job();

    // Decrement the job counter
    job_count_.acquire();
  }
}

}  // namespace common
}  // namespace vtr
