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
 * \file base_path_planner.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_path_planning/base_path_planner.hpp"

namespace vtr {
namespace path_planning {

BasePathPlanner::BasePathPlanner(const unsigned int& control_period,
                                 const Callback::Ptr& callback)
    : control_period_(control_period), callback_(callback) {
  //
  thread_count_ = 1;
  process_thread_ = std::thread(&BasePathPlanner::process, this);
}

BasePathPlanner::~BasePathPlanner() { stop(); }

void BasePathPlanner::initializeRoute() {
  UniqueLock lock(mutex_);
  //
  // TODO(Yuchen): implement this function
  //
}

void BasePathPlanner::setRunning(const bool running) {
  UniqueLock lock(mutex_);
  running_ = running;
  cv_terminate_or_state_changed_.notify_all();
  callback_->stateChanged(running_);
}

void BasePathPlanner::stop() {
  UniqueLock lock(mutex_);
  //
  terminate_ = true;
  cv_terminate_or_state_changed_.notify_all();
  cv_thread_finish_.wait(lock, [this] { return thread_count_ == 0; });
  if (process_thread_.joinable()) process_thread_.join();
}

void BasePathPlanner::process() {
  el::Helpers::setThreadName("path_planning");
  CLOG(INFO, "path_planning") << "Starting the path planning thread.";
  while (true) {
    UniqueLock lock(mutex_);
    cv_terminate_or_state_changed_.wait(
        lock, [this] { return terminate_ || running_; });

    if (terminate_) {
      --thread_count_;
      CLOG(INFO, "path_planning") << "Stopping the path planning thread.";
      cv_thread_finish_.notify_all();
      return;
    }

    /// \note command computation should not require the lock, and this is
    /// required to give other threads a chance to acquire the lock
    lock.unlock();

    //
    const auto wait_until_time = std::chrono::steady_clock::now() +
                                 std::chrono::milliseconds(control_period_);
    const auto command = computeCommand();
    callback_->commandReceived(command);
    std::this_thread::sleep_until(wait_until_time);
  }
}

}  // namespace path_planning
}  // namespace vtr