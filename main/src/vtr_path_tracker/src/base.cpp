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
 * \file base.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <thread>

#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_logging/logging.hpp>
#include <vtr_path_tracker/base.hpp>

namespace vtr {
namespace path_tracker {

Base::Base(const std::shared_ptr<Graph> &graph,
           const std::shared_ptr<rclcpp::Node> &node,
           const std::string &param_prefix)
    : node_(node), param_prefix_(param_prefix), graph_(graph) {
  CLOG(INFO, "path_tracker")
      << "Path tracker using namespace: " << param_prefix_;
  // clang-format off
  use_fixed_ctrl_rate_ = node->declare_parameter<bool>(param_prefix_ + ".use_fixed_ctrl_rate", false);
  control_period_ms_ = node->declare_parameter<double>(param_prefix_ + ".control_period_ms", 50.0);
  CLOG_IF(use_fixed_ctrl_rate_, DEBUG, "path_tracker") << "Using fixed control rate of " << control_period_ms_ << "ms.";
  // clang-format on
  /// \todo current do not support non-fixed control rate
  if (!use_fixed_ctrl_rate_) {
    std::string err{
        "Currently do not support non-fixed control rate. MPC has this "
        "assumption in several places, look for: "
        "'use_fixed_ctrl_rate'"};
    CLOG(ERROR, "path_tracker") << err;
    throw std::runtime_error{err};
  }
  publisher_ = node_->create_publisher<TwistMsg>("command", 1);
}

std::shared_ptr<Base> Create() {
  CLOG(ERROR, "path_tracker") << "Create method for base not implemented! "
                                 "Please use derived class instead.";
  return nullptr;
}

Base::~Base() {
  // Make sure the thread gets a stop notification
  stopAndJoin();
}

void Base::followPathAsync(const State &state, const Chain::Ptr &chain) {
  // We can't follow a new path if we're still following an old one.
  CLOG_IF(isRunning(), WARNING, "path_tracker")
      << "New path following objective set while still running.\n Discarding "
         "the old path and starting the new one.";
  stopAndJoin();

  // set the initial state and launch the control loop thread
  CLOG(INFO, "path_tracker") << "Start following a new path.";
  /// \todo yuchen: std::lock_guard<std::mutex> lock(state_mtx_);
  state_ = state;
  reset();
  chain_ = chain;

  control_loop_ = std::async(std::launch::async, &Base::controlLoop, this);
}

void Base::controlLoop() {
  el::Helpers::setThreadName("path_tracker.control_loop");

  // Do any pre-processing and load parameters
  loadConfigs();

  // the main control loop, which runs until STOP
  while (state_ != State::STOP) {
    CLOG(DEBUG, "path_tracker") << "=== Control Step Start ===";
    step_timer_.reset();

    // run the control loop if we're in the RUN state.
    // set command to stop in the PAUSE state
    if (state_ == State::RUN) {
      std::lock_guard<std::mutex> lock(state_mtx_);
      latest_command_ = controlStep();
    } else if (state_ == State::PAUSE) {
      latest_command_ = Command();  // sets command to zero.
    }

    // Sleep the remaining time in the control loop
    controlLoopSleep();

    // Only publish the command if we have received an update within 500 ms.
    if (state_ == State::RUN) publishCommand(latest_command_);

    CLOG(DEBUG, "path_tracker") << "===  Constrol Step End  ===";
  }
  finishControlLoop();
  CLOG(INFO, "path_tracker") << "Path tracker thread exiting";
}

void Base::controlLoopSleep() {
  // check how long it took the step to run
  double step_ms = step_timer_.elapsedMs();
  if (step_ms > control_period_ms_) {
    // uh oh, we're not keeping up to the requested rate
    CLOG(WARNING, "path_tracker") << "Path tracker step took " << step_ms
                                  << " ms > " << control_period_ms_ << " ms.";
  } else if (use_fixed_ctrl_rate_) {
    // sleep for remaining time in control loop
    const auto sleep_duration = common::timing::milliseconds(
        static_cast<long>(control_period_ms_ - step_ms));
    std::this_thread::sleep_for(sleep_duration);
  }
}

void Base::finishControlLoop() {
  CLOG(INFO, "path_tracker") << "Path tracker finished control loop";
  setState(State::STOP);
}

void Base::publishCommand(Command &command) {
  publisher_->publish(command.twist);
}

}  // namespace path_tracker
}  // namespace vtr
