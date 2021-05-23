
#include <vtr_path_tracker/base.h>

#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_logging/logging.hpp>

#include <thread>

namespace vtr {
namespace path_tracker {

Base::Base(const std::shared_ptr<Graph> &graph,
           const rclcpp::Clock& node_clock,
           double control_period_ms = 50 /* 20 hz */)
    : ros_clock(node_clock), graph_(graph), control_period_ms_(control_period_ms) {
}

std::shared_ptr<Base> Create() {
  LOG(ERROR) << "Create method for base not implemented! Please use derived class instead.";
  return nullptr;
}

Base::~Base() {
  // Make sure the thread gets a stop notification
  stopAndJoin();
}

void Base::followPathAsync(const State &state,
                           Chain &chain) {
  // We can't follow a new path if we're still following an old one.
  LOG(DEBUG) << "In followPathAsynch";
  LOG_IF(isRunning(), WARNING)
    << "New path following objective set while still running.\n Discarding the old path and starting the new one.";
  stopAndJoin();

  // set the initial state and launch the control loop thread
  state_ = state;
  reset();
  chain_ = std::make_shared<Chain>(chain);

  // Set the last safety monitor update to 5 years ago to ensure waiting for a new update.
  t_last_safety_monitor_update_ = Clock::now() - common::timing::years(5);

  control_loop_ = std::async(std::launch::async, &Base::controlLoop, this);
}

void Base::finishControlLoop() {
  LOG(INFO) << "Path tracker finished controlLoop" << std::endl;
  setState(State::STOP);
}

void Base::controlLoop() {

  // Do any pre-processing and load parameters
  loadConfigs();

  // the main control loop, which runs until STOP
  while (state_ != State::STOP) {
    step_timer_.reset();

    // run the control loop if we're in the RUN state.
    // set command to stop in the PAUSE state
    if (state_ == State::RUN) {
      std::lock_guard<std::mutex> lock(state_mtx_);
      latest_command_ = controlStep();
    } else if (state_ == State::PAUSE) {
      latest_command_ = Command(); // sets command to zero.
    }

    // Sleep the remaining time in the control loop
    controlLoopSleep();

    // Only publish the command if we have received an update within 500 ms.
    if (Clock::now() - t_last_safety_monitor_update_ < common::timing::duration_ms(500)) {
      if (state_ == State::RUN) {
        publishCommand(latest_command_);
      }
    } else {
      LOG_EVERY_N(10, WARNING) << "Path tracker has not received an update from the safety monitor in "
                               << std::chrono::duration_cast<std::chrono::milliseconds>(
                                   Clock::now() - t_last_safety_monitor_update_).count()
                               << " ms";
    }
  }
  finishControlLoop();
  LOG(INFO) << "Path tracker thread exiting";
}

void Base::controlLoopSleep() {
  // check how long it took the step to run
  double step_ms = step_timer_.elapsedMs();
  if (step_ms > control_period_ms_) {
    // uh oh, we're not keeping up to the requested rate
    LOG(ERROR) << "Path tracker step took " << step_ms
               << " ms > " << control_period_ms_ << " ms.";
  } else {
    // sleep the duration of the control period
    common::timing::milliseconds sleep_duration(35);
    // common::timing::milliseconds sleep_duration(static_cast<long>(control_period_ms_ - step_ms));
    std::this_thread::sleep_for(sleep_duration);
  }
}

void Base::publishCommand(Command &command) {
  (void) &command.twist; // suppress warning
  // publisher_->publish(command.twist);
}

} // path_tracker
} // vtr
