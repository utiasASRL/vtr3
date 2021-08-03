
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
  CLOG(ERROR, "path_tracker") << "Create method for base not implemented! Please use derived class instead.";
  return nullptr;
}

Base::~Base() {
  // Make sure the thread gets a stop notification
  stopAndJoin();
}

void Base::followPathAsync(const State &state,
                           Chain &chain) {
  // We can't follow a new path if we're still following an old one.
  CLOG(DEBUG, "path_tracker") << "In followPathAsynch";
  CLOG_IF(isRunning(), WARNING, "path_tracker")
    << "New path following objective set while still running.\n Discarding the old path and starting the new one.";
  stopAndJoin();

  // set the initial state and launch the control loop thread
  /// \todo yuchen: std::lock_guard<std::mutex> lock(state_mtx_);
  state_ = state;
  reset();
  chain_ = std::make_shared<Chain>(chain);

  control_loop_ = std::async(std::launch::async, &Base::controlLoop, this);
}

void Base::finishControlLoop() {
  CLOG(INFO, "path_tracker") << "Path tracker finished controlLoop" << std::endl;
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
      latest_command_ = Command();  // sets command to zero.
    }

    // Sleep the remaining time in the control loop
    controlLoopSleep();

    // Only publish the command if we have received an update within 500 ms.
    if (state_ == State::RUN) publishCommand(latest_command_);
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
  } else {
    // sleep the duration of the control period
    /// \todo yuchen: hardcoded number 35 not making sense
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
