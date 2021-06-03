#include <vtr_safety_monitor/inputs/deadman_monitor.hpp>

namespace vtr {
namespace safety_monitor {

const int DeadmanMonitorInput::axis_array[] = {7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6};
const int DeadmanMonitorInput::value_array[] = {1, 0, 1, 0, -1, 0, -1, 0, 1, 0, -1, 0, 1, 0, -1, 0};

// Create the monitor
DeadmanMonitorInput::DeadmanMonitorInput(const std::shared_ptr<rclcpp::Node> node) :
    SafetyMonitorInput(node),
    followingMode_(MANUAL),
    codeState_(0) {

  signal_monitors.clear();

  // Initialize DEADMAN monitor
  signal_monitors.emplace_back(SignalMonitor(node));
  signal_monitors.back().initializeType(DISCRETE_MONITOR);
  double msg_timeout = 1;
  signal_monitors.back().initialize("Deadman Monitor", msg_timeout);
  deadman_monitor = (int) signal_monitors.size() - 1;

  // Initialize Message Subscriptions
  gamepad_subscriber_ = node_->create_subscription<JoyMsg>("/xbox_joy",
                                                           10,
                                                           std::bind(&DeadmanMonitorInput::gamepadCallback,
                                                                     this, std::placeholders::_1));

  deadman_button_index_ = (int) node_->declare_parameter<int>("deadman_button_index", 1);

  // Initialize the gamepad callback
  timeofLastCodeSequence_ = rclcpp::Clock().now();
}

/********************
// Define Message Callback Functions
********************/

void DeadmanMonitorInput::gamepadCallback(const JoyMsg::SharedPtr msg) {

  signal_monitors[deadman_monitor].registerMsgButNoStatusChange();

  bool pressed = msg->buttons[deadman_button_index_] != 0;

  if (signal_monitors[deadman_monitor].getDesiredAction() == PAUSE) {

    if (followingMode_ == AUTO || pressed) {
      signal_monitors[deadman_monitor].setMonitorDesiredAction(CONTINUE);
    } else {
      signal_monitors[deadman_monitor].setMonitorDesiredAction(PAUSE);
    }

    // check the cheat code state
    checkCodeState(msg);
    // if the cheat code has been activated, then put the path tracker into auto mode.
    if (codeState_ == CHEAT_CODE_ACTIVATED) {
      LOG(INFO) << "CHEAT CODE ACTIVATED, SWITCHING TO AUTO MODE";
      followingMode_ = AUTO;
    }
  } else if (signal_monitors[deadman_monitor].getDesiredAction() == CONTINUE) {

    if (followingMode_ == AUTO) {

      // Check if we need to disable AUTO mode
      for (int button : msg->buttons) {
        rclcpp::Duration diff = rclcpp::Clock().now() - timeofLastCodeSequence_;

        if (button == 1 && diff.seconds() > 1.0) {
          LOG(INFO) << "CHEAT CODE DEACTIVATED, SWITCHING TO MANUAL MODE\n";
          followingMode_ = MANUAL;
          codeState_ = 0;
        }
      }

      // Now that we've checked the cheat code and maybe changed followingMode_...
      if (followingMode_ == MANUAL && !pressed) {
        signal_monitors[deadman_monitor].setMonitorDesiredAction(PAUSE);
      } else {
        signal_monitors[deadman_monitor].setMonitorDesiredAction(CONTINUE);
      }
    } else if (!pressed) {
      signal_monitors[deadman_monitor].setMonitorDesiredAction(PAUSE);
    } else {
      signal_monitors[deadman_monitor].registerMsgButNoStatusChange();
    }
  } else {
    signal_monitors[deadman_monitor].setMonitorDesiredAction(PAUSE);
  }
}

void DeadmanMonitorInput::checkCodeState(const JoyMsg::SharedPtr &msg) {
  rclcpp::Duration timeDiff = rclcpp::Clock().now() - timeofLastCodeSequence_;
  LOG(DEBUG) << "Time since last button: " << timeDiff.seconds() << " \n";

  if (timeDiff.seconds() > 0.75) {
    codeState_ = 0;
    LOG(DEBUG) << "Too long since button press, codeState_ reset.";
  }
  if (codeState_ < 16) {
    if (msg->axes[axis_array[codeState_]] == value_array[codeState_]) {
      codeState_++;
      timeofLastCodeSequence_ = rclcpp::Clock().now();
    }
  } else if (codeState_ >= 16 && msg->buttons[deadman_button_index_] != 0) {
    codeState_ = CHEAT_CODE_ACTIVATED;
  }
}

} // safety_monitor
} // vtr
