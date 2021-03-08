// asrl
#include <asrl/terrain_assessment/ros_conversions.hpp>

namespace asrl {
namespace safetyMonitor {

// Define button presses.
static const uint16_t NOTHING = 0;
static const uint16_t A = 1;
static const uint16_t B = 1 << 1;
static const uint16_t X = 1 << 2;
static const uint16_t Y = 1 << 3;
static const uint16_t LB = 1 << 4;
static const uint16_t RB = 1 << 5;
static const uint16_t SELECT = 1 << 6;
static const uint16_t START = 1 << 7;
static const uint16_t XBOX = 1 << 8;
static const uint16_t L3 = 1 << 9;
static const uint16_t R3 = 1 << 10;
static const uint16_t D_LEFT = 1 << 11;
static const uint16_t D_RIGHT = 1 << 12;
static const uint16_t D_UP = 1 << 13;
static const uint16_t D_DOWN = 1 << 14;

// Create the monitor
terrain_assessment_monitor_input::terrain_assessment_monitor_input(ros::NodeHandle node_handle)
    : safetyMonitorInput(node_handle) {
  // Initialize signal monitors.
  signal_monitors.clear();
  signalMonitor empty_signal_monitor(node_handle);

  // Initialize subscribers
  gamepad_subscriber_ = nodeHandle_.subscribe("in/joy", 1, &terrain_assessment_monitor_input::GamepadCallback, this);
  safety_subscriber_   = nodeHandle_.subscribe("in/safety", 1, &terrain_assessment_monitor_input::SafetyCallback, this);

  // Initialize terrain assessment monitor
  signal_monitors.push_back(empty_signal_monitor);
  terrain_assessment_monitor_ = signal_monitors.size() - 1;
  double msg_timeout = 1;
  signal_monitors[terrain_assessment_monitor_].initialize_type(DISCRETE_MONITOR);
  signal_monitors[terrain_assessment_monitor_].initialize("Terrain Monitor", msg_timeout);
  
  // Initialize slow speed
  slow_speed_ = 0.5;
  n_stop_to_manual_ = 10;

  // Initialize button indices
  deadman_button_index_ = 1;
  auto_button_index_ = 2;
  obstacle_button_index_ = 3;

  // Initialize button presses.
  auto_pressed_previously_ = false;
  deadman_pressed_previously_ = false;
  
  // Initialize state.
  safety_.state = asrl::terrain_assessment::Safety::State::STOP;
  drive_mode_ = MANUAL_DRIVE;
  n_stop_ = 0;
  
  // Set up cheat code.
  cheat_code_ = {D_UP, NOTHING,
                 D_UP, NOTHING,
                 D_DOWN, NOTHING,
                 D_DOWN, NOTHING,
                 D_LEFT, NOTHING,
                 D_RIGHT, NOTHING,
                 D_LEFT, NOTHING,
                 D_RIGHT, NOTHING,
                 B, NOTHING,
                 A, NOTHING};
  code_idx_ = 0;
  code_timeout_ = ros::Duration(1.0);
  last_code_time_ = ros::Time(0.0);

  LOG(INFO) << "Terrain monitor initialized.";
}

void terrain_assessment_monitor_input::GamepadCallback(const sensor_msgs::JoyPtr& joy_msg) {
  // Register that the message was received...? So it doesn't time out...?
  signal_monitors[terrain_assessment_monitor_].register_msg_but_no_status_change();

  // Check the state of the cheat code.
  auto cheat_code_enabled = CheckCheatCode(joy_msg);
  if (cheat_code_enabled) {
    LOG(INFO) << "Cheat code enabled.";
  }

  // Check if any important buttons are pressed.
  bool auto_pressed = joy_msg->buttons[auto_button_index_] != 0;
  bool deadman_pressed = joy_msg->buttons[deadman_button_index_] != 0;
  
  // Check if anything else is pressed.
  bool any_pressed = false;
  for (unsigned int idx = 0; idx < joy_msg->buttons.size(); ++idx) {
    if (joy_msg->buttons[idx] != 0 && 
        idx != deadman_button_index_ &&
        idx != auto_button_index_ &&
        idx != obstacle_button_index_) {
      any_pressed = true; 
    }
  }

  // Reset n_stop if we're not stopped
  if (safety_.state != asrl::terrain_assessment::Safety::State::STOP) {
    n_stop_ = 0;
  }

  // Set the drive mode.
  if (drive_mode_ == MANUAL_DRIVE) {
    if (auto_pressed == true &&
        auto_pressed_previously_ == false) {
      // Set drive mode to automatic.
      drive_mode_ = AUTOMATIC_DRIVE;
    }
  }
  else if (drive_mode_ == AUTOMATIC_DRIVE) {
    if (safety_.state == asrl::terrain_assessment::Safety::State::STOP) {
      // Commenting out the next line means this will never exit from AUTOMATIC mode unless you
      // manually ask it to. It will still stop for unsafe terrain assessments, but will continue
      // when the path clears.
      // n_stop_++;
    }
    if (n_stop_ > n_stop_to_manual_ || any_pressed ||
        (auto_pressed == true && auto_pressed_previously_ == false) ||
        (deadman_pressed == true && deadman_pressed_previously_ == false)) {
      // change drive mode to manual
      drive_mode_ = MANUAL_DRIVE;
    }
  }

  // Set drive command according to drive mode
  if (deadman_pressed || cheat_code_enabled) {
    signal_monitors[terrain_assessment_monitor_].set_monitor_desired_action(CONTINUE);
  }
  else if (drive_mode_ == AUTOMATIC_DRIVE) {
    if (safety_.state == asrl::terrain_assessment::Safety::State::SAFE) {
      signal_monitors[terrain_assessment_monitor_].set_monitor_desired_action(CONTINUE);
    }
    else if (safety_.state == asrl::terrain_assessment::Safety::State::SLOW) {
      signal_monitors[terrain_assessment_monitor_].set_monitor_desired_action(SLOW);
      signal_monitors[terrain_assessment_monitor_].set_max_allowed_speed(slow_speed_);
    }
    else if (safety_.state == asrl::terrain_assessment::Safety::State::STOP) {
      signal_monitors[terrain_assessment_monitor_].set_monitor_desired_action(PAUSE);
    }
  }
  else {
    signal_monitors[terrain_assessment_monitor_].set_monitor_desired_action(PAUSE);
  }

  // Update button presses.
  auto_pressed_previously_ = auto_pressed;
  deadman_pressed_previously_ = deadman_pressed;
}

void terrain_assessment_monitor_input::SafetyCallback(const asrl__terrain_assessment::SafetyPtr& safety_msg) {
  if (safety_msg->state == asrl__terrain_assessment::Safety::SAFE) {
    safety_.state = asrl::terrain_assessment::Safety::State::SAFE;
  }
  else if (safety_msg->state == asrl__terrain_assessment::Safety::SLOW) {
    safety_.state = asrl::terrain_assessment::Safety::State::SLOW;
  }
  else if (safety_msg->state == asrl__terrain_assessment::Safety::STOP) {
    safety_.state = asrl::terrain_assessment::Safety::State::STOP;
  }
}

bool terrain_assessment_monitor_input::CheckCheatCode(const sensor_msgs::JoyPtr& joy_msg) {
  // Reset the code index if the time between this button press and the last successful one is too long
  if (joy_msg->header.stamp - last_code_time_ > code_timeout_ &&
      code_idx_ < cheat_code_.size()) {
    LOG(INFO) << "Cheat code sequence timeout.";
    code_idx_ = 0;
    last_code_time_ = joy_msg->header.stamp;
  }
  else {
    // Check which buttons are pressed.
    uint16_t buttons = 0;
    for (unsigned int idx = 0; idx < joy_msg->buttons.size(); ++idx) {
      if (joy_msg->buttons.at(idx)) {
        buttons += 1 << idx;
      }
    }

    // Check for dpad buttons
    if (joy_msg->axes.at(6) == 1) {
      buttons += 1 << 11;
    }
    if (joy_msg->axes.at(6) == -1) {
      buttons += 1 << 12;
    }
    if (joy_msg->axes.at(7) == 1) {
      buttons += 1 << 13;
    }
    if (joy_msg->axes.at(7) == -1) {
      buttons += 1 << 14;
    }

    // Get previous cheat code button
    uint16_t prev_button = 0;
    if (code_idx_ > 0 && (code_idx_ - 1) < cheat_code_.size()) {
      prev_button = cheat_code_.at(code_idx_ - 1);
    }

    // Get current cheat code button
    uint16_t current_button = 0;
    if (code_idx_ < cheat_code_.size()) {
      current_button = cheat_code_.at(code_idx_);
    }

    // Reset the code index if the button pressed is not equal to the current,
    // previous, or nothing.
    if (buttons != prev_button &&
        buttons != current_button &&
        buttons != 0) {
      LOG(INFO) << "Cheat code reset.";
      code_idx_ = 0;
    }
    else if (buttons == current_button && code_idx_ < cheat_code_.size()) {
      if (code_idx_ == 0) {
        LOG(INFO) << "Cheat code sequence initialized...";
      }
      code_idx_++;
      last_code_time_ = joy_msg->header.stamp;
    }
  }
  
  // Cheat code enabled if length is greater than 0 and we're at the end.
  return (cheat_code_.size() > 0 && code_idx_ == cheat_code_.size());
}

}
}