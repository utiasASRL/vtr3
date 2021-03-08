#pragma once

// asrl
#include <asrl/terrain_assessment/Safety.hpp>
#include <asrl__terrain_assessment/Safety.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

enum DriveMode {
  MANUAL_DRIVE,
  AUTOMATIC_DRIVE
};

namespace asrl {
namespace safetyMonitor {

class terrain_assessment_monitor_input : public safetyMonitorInput {
 public:
  /**
   * Constructor
   */
  terrain_assessment_monitor_input(ros::NodeHandle node_handle);
  
  /**
   * Gamepad callback
   */
  void GamepadCallback(const sensor_msgs::JoyPtr &joy_msg);
  
  /**
   * Safety callback
   */
  void SafetyCallback(const asrl__terrain_assessment::SafetyPtr &safety_msg);

 private:
  bool CheckCheatCode(const sensor_msgs::JoyPtr& joy_msg);

  // void publish_ambience(const int &beacon,
  //                       const int &beep,
  //                       const int &headlight,
  //                       const int &taillight,
  //                       const double &duration);

  // ros
  ros::Subscriber gamepad_subscriber_;
  ros::Subscriber safety_subscriber_;

  // state
  int terrain_assessment_monitor_;
  DriveMode drive_mode_;
  asrl::terrain_assessment::Safety safety_;
  int n_stop_;

  // other
  double slow_speed_;
  int n_stop_to_manual_;

  // button presses
  int auto_button_index_;
  int deadman_button_index_;
  int obstacle_button_index_;

  bool auto_pressed_;
  bool auto_pressed_previously_;
  bool deadman_pressed_;
  bool deadman_pressed_previously_;

  // cheat code
  std::vector<uint16_t> cheat_code_;
  int code_idx_;
  ros::Duration code_timeout_;
  ros::Time last_code_time_;
};
}
}

#include <asrl/safety_monitor/implementation/terrain_assessment_monitor.cpp>