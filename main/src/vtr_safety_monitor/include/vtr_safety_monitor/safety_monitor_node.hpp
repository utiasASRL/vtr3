#pragma once

#include <rclcpp/rclcpp.hpp>

#include <vtr_safety_monitor/base/safety_monitor_input_base.hpp>

#include <vtr_messages/msg/desired_action_in.hpp>
#if 0     // not ported
#include <vtr_messages/msg/monitor_debug.hpp>
#endif
#include <vtr_messages/msg/robot_status.hpp>

#include <vtr_safety_monitor/inputs/localization_monitor.hpp>
#include <vtr_safety_monitor/inputs/deadman_monitor.hpp>

#if 0
#include <std_msgs/String.h>

// Definition of the safety monitor input base


//Definition of the safety monitor messages
#include <asrl__messages/DesiredActionIn.h>
#include <asrl__safety_monitor/MonitorDebug.h>

// Definition of the graph navigation status message
//#include <asrl__mapping__graph_navigation/Status.h>
#include <asrl__messages/RobotStatus.h>
//#include <asrl__mapping__graph_navigation/KeyFrameNavigationStatus.h>


// The various safety monitor inputs:
//#include <asrl/safety_monitor/vtr_monitor.hpp>
#include <safety_monitor/deadman_monitor.hpp>
#include <safety_monitor/path_tracker_monitor.hpp>
#include <safety_monitor/LocalizationMonitorInput.hpp>
#include <safety_monitor/incline_monitor.hpp>
#include <safety_monitor/grizzly_diagnostics_msg_monitor.hpp>
#include <safety_monitor/estop_monitor.hpp>
// #include <asrl/safety_monitor/terrain_assessment_monitor.hpp>
#endif

namespace vtr {
namespace safety_monitor {

 class SafetyMonitorNode : public rclcpp::Node {
 public :
  SafetyMonitorNode();

  void initializeMonitors();

  const static std::string desired_action_str_array[];

 private :

  // List of Monitors
  std::vector<std::string> list_of_monitors;
  std::vector<vtr::safety_monitor::SafetyMonitorInput *> monitor_vector;

  // Objects for periodic status updates
  rclcpp::TimerBase::SharedPtr safety_status_timer_;
  double safety_status_period_;
  void getSafetyStatusCallback();
  rclcpp::Publisher<vtr_messages::msg::DesiredActionIn>::SharedPtr safety_status_publisher_;
#if 0
  rclcpp::Publisher<vtr_messages::msg::MonitorDebug>::SharedPtr safety_debug_publisher_;
#endif

  /** \brief This function converts strings provided in the launch file into safety monitor objects **/
  SafetyMonitorInput *SafetyMonitorFactory(std::string monitor_input_str);

  // Parameters
  double initialTimeOutSeconds_;
  double time_out_seconds_;
  double absolute_max_speed_;

};

} // safety_monitor
} // vtr

