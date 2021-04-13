#pragma once

#include <rclcpp/rclcpp.hpp>

const int DISCRETE_MONITOR = 1;

// Define possible desired actions
// Integers are used to determine priority, higher num = higher priority
const int CONTINUE = 1;
const int SLOW = 2;
const int UNKNOWN = 3;
const int PAUSE = 4;
const int PAUSE_AND_RELOCALIZE = 5;
const int HALT = 6;
const int HALT_AND_REPLAN = 7;
const int HALT_AND_BLOCK = 8;

namespace vtr {
namespace safety_monitor {

class SignalMonitor {
 public :
  /** \brief Constructor */
  SignalMonitor(std::shared_ptr<rclcpp::Node> node);

  /** \brief  */
  void initializeType(int /*type*/);
  /** \brief  */
  void initialize(std::string name, double max_time_in);
  /** \brief  */
  int monitor_type;

  // Interactions for discrete monitor
  /** \brief  */
  void setMonitorDesiredAction(int desired_action);
  /** \brief  */
  void registerMsgButNoStatusChange();

  // Interactions for all signal monitor types
  /** \brief  */
  void setStatusUnknown();
  /** \brief  */
  double getMaxAllowedSpeed();
  /** \brief  */
  void setMaxAllowedSpeed(double &max_allowed_speed_in);
  /** \brief  */
  rclcpp::Time getLastUpdateTime();
  /** \brief  */
  void setMsgTimeout(double new_msg_timeout);
  /** \brief  */
  int getDesiredAction();
  /** \brief  */
  double max_time_between_updates;
  /** \brief  */
  std::string monitor_name;

 private :

  /** \brief ROS-handle for communication */
  const std::shared_ptr<rclcpp::Node> node_;

  // Monitor variables
  /** \brief  */
  bool signal_monitor_status_unknown;
  /** \brief  */
  int desired_action;
  /** \brief  */
  rclcpp::Time last_update;
  /** \brief  */
  double max_allowed_speed_;

}; // class SignalMonitor

} // namespace safety_monitor
} // namespace vtr
