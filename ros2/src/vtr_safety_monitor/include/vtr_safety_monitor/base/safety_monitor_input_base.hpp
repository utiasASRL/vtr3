#include <rclcpp/rclcpp.hpp>

// These are defined so that we don't have to worry when things are changed
const int MAX_ACTION = 8;
const int MIN_ACTION = 0;

// Definition of a generic signal monitor
#include <vtr_safety_monitor/base/signal_monitor.hpp>

namespace vtr {
namespace safety_monitor {

class SafetyMonitorInput {
 public :
  /** \brief  */
  SafetyMonitorInput(std::shared_ptr<rclcpp::Node> node);

  /** \brief  */
  virtual int getDesiredAction() = 0;

  /** \brief  */
  bool updateSafetyMonitorAction(int &desired_action,
                                 double &speed_limit,
                                 std::vector<std::string> &limiting_signal_monitor_names,
                                 std::vector<int> &limiting_signal_monitor_actions);

 protected :
  /** \brief ROS-handle for communication */
  const std::shared_ptr<rclcpp::Node> node_;

  /** \brief  */
  std::vector<SignalMonitor> signal_monitors;

}; // class safetyMonitorInput

} // namespace safety_monitor
} // namespace vtr
