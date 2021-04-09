#include <vtr_safety_monitor/safety_monitor_node.hpp>

#include <vtr_logging/logging_init.hpp>

namespace vtr {
namespace safety_monitor {

// Unknown is interpreted as PAUSE for the path tracker
const std::string SafetyMonitorNode::desired_action_str_array[] = {"NOT_READY", "CONTINUE",
                                                                   "MAINTAIN_SLOW", "SLOW_DOWN",
                                                                   "PAUSE", "PAUSE",
                                                                   "PAUSE_AND_RELOCALIZE",
                                                                   "HALT_AND_REPLAN",
                                                                   "HALT_AND_BLOCK"};

SafetyMonitorNode::SafetyMonitorNode() :
    Node("safety_monitor") {
  safety_status_period_ = 0.2;
  safety_status_publisher_ = this->create_publisher<vtr_messages::msg::DesiredActionIn>("safety_monitor_node/out/desired_action", 10);
#if 0
  safety_debug_publisher_ = this->create_publisher<vtr_messages::msg::MonitorDebug>("out/debug", 10);
#endif

  absolute_max_speed_ = this->declare_parameter<double>("max_allowed_speed", 5.0);    //todo: prefix?
  time_out_seconds_ = this->declare_parameter<double>("relocation_timeout", 10.0);    //todo: prefix?

  initializeMonitors();
}

void SafetyMonitorNode::initializeMonitors() {

  list_of_monitors = this->declare_parameter<std::vector<std::string>>("list_of_monitors",
                                                                       std::vector<std::string>{});    //todo: prefix?

  LOG(INFO) << "Found " << list_of_monitors.size() << " monitors.";

  for (auto &monitor : list_of_monitors) {
    monitor_vector.push_back(SafetyMonitorFactory(monitor));
  }

  safety_status_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000*safety_status_period_)), std::bind(&SafetyMonitorNode::getSafetyStatusCallback, this));
}

SafetyMonitorInput *SafetyMonitorNode::SafetyMonitorFactory(std::string monitor_input_str) {

  const char *string = monitor_input_str.c_str();

  LOG(INFO) << "Initializing " << monitor_input_str.c_str();

  if (std::strcmp(string, "localization_monitor") == 0) {
    return new LocalizationMonitorInput(static_cast<std::shared_ptr<Node>>(this));
  } else if (std::strcmp(string, "deadman_monitor") == 0) {
    return new DeadmanMonitorInput(static_cast<std::shared_ptr<Node>>(this));
  }
#if 0
  else if (std::strcmp(string, "incline_monitor") == 0) {
    return new incline_monitor_input(nh);
  }
#endif
  else {
    LOG(ERROR) << "SafetyMonitorFactory: Safety monitor launch script is requesting a monitor that doesn't exist.";
    throw std::invalid_argument("Invalid monitor name");
  }
}

void SafetyMonitorNode::getSafetyStatusCallback() {

  // Initialize values
  int desired_action = CONTINUE;
  double speed_limit = absolute_max_speed_;
  std::vector<std::string> limiting_signal_monitor_names;
  limiting_signal_monitor_names.clear();
  std::vector<int> limiting_signal_monitor_actions;
  limiting_signal_monitor_actions.clear();

  // Go through all monitors and signals to determine highest priority behavior and speed limit
  for (auto &i : monitor_vector) {
    i->updateSafetyMonitorAction(desired_action, speed_limit, limiting_signal_monitor_names,
                                 limiting_signal_monitor_actions);
  }

  LOG(DEBUG) << "Current safety layer speed limit: " << speed_limit;
  LOG(DEBUG) << "Current desired action: " << desired_action;

  // Construct the safety monitor msg
  vtr_messages::msg::DesiredActionIn msg;
#if 0
  asrl__safety_monitor::MonitorDebug debug_msg;
#endif

  bool valid_action = true;
  if (desired_action == CONTINUE) {
    // The path tracker will ramp up to the prescheduled speed
    msg.desired_action = "CONTINUE";

  } else if (desired_action == SLOW) {
    // The path tracker will slow to the speed specified in msg.speed_limit
    msg.desired_action = "SLOW";

  } else if (desired_action == PAUSE) {
    // The path tracker will pause, but not discard the current path
    msg.desired_action = "PAUSE";

  } else if (desired_action == PAUSE_AND_RELOCALIZE) {
    // The path tracker will pause, but not discard the current path
    msg.desired_action = "PAUSE_AND_RELOCALIZE";

  } else if (desired_action == HALT) {
    // The path tracker will pause AND discard the current path
    msg.desired_action = "HALT";

  } else if (desired_action == HALT_AND_REPLAN) {
    // The path tracker will pause AND discard the current path
    msg.desired_action = "HALT_AND_REPLAN";

  } else if (desired_action == HALT_AND_BLOCK) {
    // The path tracker will pause AND discard the current path
    msg.desired_action = "HALT_AND_BLOCK";

  } else if (desired_action == UNKNOWN) {
    // One or more monitors is unknown, the path tracker will pause the current path
    msg.desired_action = "PAUSE";

  } else {
    valid_action = false;
    LOG(ERROR) << "Safety monitor node: Desired path tracker action not in range.";
    msg.desired_action = "PAUSE";
    msg.speed_limit = 0.05;

#if 0
    debug_msg.limiting_signal_monitor_names.push_back("Safety_Monitor_Issue");
    debug_msg.limiting_signal_monitor_actions.push_back(PAUSE);
#endif

  }

  if (valid_action) {
    msg.speed_limit = speed_limit;
#if 0
    debug_msg.limiting_signal_monitor_names = limiting_signal_monitor_names;
    debug_msg.limiting_signal_monitor_actions = limiting_signal_monitor_actions;
#endif
  }

  safety_status_publisher_->publish(msg);
#if 0
  safety_debug_publisher_->publish(debug_msg);
#endif
}

} // safety_monitor
} // vtr

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  vtr::logging::configureLogging();

  LOG(INFO) << "Safety monitor booting up.";
  rclcpp::spin(std::make_shared<vtr::safety_monitor::SafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
