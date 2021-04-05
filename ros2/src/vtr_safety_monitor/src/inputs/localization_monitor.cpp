#include <vtr_safety_monitor/inputs/localization_monitor.hpp>
#include <vtr_logging/logging.hpp>

namespace vtr {
namespace safety_monitor {

LocalizationMonitorInput::LocalizationMonitorInput(const std::shared_ptr<rclcpp::Node> node) :
    SafetyMonitorInput(node) {
  signal_monitors.clear();
  double msg_timeout = 0; // Don't check for timeout of msg

  // Initialize localization uncertainty monitor.
  signal_monitors.emplace_back(SignalMonitor(node));
  signal_monitors.back().initializeType(DISCRETE_MONITOR);
  signal_monitors.back().initialize("Localization Uncertainty", msg_timeout);

  // Initialize message subscriptions
  status_subscriber_ = node_->create_subscription<RobotStatus>("robot",
                                                               10,
                                                               std::bind(&LocalizationMonitorInput::statusCallback,
                                                                         this,
                                                                         std::placeholders::_1));

  uncertainty_limits(0) = node_->declare_parameter<double>("translation_x_limit", 0.5);    //todo: prefixes?
  uncertainty_limits(1) = node_->declare_parameter<double>("translation_y_limit", 0.5);
  uncertainty_limits(2) = node_->declare_parameter<double>("rotation_z_limit", 15.0); // degrees

  // convert rotation limits to radians
  uncertainty_limits(2) = uncertainty_limits(2) / 57.29577;

  LOG(INFO) << " Localization Uncertainty Limits: \n" << uncertainty_limits;
}

void LocalizationMonitorInput::statusCallback(const RobotStatus::SharedPtr status) {
  if (status->state == "::Repeat::MetricLocalize") {        // todo: should there also be a soft limit? (slow down)
    if (status->cov_leaf_trunk.size() != 3){
      LOG(WARNING) << "Expected cov_leaf_trunk array to be of size 3 but was actually size: " << status->cov_leaf_trunk.size();
      signal_monitors[0].setMonitorDesiredAction(PAUSE);
      return;
    }
    auto stddev = Eigen::Vector3d(status->cov_leaf_trunk.at(0), status->cov_leaf_trunk.at(1), status->cov_leaf_trunk.at(2));

    for (int idx = 0; idx < uncertainty_limits.rows(); ++idx) {
      if (stddev(idx) >= uncertainty_limits(idx)) {
        LOG(ERROR) << "std. dev in dimension " << idx << " is off";
        LOG(ERROR) << " std. dev: " << stddev(idx) << " threshold: " << uncertainty_limits(idx);
        signal_monitors[0].setMonitorDesiredAction(PAUSE);
        return;
      }
    }
    signal_monitors[0].setMonitorDesiredAction(CONTINUE);
  }
}

} // safety_monitor
} // vtr
