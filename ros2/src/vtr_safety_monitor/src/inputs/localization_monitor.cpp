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
  uncertainty_limits(2) = node_->declare_parameter<double>("translation_z_limit", 10.0);
  uncertainty_limits(3) = node_->declare_parameter<double>("rotation_x_limit", 10.0);
  uncertainty_limits(4) = node_->declare_parameter<double>("rotation_y_limit", 10.0);
  uncertainty_limits(5) = node_->declare_parameter<double>("rotation_z_limit", 0.5);

  // convert rotation limits to radians
  for (int idx = 3; idx < 6; ++idx) {
    uncertainty_limits(idx) = uncertainty_limits(idx) / 57.29577;
  }
  LOG(INFO) << " Localization Uncertainty Limits: \n" << uncertainty_limits;
}

void LocalizationMonitorInput::statusCallback(const RobotStatus::SharedPtr status) {
  if (status->state == "::Repeat::Follow") {        // todo: should there also be a soft limit? (slow down)
    auto cov = Eigen::Matrix<double, 6, 6>(status->cov_leaf_trunk.data());
    auto var = cov.diagonal();
    for (int idx = 0; idx < uncertainty_limits.rows(); ++idx) {
      if (sqrt(var(idx)) >= uncertainty_limits(idx)) {
        LOG(ERROR) << "std. dev in dimension " << idx << " is off";
        LOG(ERROR) << " std. dev: " << sqrt(var(idx)) << " threshold: " << uncertainty_limits(idx);
        signal_monitors[0].setMonitorDesiredAction(PAUSE);
        return;
      }
    }
    signal_monitors[0].setMonitorDesiredAction(CONTINUE);
  }
}

} // safety_monitor
} // vtr
