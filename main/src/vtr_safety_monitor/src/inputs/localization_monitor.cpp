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

  // Initialize dead-reckoning monitor.
  signal_monitors.emplace_back(SignalMonitor(node));
  signal_monitors.back().initializeType(DISCRETE_MONITOR);
  signal_monitors.back().initialize("Dead-Reckoning Distance", msg_timeout);
  signal_monitors.back().setMonitorDesiredAction(CONTINUE); // initialize to CONTINUE

  // Initialize message subscriptions
  status_subscriber_ = node_->create_subscription<RobotStatus>("robot",
                                                               10,
                                                               std::bind(&LocalizationMonitorInput::statusCallback,
                                                                         this,
                                                                         std::placeholders::_1));

  vo_subscriber_ = node_->create_subscription<std_msgs::msg::Int32>("frames_on_vo",
                                                                    10,
                                                                    std::bind(&LocalizationMonitorInput::voCallback,
                                                                              this,
                                                                              std::placeholders::_1));

  // set to high value as we don't necessarily trust our that our uncertainty is consistent
  uncertainty_limits(0) = node_->declare_parameter<double>("translation_x_limit", 1.5);
  uncertainty_limits(1) = node_->declare_parameter<double>("translation_y_limit", 1.5);
  uncertainty_limits(2) = node_->declare_parameter<double>("rotation_z_limit", 15.0); // degrees

  max_frames_on_vo_slow_ = node_->declare_parameter<int>("max_frames_on_vo_slow", 3);
  max_frames_on_vo_stop_ = node_->declare_parameter<int>("max_frames_on_vo_stop", 30);

  // convert rotation limits to radians
  uncertainty_limits(2) = uncertainty_limits(2) / 57.29577;

  LOG(INFO) << " Localization Uncertainty Limits: \n" << uncertainty_limits;

  LOG(INFO) << " Dead reckoning frames to slow: " << max_frames_on_vo_slow_;
  LOG(INFO) << " Dead reckoning frames to stop: " << 40;
}

void LocalizationMonitorInput::statusCallback(const RobotStatus::SharedPtr status) {
  if (status->state == "::Repeat::Follow") {
    if (status->cov_leaf_trunk.size() != 3) {
      LOG(WARNING) << "Expected cov_leaf_trunk array to be of size 3 but was size: " << status->cov_leaf_trunk.size();
      signal_monitors[0].setMonitorDesiredAction(PAUSE);
      return;
    }
    auto stddev =
        Eigen::Vector3d(status->cov_leaf_trunk.at(0), status->cov_leaf_trunk.at(1), status->cov_leaf_trunk.at(2));

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

void LocalizationMonitorInput::voCallback(const std_msgs::msg::Int32::SharedPtr frames) {
  // should only receive this message from MetricLocalization pipeline so shouldn't need to check state
  int vo_frames = frames->data;

  if (vo_frames < max_frames_on_vo_slow_) {
    signal_monitors[1].setMonitorDesiredAction(CONTINUE);
  } else if (vo_frames < max_frames_on_vo_stop_) {
    signal_monitors[1].setMonitorDesiredAction(SLOW);
    LOG(WARNING) << vo_frames << " consecutive frames relied on VO. Setting action to SLOW.";
  } else {
    signal_monitors[1].setMonitorDesiredAction(PAUSE);
    LOG(WARNING) << vo_frames << " consecutive frames relied on VO. Setting action to PAUSE.";
  }
}

} // safety_monitor
} // vtr
