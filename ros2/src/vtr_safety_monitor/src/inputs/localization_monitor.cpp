#include <asrl/messages/lgmath_conversions.hpp>
#include <asrl/common/logging.hpp>
namespace asrl {
namespace safetyMonitor {

/////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief constructor
LocalizationMonitorInput::LocalizationMonitorInput(ros::NodeHandle nh) :
safetyMonitorInput(nh) {
  signal_monitors.clear();
  double msg_timeout = 0; // Don't check for timeout of msg

  /********************
  // Initialize Localizaton uncertainty monitor.
  ********************/
  signal_monitors.emplace_back(signalMonitor(nh));
  signal_monitors.back().initialize_type(DISCRETE_MONITOR);
  signal_monitors.back().initialize("Localization Uncertainty", msg_timeout);

  /********************
  // Initialize Message Subscriptions
  ********************/
  statusSubscriber_ = nodeHandle_.subscribe<asrl__messages::TrackingStatus>
                        ("in/tracking_status", 100, &LocalizationMonitorInput::statusCallback, this);

  
    asrl::rosutil::param<double>(nodeHandle_,"translation_x_limit",uncertainty_limits(0),0.5);
    asrl::rosutil::param<double>(nodeHandle_,"translation_y_limit",uncertainty_limits(1),0.5);
    asrl::rosutil::param<double>(nodeHandle_,"translation_z_limit",uncertainty_limits(2),10.0);
    asrl::rosutil::param<double>(nodeHandle_,"rotation_x_limit",uncertainty_limits(3),10.0);
    asrl::rosutil::param<double>(nodeHandle_,"rotation_y_limit",uncertainty_limits(4),10.0);
    asrl::rosutil::param<double>(nodeHandle_,"rotation_z_limit",uncertainty_limits(5),0.5);

    // convert rotation limits to radians
    for(int idx = 3; idx < 6; ++idx) {
      uncertainty_limits(idx) = uncertainty_limits(idx) / 57.29577;
    }
    LOG(INFO) << " Localization Uncertainty Limits: \n" << uncertainty_limits;
}


void LocalizationMonitorInput::statusCallback(const asrl__messages::TrackingStatusConstPtr & status) {
  // 1. Make sure the uncertainty is lower than the given threshold.
  // 2. Make sure the uncertianty is currently increasing.
  // 3. if so, then stop the robot.

  if (status->state == "::Repeat::Follow") {
    if (status->T_leaf_trunk.cov_set) {
      auto cov = Eigen::Matrix<double, 6, 6>(status->T_leaf_trunk.cov.data());
      auto var = cov.diagonal();
      for (int idx = 0; idx < uncertainty_limits.rows(); ++idx) {
        if (sqrt(var(idx)) >= uncertainty_limits(idx)) {
          LOG(ERROR) << "std. dev in dimension " << idx << " is off";
          LOG(ERROR) << " std. dev: " << sqrt(var(idx)) << " threshold: " << uncertainty_limits(idx);
          signal_monitors[0].set_monitor_desired_action(PAUSE);
          return;
        }
      }
      signal_monitors[0].set_monitor_desired_action(CONTINUE);
    }
  } else {
    signal_monitors[0].set_monitor_desired_action(PAUSE);
  }
}

}} 
