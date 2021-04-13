#if 0
#include <vtr_safety_monitor/inputs/path_tracker_monitor.hpp>

//#include <asrl/diagnostics/Diagnostics.hpp>


namespace vtr {
namespace safety_monitor {

// Create the monitor
PathTrackerMonitorInput::PathTrackerMonitorInput(const std::shared_ptr<rclcpp::Node> node) :
    SafetyMonitorInput(node) {

  signal_monitors.clear();
  signalMonitor empty_signal_monitor(nh);

  asrl::rosutil::param<double>(nodeHandle_, "min_allowed_speed", min_limit_speed_, 0.3);
  asrl::rosutil::param<double>(nodeHandle_, "max_allowed_speed", absolute_max_speed_, 5.0);

  /********************
  // Initialize Message Subscriptions
  ********************/
  statusSubscriber_ = nodeHandle_.subscribe<asrl__control__path_tracker::StatusOut>("in/path_tracker_status",
                                                                                    100,
                                                                                    &PathTrackerMonitorInput::statusCallback,
                                                                                    this);

  /********************
  // Initialize Path Tracker Error monitor
  ********************/
  signal_monitors.push_back(empty_signal_monitor);
  pt_error_monitor = signal_monitors.size() - 1;
  double msg_timeout = 0; // Don't check for timeout of msg
  signal_monitors[pt_error_monitor].initializeType(DISCRETE_MONITOR);
  signal_monitors[pt_error_monitor].initialize("Current Tracking Error", msg_timeout);
  asrl::rosutil::param<double>(nodeHandle_, "acceptable_lateral_error", acceptable_lateral_error_, 0.2);
  asrl::rosutil::param<double>(nodeHandle_, "acceptable_heading_error", acceptable_heading_error_, 0.2);
  asrl::rosutil::param<double>(nodeHandle_, "tracking_error_filter_time", tracking_error_filter_time_, 0.4);
  filtered_lat_err_ = 0;
  filtered_head_err_ = 0;

  last_pathTracker_status_msg_time_ = rclcpp::Clock().now();

  /********************
  // Initialize Predicted Path Tracker Error monitor
  ********************/
  signal_monitors.push_back(empty_signal_monitor);
  pred_pt_error_monitor = signal_monitors.size() - 1;
  msg_timeout = 0; // Don't check for timeout of msg
  signal_monitors[pred_pt_error_monitor].initializeType(DISCRETE_MONITOR);
  signal_monitors[pred_pt_error_monitor].initialize("Uncertain Pred Tracking Error", msg_timeout);
  asrl::rosutil::param<double>(nodeHandle_, "acceptable_pred_lateral_error", acceptable_pred_lateral_error_, 0.2);
  asrl::rosutil::param<double>(nodeHandle_, "acceptable_pred_heading_error", acceptable_pred_heading_error_, 0.2);
  asrl::rosutil::param<double>(nodeHandle_,
                               "pred_tracking_error_speed_reduction_pct",
                               pred_tracking_error_speed_reduction_pct_,
                               0.99);
  pred_tracking_error_speed_reduction_pct_ = std::min(std::max(pred_tracking_error_speed_reduction_pct_, 0.1), 1.0);

  filtered_pred_lat_err_ = 0;
  filtered_pred_head_err_ = 0;

  /********************
  // Initialize Localization Delay monitor
  ********************/
// TODO: This should be replaced with some status from navigation
  asrl::rosutil::param<double>(nodeHandle_,
                               "max_dist_between_vo_or_loc_updates",
                               max_dist_between_vo_or_loc_updates_,
                               0.3);
  asrl::rosutil::param<double>(nodeHandle_,
                               "max_time_between_vo_or_loc_updates",
                               max_time_between_vo_or_loc_updates_,
                               1.0);

  signal_monitors.push_back(empty_signal_monitor);
  loc_delay_monitor = signal_monitors.size() - 1;

  // TODO: We cannot have a timeout here, because the path tracker doesn't publish continuously while WAITING
  msg_timeout = max_time_between_vo_or_loc_updates_;
  signal_monitors[loc_delay_monitor].initializeType(DISCRETE_MONITOR);
  signal_monitors[loc_delay_monitor].initialize("Localization Timeout", msg_timeout);
  max_speed_filter_const_ = 0.5;
}

/********************
// Define Message Callback Functions
********************/

double PathTrackerMonitorInput::get_speed_increase(double spd_in, double pct_change) const {
  return pct_change * spd_in + (1 - pct_change) * absolute_max_speed_;
}

double PathTrackerMonitorInput::get_speed_decrease(double spd_in, double pct_change) const {
  return pct_change * spd_in + (1 - pct_change) * min_limit_speed_;
}

void PathTrackerMonitorInput::statusCallback(const PTStatus::SharedPtr status) {

  // Compute delta t
  rclcpp::Duration t_lastMsgRos = rclcpp::Clock().now() - last_pathTracker_status_msg_time_;
  double t_lastMsg = std::min(t_lastMsgRos.seconds(), tracking_error_filter_time_);
  last_pathTracker_status_msg_time_ = rclcpp::Clock().now();

  if (status->state == std::string("FOLLOWING") || status->state == std::string("PAUSED")) {

    // Compute filtered tracking error values
    filtered_lat_err_ =
        ((tracking_error_filter_time_ - t_lastMsg) * filtered_lat_err_ + t_lastMsg * status->lateral_error)
            / tracking_error_filter_time_;
    filtered_head_err_ =
        ((tracking_error_filter_time_ - t_lastMsg) * filtered_head_err_ + t_lastMsg * status->heading_error)
            / tracking_error_filter_time_;
    filtered_pred_lat_err_ = ((tracking_error_filter_time_ - t_lastMsg) * filtered_pred_lat_err_
        + t_lastMsg * status->predicted_worst_lateral_error) / tracking_error_filter_time_;
    filtered_pred_head_err_ = ((tracking_error_filter_time_ - t_lastMsg) * filtered_pred_head_err_
        + t_lastMsg * status->predicted_worst_heading_error) / tracking_error_filter_time_;

    // Assess tracking error and assign speed limits if necessary
    bool acceptable_lateral_error = std::abs(filtered_lat_err_) < acceptable_lateral_error_;
    bool acceptable_heading_error = std::abs(filtered_head_err_) < acceptable_heading_error_ * 3.14159 / 180;
    if (acceptable_lateral_error == true && acceptable_heading_error == true) {
      signal_monitors[pt_error_monitor].setMonitorDesiredAction(CONTINUE);
      double new_speed_limit =
          get_speed_increase(signal_monitors[pt_error_monitor].getMaxAllowedSpeed(), max_speed_filter_const_);
      signal_monitors[pt_error_monitor].setMaxAllowedSpeed(new_speed_limit);

    } else {
      signal_monitors[pt_error_monitor].setMonitorDesiredAction(SLOW);
      double new_speed_limit =
          get_speed_decrease(fabs(status->commanded_linear_speed), pred_tracking_error_speed_reduction_pct_);
      signal_monitors[pt_error_monitor].setMaxAllowedSpeed(new_speed_limit);
      LOG(INFO) << "Requesting slow for tracking errors: " << new_speed_limit << " m/s";

    }

    // Assess predicted tracking errors and assign speed limits if necessary
    bool flg_acceptable_pred_lateral_error = std::abs(filtered_pred_lat_err_) < acceptable_pred_lateral_error_;
    bool flg_acceptable_pred_heading_error =
        std::abs(filtered_pred_head_err_) < acceptable_pred_heading_error_ * 3.14159 / 180;
    if (flg_acceptable_pred_lateral_error == true && flg_acceptable_pred_heading_error == true) {
      signal_monitors[pred_pt_error_monitor].setMonitorDesiredAction(CONTINUE);
      double new_speed_limit =
          get_speed_increase(signal_monitors[pred_pt_error_monitor].getMaxAllowedSpeed(), max_speed_filter_const_);
      signal_monitors[pred_pt_error_monitor].setMaxAllowedSpeed(new_speed_limit);
      //LOG(INFO) << ""Pred lat ok with: %f / %f", filtered_pred_lat_err_, status->predicted_worst_lateral_error);

    } else {
      signal_monitors[pred_pt_error_monitor].setMonitorDesiredAction(SLOW);
      double new_speed_limit =
          get_speed_decrease(fabs(status->commanded_linear_speed), pred_tracking_error_speed_reduction_pct_);
      signal_monitors[pred_pt_error_monitor].setMaxAllowedSpeed(new_speed_limit);
      LOG(INFO) << "Requesting slow for pred tracking errors: " << new_speed_limit << " m/s";
    }

    // Update localization delay monitor
    float est_distance_travelled = std::abs(status->commanded_linear_speed) * status->localization_computation_time;
    bool loc_delay_negative = status->localization_computation_time < 0;
    // The delay distance is not enough on it's own, since it assumes the entire duration has been
    // at the last commanded speed, so we also need the hard time delay, below
    bool acceptable_loc_delay_dist = est_distance_travelled < max_dist_between_vo_or_loc_updates_;
    bool acceptable_loc_delay_time = status->localization_computation_time < max_time_between_vo_or_loc_updates_;

    if (!acceptable_loc_delay_time) {
      signal_monitors[loc_delay_monitor].setMonitorDesiredAction(PAUSE);
      double new_speed_limit = min_limit_speed_;
      signal_monitors[loc_delay_monitor].setMaxAllowedSpeed(new_speed_limit);
      LOG_EVERY_N(10, INFO) << "Localization delay is " << status->localization_computation_time << " s. Stopping the vehicle.";

    } else if (!acceptable_loc_delay_dist) {
      double new_speed_limit = max_dist_between_vo_or_loc_updates_ / status->localization_computation_time;
      new_speed_limit = std::min(absolute_max_speed_, std::max(min_limit_speed_, new_speed_limit));
      signal_monitors[loc_delay_monitor].setMonitorDesiredAction(SLOW);
      signal_monitors[loc_delay_monitor].setMaxAllowedSpeed(new_speed_limit);
      LOG(INFO) << "Localization delay is " << status->localization_computation_time << " s at "<< std::abs(status->commanded_linear_speed) << " m/s, slowing the vehicle to " << new_speed_limit << " m/s";

    } else if (loc_delay_negative) {
      signal_monitors[loc_delay_monitor].setMonitorDesiredAction(SLOW);
      double new_speed_limit = min_limit_speed_;
      signal_monitors[loc_delay_monitor].setMaxAllowedSpeed(min_limit_speed_);
      LOG(INFO) << "Localization delay is negative! Limping along at the min speed: " << new_speed_limit << " m/s";

    } else {
      signal_monitors[loc_delay_monitor].setMonitorDesiredAction(CONTINUE);
      double new_speed_limit =
          get_speed_increase(signal_monitors[loc_delay_monitor].getMaxAllowedSpeed(), max_speed_filter_const_);
      signal_monitors[loc_delay_monitor].setMaxAllowedSpeed(new_speed_limit);
    }

  } else {
    signal_monitors[pt_error_monitor].setMonitorDesiredAction(CONTINUE);
    signal_monitors[pred_pt_error_monitor].setMonitorDesiredAction(CONTINUE);
    signal_monitors[loc_delay_monitor].setMonitorDesiredAction(CONTINUE);

    filtered_lat_err_ = 0;
    filtered_head_err_ = 0;
    filtered_pred_lat_err_ = 0;
    filtered_pred_head_err_ = 0;
  }

}
} // safety_monitor
} // vtr
#endif