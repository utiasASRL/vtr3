#if 0
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <vtr_logging/logging.hpp>
#include <vtr_safety_monitor/base/safety_monitor_input_base.hpp>

#include <vtr_messages/msg/path_tracker_status.hpp>

using PTStatus = vtr_messages::msg::PathTrackerStatus;

namespace vtr {
namespace safety_monitor {

class PathTrackerMonitorInput : public SafetyMonitorInput {
 public :
  PathTrackerMonitorInput(const std::shared_ptr<rclcpp::Node> node);
  void statusCallback(const PTStatus::SharedPtr status);
  double max_allowed_speed_;

 private :

  rclcpp::Subscription<PTStatus>::SharedPtr statusSubscriber_;

  int pt_error_monitor;
  int pred_pt_error_monitor;
  int loc_delay_monitor;

  double acceptable_lateral_error_, acceptable_heading_error_;
  double acceptable_pred_lateral_error_, acceptable_pred_heading_error_, pred_tracking_error_speed_reduction_pct_;
  double max_dist_between_vo_or_loc_updates_, max_time_between_vo_or_loc_updates_;
  double absolute_max_speed_;
  double min_limit_speed_;
  double max_speed_filter_const_;

  double tracking_error_filter_time_;
  double pred_tracking_error_filter_time_;
  double localization_delay_filter_time_;

  double filtered_lat_err_, filtered_head_err_, filtered_pred_lat_err_, filtered_pred_head_err_;

  rclcpp::Time last_pathTracker_status_msg_time_;

  double get_speed_increase(double spd_in, double pct_change) const;
  double get_speed_decrease(double spd_in, double pct_change) const;

};

} // safety_monitor
} // vtr
#endif