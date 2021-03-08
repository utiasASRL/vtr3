#ifndef ASRL_PATH_TRACKER_MONITOR_HPP
#define ASRL_PATH_TRACKER_MONITOR_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>

// Definition of the graph navigation status message
#include <asrl__control__path_tracker/StatusOut.h>

namespace asrl {
  namespace safetyMonitor {

    class path_tracker_monitor_input : public safetyMonitorInput
    {
    public :
        path_tracker_monitor_input(ros::NodeHandle nh);
        void statusCallback(const asrl__control__path_tracker::StatusOutConstPtr & status);
        double max_allowed_speed_;

    private :

        ros::Subscriber statusSubscriber_;

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

        ros::Time last_pathTracker_status_msg_time_;

        double get_speed_increase(float spd_in, float pct_change);
        double get_speed_decrease(float spd_in, float pct_change);

    };

  } // safetyLayer
} // asrl

#include <asrl/safety_monitor/implementation/path_tracker_monitor.cpp>


#endif  // ASRL_PATH_TRACKER_MONITOR_HPP
