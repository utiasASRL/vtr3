
namespace asrl {
namespace safetyMonitor {

safetyMonitorInput::safetyMonitorInput(ros::NodeHandle nh) : nodeHandle_(nh) {
    // Do nothing
}

bool safetyMonitorInput::update_safety_monitor_action(int &desired_action, double &speed_limit,
                                                      std::vector <std::string> &limiting_signal_monitor_names,
                                                      std::vector<int> &limiting_signal_monitor_actions) {

    /** Check all signals for a given monitor **/
    for (int i = 0; i < signal_monitors.size(); i++) {

        int desired_action_signal;

        // Check how long it has been since the monitor received a signal!
        ros::Duration time_since_update_ros = ros::Time::now() - signal_monitors[i].get_last_update_time();
        double time_since_update = time_since_update_ros.toSec();

        // Get the desired behavior for the given signal
        if (time_since_update > signal_monitors[i].max_time_between_updates &&
            signal_monitors[i].max_time_between_updates > 0) {
            signal_monitors[i].set_status_unknown();
            desired_action_signal = UNKNOWN;

        } else if (signal_monitors[i].monitor_type == DISCRETE_MONITOR) {
            desired_action_signal = signal_monitors[i].get_desired_action();

        } else {
            ROS_ERROR("Incorrect definition of safety monitor type: only supporting Discrete monitors.");
            desired_action_signal = PAUSE;

        }

        // Store signal info if NOT continue
        if (desired_action_signal != CONTINUE) {
            limiting_signal_monitor_names.push_back(signal_monitors[i].monitor_name);
            limiting_signal_monitor_actions.push_back(desired_action_signal);
            speed_limit = std::min(speed_limit, signal_monitors[i].get_max_allowed_speed());
        }

        // Find the worst-case desired behavior
        desired_action = std::max(desired_action, desired_action_signal);

    }

    return true;
}

}
}
