namespace asrl {
namespace safetyMonitor {


signalMonitor::signalMonitor(ros::NodeHandle nh) :
    nodeHandle_(nh) {
    desired_action = PAUSE;
    signal_monitor_status_unknown = true;
    asrl::rosutil::param<double>(nodeHandle_, "max_allowed_speed", max_allowed_speed_, 5.0);
}

void signalMonitor::initialize_type(int /*type_in*/) {
    monitor_type = DISCRETE_MONITOR;
}

void signalMonitor::initialize(std::string name_in, double max_time_in) {
    monitor_name = name_in;
    last_update = ros::Time::now();
    max_time_between_updates = max_time_in;
}

// Interactions
void signalMonitor::set_monitor_desired_action(int desired_action_in) {
    desired_action = desired_action_in;
    last_update = ros::Time::now();
    signal_monitor_status_unknown = false;
}

int signalMonitor::get_desired_action() {
    return desired_action;
}

void signalMonitor::set_status_unknown() {
    signal_monitor_status_unknown = true;
}

void signalMonitor::register_msg_but_no_status_change() {
    last_update = ros::Time::now();
    signal_monitor_status_unknown = false;
}

ros::Time signalMonitor::get_last_update_time() {
    return last_update;
}

void signalMonitor::set_msg_timeout(double new_msg_timeout) {
    max_time_between_updates = new_msg_timeout;
}


double signalMonitor::get_max_allowed_speed() {
    return max_allowed_speed_;
}

void signalMonitor::set_max_allowed_speed(double &max_allowed_speed_in) {
    max_allowed_speed_ = std::abs(max_allowed_speed_in);
}

}
}
