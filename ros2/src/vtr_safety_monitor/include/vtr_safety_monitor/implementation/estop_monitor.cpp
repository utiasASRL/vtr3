
// Create the monitor
asrl::safetyMonitor::estop_monitor_input::estop_monitor_input(ros::NodeHandle nh) :
  safetyMonitorInput(nh)
{

    signal_monitors.clear();
    signalMonitor empty_signal_monitor(nh);

    /********************
    // Initialize Message Subscriptions
    ********************/
    statusSubscriber_ = nodeHandle_.subscribe("in/estop_status",1,&estop_monitor_input::statusCallback,this);

    /********************
    // Initialize ESTOP monitor
    ********************/
    signal_monitors.push_back(empty_signal_monitor);
    estop_signal_monitor = signal_monitors.size()-1;
    double msg_timeout = 1;
    signal_monitors[estop_signal_monitor].initialize_type(DISCRETE_MONITOR);
    signal_monitors[estop_signal_monitor].initialize("EStop Monitor", msg_timeout);

}

/********************
// Define Message Callback Functions
********************/

void asrl::safetyMonitor::estop_monitor_input::statusCallback(const std_msgs::Bool::Ptr & msg){

    bool estop_active = (msg->data == true);

    if (estop_active){
        signal_monitors[estop_signal_monitor].set_monitor_desired_action(PAUSE);
    } else {
        signal_monitors[estop_signal_monitor].set_monitor_desired_action(CONTINUE);
    }

}
