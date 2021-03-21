
// Create the monitor
asrl::safetyMonitor::grizzly_diagnostics_monitor_input::grizzly_diagnostics_monitor_input(ros::NodeHandle nh) :
  safetyMonitorInput(nh)
{

    signal_monitors.clear();
    signalMonitor empty_signal_monitor(nh);

    /********************
    // Initialize Message Subscriptions
    ********************/
    statusSubscriber_ = nodeHandle_.subscribe("in/grizzly_diagnostics_status",1,&grizzly_diagnostics_monitor_input::statusCallback,this);

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

void asrl::safetyMonitor::grizzly_diagnostics_monitor_input::statusCallback(const diagnostic_msgs::DiagnosticArray::Ptr & msg){

    //ROS_INFO_STREAM("Rx diagnostics msg");

    for (int i = 0; i < msg->status.size(); i++){
        for (int j = 0; j < msg->status[i].values.size(); j++){
            if (msg->status[i].values[j].key == "e-stop"){
                if (msg->status[i].values[j].value == "True"){
                    signal_monitors[estop_signal_monitor].set_monitor_desired_action(PAUSE);
                    //ROS_INFO_STREAM("Estop Stopping Robot.");
                } else if (msg->status[i].values[j].value == "False"){
                    signal_monitors[estop_signal_monitor].set_monitor_desired_action(CONTINUE);
                    //ROS_INFO_STREAM("Estop OK.");
                } else {
                    ROS_WARN("Issue parsing e-stop diagnostic message.");
                }
            }
        }
    }


    signal_monitors[estop_signal_monitor].register_msg_but_no_status_change();
}




