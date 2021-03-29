#if 0   // not ported yet

// Create the monitor
asrl::safetyMonitor::incline_monitor_input::incline_monitor_input(ros::NodeHandle nh) :
  safetyMonitorInput(nh)
{

    signal_monitors.clear();
    signalMonitor empty_signal_monitor(nh);

    /********************
    // Initialize Message Subscriptions
    ********************/
    statusSubscriber_ = nodeHandle_.subscribe("in/incline_status",1,&incline_monitor_input::statusCallback,this);

    /********************
    // Initialize Incline monitor
    ********************/
    /*
    signal_monitors.push_back(empty_signal_monitor);
    incline_signal_monitor = signal_monitors.size()-1;
    int bucket_size = 5;
    int thresh_fail_intermit = 3;
    int thresh_fail = 4;
    float msg_timeout = 1;
    signal_monitors[incline_signal_monitor].initialize_type(LEAKY_BUCKET_MONITOR);
    signal_monitors[incline_signal_monitor].initialize("Incline Monitor",bucket_size,thresh_fail_intermit,thresh_fail,msg_timeout);
    // set_actions(pass, fail_intermittent, fail, not_ready, unknown);
    signal_monitors[incline_signal_monitor].set_actions(CONTINUE, SLOW_DOWN, SLOW_DOWN, PAUSE, PAUSE);
    */

    /********************
    // Initialize Critical Incline monitor
    ********************/
    /*
    signal_monitors.push_back(empty_signal_monitor);
    critical_incline_monitor = signal_monitors.size()-1;
    bucket_size = 5;
    thresh_fail_intermit = 3;
    thresh_fail = 4;
    msg_timeout = 1;
    signal_monitors[critical_incline_monitor].initialize_type(LEAKY_BUCKET_MONITOR);
    signal_monitors[critical_incline_monitor].initialize("Critical Incline Monitor",bucket_size,thresh_fail_intermit,thresh_fail,msg_timeout);
    // set_actions(pass, fail_intermittent, fail, not_ready, unknown);
    signal_monitors[critical_incline_monitor].set_actions(CONTINUE, SLOW_DOWN, PAUSE, PAUSE, PAUSE);
    */

}

/********************
// Define Message Callback Functions
********************/

void asrl::safetyMonitor::incline_monitor_input::statusCallback(const sensor_msgs::Imu::Ptr & /*msg*/){

    //bool estop_active = (msg->data == true);

    //signal_monitors[incline_signal_monitor].set_monitor_pass();
    //signal_monitors[critical_incline_monitor].set_monitor_pass();

}


/*

    // Check to make sure that the attitude estimate of the vehicle is still reasonable
    // This is legacy code from the BERNS teach pass, long traverses using only VO result in the
    // pose estimate pitching up
    tf::Transform T_v_0(transform.inverse());
    if ( !isAttitudeSafe(T_v_0) )
    {
        std::string abortDescription = "Pathtracker is stopping the vehicle because the vehicle's pitch estimate is " +
        asrl::padded( angles::to_degrees(getPitchFrom(T_v_0)), 3, 3 ) + " degrees!";
        setPathAborted(-3, abortDescription);
        return true;
    }

    */

#endif