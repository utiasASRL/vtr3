//#include <asrl/diagnostics/Diagnostics.hpp>

const int asrl::safetyMonitor::deadman_monitor_input::axis_array[] =  {7,7,7,7, 7,7, 7,7,6,6, 6,6,6,6, 6,6};
const int asrl::safetyMonitor::deadman_monitor_input::value_array[] = {1,0,1,0,-1,0,-1,0,1,0,-1,0,1,0,-1,0};

// Create the monitor
asrl::safetyMonitor::deadman_monitor_input::deadman_monitor_input(ros::NodeHandle nh) :
  safetyMonitorInput(nh),
  followingMode_(MANUAL),
  codeState_(0)
{

    signal_monitors.clear();
    signalMonitor empty_signal_monitor(nh);

    /********************
    // Initialize Message Subscriptions
    ********************/
    gamepadSubscriber_ = nodeHandle_.subscribe("in/joy",1,&deadman_monitor_input::gamepadCallback,this);

    /********************
    // Initialize DEADMAN monitor
    ********************/
    signal_monitors.push_back(empty_signal_monitor);
    deadman_monitor = signal_monitors.size()-1;
    double msg_timeout = 1;
    signal_monitors[deadman_monitor].initialize_type(DISCRETE_MONITOR);
    signal_monitors[deadman_monitor].initialize("Deadman Monitor", msg_timeout);
    asrl::rosutil::param<int>(nodeHandle_, "base/deadman_button_index", deadman_button_index_, 1);

    // Initialize the gamepad callback
    timeofLastCodeSequence_ = ros::Time::now();

}

/********************
// Define Message Callback Functions
********************/

void asrl::safetyMonitor::deadman_monitor_input::gamepadCallback(const sensor_msgs::Joy::Ptr & msg){

    signal_monitors[deadman_monitor].register_msg_but_no_status_change();

    bool pressed = msg->buttons[deadman_button_index_] != 0;

    if (signal_monitors[deadman_monitor].get_desired_action() == PAUSE){

        if (followingMode_ == AUTO || pressed){
            signal_monitors[deadman_monitor].set_monitor_desired_action(CONTINUE);
        } else {
            signal_monitors[deadman_monitor].set_monitor_desired_action(PAUSE);
        }

        // check the cheat code state
        checkCodeState(msg);
        // if the cheat code has been activated, then put the path tracker into auto mode.
        if(codeState_ == CHEAT_CODE_ACTIVATED){
            ROS_INFO_STREAM("CHEAT CODE ACTIVATED, SWITCHING TO AUTO MODE");
            followingMode_ =AUTO;
        }

    } else if (signal_monitors[deadman_monitor].get_desired_action() == CONTINUE){

        if (followingMode_ == AUTO){

            // Check if we need to disable AUTO mode
            for(int idx = 0; idx < msg->buttons.size(); ++idx){
                ros::Duration diff = ros::Time::now()-timeofLastCodeSequence_;

                if(msg->buttons[idx] == 1 && diff.toSec() > 1.0){
                    ROS_INFO("CHEAT CODE DEACTIVATED, SWITCHING TO MANUAL MODE\n");
                    followingMode_ = MANUAL;
                    codeState_ = 0;
                }
            }

            // Now that we've checked the cheat code and maybe changed followingMode_...
            if (followingMode_ == MANUAL && pressed == false){
                signal_monitors[deadman_monitor].set_monitor_desired_action(PAUSE);

            } else {
                signal_monitors[deadman_monitor].set_monitor_desired_action(CONTINUE);
            }

        } else if (pressed == false){
            signal_monitors[deadman_monitor].set_monitor_desired_action(PAUSE);

        } else {
            signal_monitors[deadman_monitor].register_msg_but_no_status_change();

        }

    } else {
        signal_monitors[deadman_monitor].set_monitor_desired_action(PAUSE);
    }

}

void asrl::safetyMonitor::deadman_monitor_input::checkCodeState(const sensor_msgs::Joy::Ptr & msg)
{
	ros::Duration timeDiff = ros::Time::now() - timeofLastCodeSequence_;
	//ROS_INFO("Time since last button: %f \n",timeDiff.toSec());

	if(timeDiff.toSec() > 0.75){
		codeState_ = 0;
        //ROS_INFO_STREAM("Too long since button press, codeState_ reset.");

    }
	if (codeState_ < 16){

        if (msg->axes[axis_array[codeState_]] == value_array[codeState_]){
            codeState_++;
            timeofLastCodeSequence_ = ros::Time::now();
        }

    } else if (codeState_ >= 16 && msg->buttons[deadman_button_index_] != 0){
        codeState_ = CHEAT_CODE_ACTIVATED;
	} else if (codeState_ == CHEAT_CODE_ACTIVATED){
        // Good shit.
	} else {
        // fail.
	}
}
