#ifndef ASRL_DEADMAN_MONITOR_HPP
#define ASRL_DEADMAN_MONITOR_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Joy.h>

enum FOLLOWING_MODE {
    AUTO,
    MANUAL
};

namespace asrl {
  namespace safetyMonitor {

    class deadman_monitor_input : public safetyMonitorInput
    {
    public :
        deadman_monitor_input(ros::NodeHandle nh);
        void gamepadCallback(const sensor_msgs::Joy::Ptr & msg);
	    const static int axis_array[16];
	    const static int value_array[16];

    private :
        ros::Subscriber gamepadSubscriber_;

        int deadman_button_index_;
        int pause_button_index_;
        bool deadman_pressed_;
		FOLLOWING_MODE followingMode_;

		int deadman_monitor;

        // the state of the cheat code.
		int codeState_;
		// the timestamp when hte last correct value in the code sequence was added.
		ros::Time timeofLastCodeSequence_;

		// The state when the cheat code is active.
	    static const int CHEAT_CODE_ACTIVATED=7777;

	    void checkCodeState(const sensor_msgs::Joy::Ptr & msg);


    };

  } // safetyLayer
} // asrl

#include <safety_monitor/implementation/deadman_monitor.cpp>


#endif  // ASRL_DEADMAN_MONITOR_HPP
