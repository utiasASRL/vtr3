#ifndef ASRL_ESTOP_MONITOR_HPP
#define ASRL_ESTOP_MONITOR_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <std_msgs/Bool.h>

namespace asrl {
  namespace safetyMonitor {

    class estop_monitor_input : public safetyMonitorInput
    {
    public :
        estop_monitor_input(ros::NodeHandle nh);
        void statusCallback(const std_msgs::Bool::Ptr & msg);

        int estop_signal_monitor;


    private :
        ros::Subscriber statusSubscriber_;

    };

  } // safetyLayer
} // asrl

#include <asrl/safety_monitor/implementation/estop_monitor.cpp>


#endif  // ASRL_ESTOP_MONITOR_HPP
