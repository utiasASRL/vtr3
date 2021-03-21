#ifndef ASRL_INCLINE_MONITOR_HPP
#define ASRL_INCLINE_MONITOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace asrl {
  namespace safetyMonitor {

    class incline_monitor_input : public safetyMonitorInput
    {
    public :
        incline_monitor_input(ros::NodeHandle nh);
        void statusCallback(const sensor_msgs::Imu::Ptr & /*msg*/);

        int incline_signal_monitor;
        int critical_incline_monitor;


    private :
        ros::Subscriber statusSubscriber_;

    };

  } // safetyLayer
} // asrl

#include <safety_monitor/implementation/incline_monitor.cpp>


#endif  // ASRL_INCLINE_MONITOR_HPP
