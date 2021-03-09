#ifndef ASRL_GRIZZLY_DIAGNOSTICS_MONITOR_HPP
#define ASRL_GRIZZLY_DIAGNOSTICS_MONITOR_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <std_msgs/Bool.h>

#include <diagnostic_msgs/DiagnosticArray.h>

namespace asrl {
  namespace safetyMonitor {

    class grizzly_diagnostics_monitor_input : public safetyMonitorInput
    {
    public :
        grizzly_diagnostics_monitor_input(ros::NodeHandle nh);
        void statusCallback(const diagnostic_msgs::DiagnosticArray::Ptr & msg);

        int estop_signal_monitor;

    private :
        ros::Subscriber statusSubscriber_;

    };

  } // safetyLayer
} // asrl

#include <safety_monitor/implementation/grizzly_diagnostics_msg_monitor.cpp>


#endif  // ASRL_GRIZZLY_DIAGNOSTICS_MONITOR_HPP
