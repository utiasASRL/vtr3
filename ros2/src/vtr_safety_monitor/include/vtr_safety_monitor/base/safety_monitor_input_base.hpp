#include <rclcpp/rclcpp.hpp>
//#include <ros/timer.h>

 For setting minimum speed
//#include <std_msgs/Float32.h>

// For sending audible messages to sound_play
//#include <sound_play/SoundRequest.h>

// Some useful functions for nodes, not necessarily used in the
// abstract definition, but it is likely used in when actually
// creating a node object.
//#include <asrl/rosutil/node_utilities.hpp>
//#include <asrl/common/rosutil/param.hpp>

/*
// Define possible desired actions
const int HALT_AND_BLOCK = 8;
const int HALT_AND_REPLAN = 7;
const int PAUSE_AND_RELOCALIZE = 6;
const int PAUSE = 5;
const int UNKNOWN = 4;
const int SLOW_DOWN = 3;
const int MAINTAIN_SLOW = 2;
const int CONTINUE = 1;
const int NOT_READY = 0;

*/

// These are defined so that we don't ahev to worry when things are changed
const int MAX_ACTION = 8;
const int MIN_ACTION = 0;

// Definition of a generic signal monitor
#include <vtr_safety_monitor/base/signal_monitor.hpp>

namespace vtr {
namespace safety_monitor {

class safetyMonitorInput
{
public :
    safetyMonitorInput(ros::NodeHandle nh);
    int getDesiredAction();
    bool update_safety_monitor_action(int & desired_action, double & speed_limit, std::vector<std::string> & limiting_signal_monitor_names, std::vector<int> & limiting_signal_monitor_actions);

protected :
    ros::NodeHandle nodeHandle_;
    std::vector<signalMonitor> signal_monitors;

}; // class safetyMonitorInput

} // namespace safety_monitor
} // namespace vtr

#include <safety_monitor/base/implementation/safety_monitor_input_base.cpp>
