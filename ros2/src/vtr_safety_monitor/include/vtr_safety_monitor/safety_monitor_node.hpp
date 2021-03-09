#include <rclcpp/rclcpp.hpp>

#include <vtr_safety_monitor/base/safety_monitor_input_base.hpp>

#include <vtr_messages/msg/desired_action_in.hpp>
#include <vtr_messages/msg/robot_status.hpp>

#if 0
#include <std_msgs/String.h>

// Definition of the safety monitor input base


//Definition of the safety monitor messages
#include <asrl__messages/DesiredActionIn.h>
#include <asrl__safety_monitor/MonitorDebug.h>

// Definition of the graph navigation status message
//#include <asrl__mapping__graph_navigation/Status.h>
#include <asrl__messages/RobotStatus.h>
//#include <asrl__mapping__graph_navigation/KeyFrameNavigationStatus.h>


// The various safety monitor inputs:
//#include <asrl/safety_monitor/vtr_monitor.hpp>
#include <safety_monitor/deadman_monitor.hpp>
#include <safety_monitor/path_tracker_monitor.hpp>
#include <safety_monitor/LocalizationMonitorInput.hpp>
#include <safety_monitor/incline_monitor.hpp>
#include <safety_monitor/grizzly_diagnostics_msg_monitor.hpp>
#include <safety_monitor/estop_monitor.hpp>
// #include <asrl/safety_monitor/terrain_assessment_monitor.hpp>
#endif


namespace vtr {
namespace safety_monitor {

class safety_monitor_node
{
public :
    safety_monitor_node(ros::NodeHandle nh);
    virtual void spin(void);

    bool initialize_monitors();
    void initialize_update_timer();

    const static std::string desired_action_str_array[];

private :
    ros::NodeHandle nodeHandle_;

    // List of Monitors
    std::vector<std::string> list_of_monitors;
    std::vector<asrl::safetyMonitor::safetyMonitorInput *> monitor_vector;

    // Objects for periodic status updates
    ros::Timer getSafetyStatusTimer;
    double safetyStatusPeriod;
    void getSafetyStatusCallback(const ros::TimerEvent & /*timerEvent*/);
    ros::Publisher safetyStatusPublisher_;
    ros::Publisher safetyDebugPublisher_;

    safetyMonitorInput * SafetyMonitorFactory(std::string monitor_input_str, ros::NodeHandle nh);

    // Parameters
    double initialTimeOutSeconds_;
    double timeOutSeconds_;
    double absolute_max_speed_;

};

} // safety_monitor
} // vtr

