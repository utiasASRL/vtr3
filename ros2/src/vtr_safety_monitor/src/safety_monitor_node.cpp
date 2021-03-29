#include <vtr_safety_monitor/safety_monitor_node.hpp>

#include <vtr_logging/logging_init.hpp>

namespace vtr {
namespace safety_monitor {

// Unknown is interpreted as PAUSE for the path tracker
const std::string safety_monitor_node::desired_action_str_array[] = {"NOT_READY", "CONTINUE",
                                                                     "MAINTAIN_SLOW", "SLOW_DOWN",
                                                                     "PAUSE", "PAUSE",
                                                                     "PAUSE_AND_RELOCALIZE",
                                                                     "HALT_AND_REPLAN",
                                                                     "HALT_AND_BLOCK"};

safety_monitor_node::safety_monitor_node(ros::NodeHandle nh) :
    nodeHandle_(nh)
{
    safetyStatusPeriod = 0.2;
    safetyStatusPublisher_ = nodeHandle_.advertise<asrl__messages::DesiredActionIn>("out/desired_action",
                                                                                                 1, true);
    safetyDebugPublisher_ = nodeHandle_.advertise<asrl__safety_monitor::MonitorDebug>("out/debug", 1, true);
    asrl::rosutil::param<double>(nodeHandle_, "max_allowed_speed", absolute_max_speed_, 5.0);
    asrl::rosutil::param<double>(nodeHandle_, "relocation_timeout", timeOutSeconds_, 10.0);

//    asrl::rosutil::param<double>(nodeHandle_,"block_distance",initialBlockDistance_,10.0);
//    asrl::rosutil::param<double>(nodeHandle_,"block_distance_step",stepBlockDistance_,5.0);
//    asrl::rosutil::param<int>(nodeHandle_,"obstruction_spacing",obsSpacing_,10);

//    blockDistance_ = initialBlockDistance_;
//    graphnavObsClient_ = nodeHandle_.serviceClient<asrl__mapping__graph_navigation::ObstructionTest>("/GraphNavigation/map/graphnav/obstruction_test");
}

bool safety_monitor_node::initialize_monitors() {

    bool success = false;
    asrl::rosutil::param<std::vector<std::string> >(nodeHandle_, "list_of_monitors", list_of_monitors,
                                                    std::vector<std::string>());

    ROS_INFO_STREAM("Found " << list_of_monitors.size() << " monitors.");

    if (list_of_monitors.size() > 0) {
        for (int i = 0; i < list_of_monitors.size(); i++) {
            monitor_vector.push_back(SafetyMonitorFactory(list_of_monitors[i], nodeHandle_));
        }

        initialize_update_timer();

        success = true;
    } else {
        // fail.
    }
    return success;
}

void safety_monitor_node::initialize_update_timer() {
    getSafetyStatusTimer = nodeHandle_.createTimer(ros::Duration(safetyStatusPeriod),
                                                   &safety_monitor_node::getSafetyStatusCallback, this, false);
}

void safety_monitor_node::spin() {
    ros::spin();
    return;
}

safetyMonitorInput *safety_monitor_node::SafetyMonitorFactory(
    std::string monitor_input_str, ros::NodeHandle nh) {

    const char *string = monitor_input_str.c_str();

    ROS_INFO_STREAM("Initializing " << monitor_input_str.c_str());

    /** Safety Monitor Factory
    *  This function converts strings provided in the launch file into safety monitor objects
    *
    **/

//    if (std::strcmp(string, "vtr_monitor") == 0){
//        return new vtr_monitor_input(nh);
//    } else
    if (std::strcmp(string, "localization_monitor") == 0) {
        return new LocalizationMonitorInput(nh);
    } else if (std::strcmp(string, "deadman_monitor") == 0) {
        return new deadman_monitor_input(nh);
    } else if (std::strcmp(string, "path_tracker_monitor") == 0) {
        ROS_INFO("CREATING A PATH-Tracker MONITOR");
        return new path_tracker_monitor_input(nh);
    } else if (std::strcmp(string, "estop_monitor") == 0) {
        return new estop_monitor_input(nh);
    } else if (std::strcmp(string, "incline_monitor") == 0) {
        return new incline_monitor_input(nh);
    } else if (std::strcmp(string, "grizzly_diagnostics_monitor") == 0) {
        return new grizzly_diagnostics_monitor_input(nh);
    }
    // else if (std::strcmp(string, "terrain_assessment_monitor") == 0){
        // return new terrain_assessment_monitor_input(nh);
    // }
    else {
        ROS_ERROR_STREAM(
            "SafetyMonitorFactory: Safety monitor launch script is requesting a monitor that doesn't exist. See: safety_monitor_node.cpp, SafetyMonitorFactory()");
        throw std::invalid_argument("Invalid monitor name");
    }
}

void safety_monitor_node::getSafetyStatusCallback(const ros::TimerEvent & /*timerEvent*/) {

    /** Initialize values **/
    int desired_action = CONTINUE;
    double speed_limit = absolute_max_speed_;
    std::vector<std::string> limiting_signal_monitor_names;
    limiting_signal_monitor_names.clear();
    std::vector<int> limiting_signal_monitor_actions;
    limiting_signal_monitor_actions.clear();

    /** Go through all monitors and signals to determine highest priority behavior and speed limit **/
    for (int i = 0; i < monitor_vector.size(); i++) {
        monitor_vector[i]->update_safety_monitor_action(desired_action, speed_limit, limiting_signal_monitor_names,
                                                        limiting_signal_monitor_actions);
    }

    //ROS_INFO_STREAM("Current safety layer speed limit: " << speed_limit);
    //ROS_INFO_STREAM("Current desired action: " << desired_action);

    /** Construct the safety monitor msg **/
    asrl__messages::DesiredActionIn msg;
    asrl__safety_monitor::MonitorDebug debug_msg;

    bool valid_action = true;
    if (desired_action == CONTINUE) {
        // The path tracker will ramp up to the prescheduled speed
        msg.desired_action = "CONTINUE";

    } else if (desired_action == SLOW) {
        // The path tracker will slow to the speed specified in msg.speed_limit
        msg.desired_action = "SLOW";

    } else if (desired_action == PAUSE) {
        // The path tracker will pause, but not discard the current path
        msg.desired_action = "PAUSE";

    } else if (desired_action == PAUSE_AND_RELOCALIZE) {
        // The path tracker will pause, but not discard the current path
        msg.desired_action = "PAUSE_AND_RELOCALIZE";

    } else if (desired_action == HALT) {
        // The path tracker will pause AND discard the current path
        msg.desired_action = "HALT";

    } else if (desired_action == HALT_AND_REPLAN) {
        // The path tracker will pause AND discard the current path
        msg.desired_action = "HALT_AND_REPLAN";

    } else if (desired_action == HALT_AND_BLOCK) {
        // The path tracker will pause AND discard the current path
        msg.desired_action = "HALT_AND_BLOCK";

    } else if (desired_action == UNKNOWN) {
        // One or more monitors is unknown, the path tracker will pause the current path
        msg.desired_action = "PAUSE";

    } else {
        valid_action = false;
        ROS_ERROR("Safety monitor node:  Desired path tracker action not in range.");
        msg.desired_action = "PAUSE";
        msg.speed_limit = 0.05;

        debug_msg.limiting_signal_monitor_names.push_back("Safety_Monitor_Issue");
        debug_msg.limiting_signal_monitor_actions.push_back(PAUSE);

    }

    if (valid_action == true) {
        msg.speed_limit = speed_limit;
        debug_msg.limiting_signal_monitor_names = limiting_signal_monitor_names;
        debug_msg.limiting_signal_monitor_actions = limiting_signal_monitor_actions;
    }

    safetyStatusPublisher_.publish(msg);
    safetyDebugPublisher_.publish(debug_msg);

}


}
}

// The main function!
int main(int argc, char **argv) {
    // Initialize Safety Monitor Node
    std::string nodeName = "Safety_Monitor";
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nodeHandle_("~");
    asrl::safetyMonitor::safety_monitor_node safety_monitor_node(nodeHandle_);

    ROS_INFO_STREAM("Booting up.");

    safety_monitor_node.initialize_monitors();

    safety_monitor_node.spin();
}

#endif