#ifndef ASRL_SIGNAL_MONITOR_HPP
#define ASRL_SIGNAL_MONITOR_HPP

// For leaky bucket
//#include <deque>

const int DISCRETE_MONITOR = 1;

// Define possible desired actions
// Integers are used to determine priority, higher num = higher priority
const int CONTINUE = 1;
const int SLOW = 2;
const int UNKNOWN = 3;
const int PAUSE = 4;
const int PAUSE_AND_RELOCALIZE = 5;
const int HALT = 6;
const int HALT_AND_REPLAN = 7;
const int HALT_AND_BLOCK = 8;

namespace asrl {
namespace safetyMonitor {

class signalMonitor
{
    public :
        // Constructor
        signalMonitor(ros::NodeHandle nh);
        void initialize_type(int /*type*/);
        void initialize(std::string name, double max_time_in);
        int monitor_type;

        // Interactions for discrete monitor
        void set_monitor_desired_action(int desired_action);
        void register_msg_but_no_status_change();

        // Interactions for all signal monitor types
        void set_status_unknown();
        double get_max_allowed_speed();
        void set_max_allowed_speed(double & max_allowed_speed_in);
        ros::Time get_last_update_time();
        void set_msg_timeout(double new_msg_timeout);
        double max_time_between_updates;
        int get_desired_action();
        std::string monitor_name;

    private :

        ros::NodeHandle nodeHandle_;

        // Monitor variables
        bool signal_monitor_status_unknown;
        int desired_action;
        ros::Time last_update;
        double max_allowed_speed_;

}; // class signalMonitor

} // namespace safetyMonitor
} // namespace asrl

#include <safety_monitor/base/implementation/signal_monitor.cpp>

#endif // ASRL_SIGNAL_MONITOR_HPP
