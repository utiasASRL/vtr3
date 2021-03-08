//#include <asrl/diagnostics/Diagnostics.hpp>

#include "../vtr_monitor.hpp"

// Create the monitor
asrl::safetyMonitor::vtr_monitor_input::vtr_monitor_input(ros::NodeHandle nh) :
  safetyMonitorInput(nh)
{

    signal_monitors.clear();
    signalMonitor empty_signal_monitor(nh);

    /********************
    // Initialize Message Subscriptions
    ********************/
    statusSubscriber_ = nodeHandle_.subscribe<asrl__mapping__graph_navigation::Status>("in/graph_status", 100, &vtr_monitor_input::statusCallback, this);
    keyFrameNavigationStatusSubscriber_ = nodeHandle_.subscribe<asrl__mapping__graph_navigation::KeyFrameNavigationStatus>("in/keyframe_navigation_status",10, &vtr_monitor_input::keyframeNavigationStatusCallback, this);
    pausePathTrackerService_ = nodeHandle_.advertiseService("in/pause_request", &vtr_monitor_input::pausePathTrackerCallback, this);
    resumePathTrackerService_ = nodeHandle_.advertiseService("in/resume_request", &vtr_monitor_input::resumePathTrackerCallback, this);


    /********************
    // Initialize VO monitor
    ********************/
    signal_monitors.push_back(empty_signal_monitor);
    vo_monitor = signal_monitors.size()-1;
    double msg_timeout = 5;
    signal_monitors[vo_monitor].initialize_type(DISCRETE_MONITOR);
    signal_monitors[vo_monitor].initialize("VO Distance Limit", msg_timeout);
    voFailure_ = false;
    voLatch_ = false;
    mapMatchFailure_ = false;
    mapMatchCount_ = 0;
    asrl::rosutil::param<double>(nodeHandle_,"maxDistanceSinceGraphLocalization",maxDistanceSinceGraphLocalization_,1);
    asrl::rosutil::param<double>(nodeHandle_,"block_distance",initialBlockDistance_,10.0);
    asrl::rosutil::param<double>(nodeHandle_,"block_distance_step",stepBlockDistance_,5.0);
    blockDistance_ = initialBlockDistance_;


    /********************
    // Initialize Obstruction monitor
    ********************/
    signal_monitors.push_back(empty_signal_monitor);
    obs_monitor = signal_monitors.size()-1;
    msg_timeout = 5;
    signal_monitors[obs_monitor].initialize_type(DISCRETE_MONITOR);
    signal_monitors[obs_monitor].initialize("Path Tracker Obstructions", msg_timeout);


    /********************
    // Initialize Matched Feature monitor
    ********************/
    signal_monitors.push_back(empty_signal_monitor);
    matched_feature_monitor = signal_monitors.size()-1;

    msg_timeout = 5;

    signal_monitors[matched_feature_monitor].initialize_type(DISCRETE_MONITOR);
    signal_monitors[matched_feature_monitor].initialize("VT&R Matched Features", msg_timeout);
    asrl::rosutil::param<double>(nodeHandle_,"minMatchedFeatures",maxMatchedFeatures_,50);  // count
    asrl::rosutil::param<double>(nodeHandle_,"minMatchedFeaturesSpeed",minMatchedFeaturesSpeed_,1.0); // m/s
    asrl::rosutil::param<double>(nodeHandle_,"max_allowed_speed",absoluteMaxSpeed_,5.0); // m/s
    asrl::rosutil::param<double>(nodeHandle_,"matched_features_filter_time",filterWindowTime_,5.0); // m/s
    filteredMatchedFeatures_ = 0; // count

    minMatchedFeatures_ = std::min(10.0, maxMatchedFeatures_-1);

    lastMatchedFeaturesMsgTime_ = ros::Time::now();

    /********************
    // Initialize Desired Action monitor
    ********************/
    signal_monitors.push_back(empty_signal_monitor);
    desired_action_monitor = signal_monitors.size()-1;
    msg_timeout = -1;
    signal_monitors[desired_action_monitor].initialize_type(DISCRETE_MONITOR);
    signal_monitors[desired_action_monitor].initialize("VTR Desired Action", msg_timeout);
    signal_monitors[desired_action_monitor].set_monitor_desired_action(CONTINUE);

}

/********************
// Define Message Callback Functions
********************/

void asrl::safetyMonitor::vtr_monitor_input::keyframeNavigationStatusCallback(const asrl__mapping__graph_navigation::KeyFrameNavigationStatusConstPtr & status)
{
    voFailure_ = status->voFailure;
    mapMatchFailure_ = status->mapMatchFailure;
    mapMatchCount_ = status->keypoint_match_count;
    return;
}

void asrl::safetyMonitor::vtr_monitor_input::statusCallback(const asrl__mapping__graph_navigation::StatusConstPtr& status)
{
    /* This monitor has complicated logic...
     *
     * 1) The soft VO distance is exceeded, and the VO monitor triggers a route blockage.  This also triggers the
     * path tracker to abandon tracking.  GraphNav knows the chain is obstructed now, but has not changed state.
     *
     * 2) Graphnav publishes a status with the obstructed flag, triggering the obstruction monitor.  This would halt
     * the path tracker, but it has already been halted in (1).  A state change command is issued by the obs monitor,
     * and the VO monitor returns to normal as we increment the acceptable distance.
     *
     * 3) Graphnav transitions to the obstructed state and looks for other routes.  Both monitors clear their
     * blocking signals, as we are no longer in the following state.  This is safe as the path tracker is still not
     * doing anything.
     *
     * 4) Normal operation resumes when GraphNav finds a route.
     *
     * Step:            1 (map fail)     2 (path obstructed)     3 (replanning)    4 (reroute successful)
     *
     * VO  Monitor      HALT_AND_BLOCK      CONTINUE                CONTINUE           CONTINUE
     * Obs Monitor      CONTINUE            HALT_AND_REPLAN         CONTINUE           CONTINUE
     * Path Tracker     HALT_AND_REPLAN     WAITING                 WAITING            FOLLOWING
     * GraphNav         FOLLOWING           FOLLOWING               OBSTRUCTED         FOLLOWING
     *
     */

    if ( status->state == std::string("Following") || status->state == std::string("FollowingRelocalization") ){
        // Update vo_monitor
        if (status->metersSinceLastMapLocalization > blockDistance_ && !voLatch_) {
            voLatch_ = true;
//            blockDistance_ = maxDistanceSinceGraphLocalization_ + stepBlockDistance_;
            signal_monitors[vo_monitor].set_monitor_desired_action(HALT_AND_BLOCK);
            ROS_WARN_STREAM("VO Monitor: " << status->metersSinceLastMapLocalization << "m since localization exceeds " << blockDistance_ << "m threshold. HALT_AND_BLOCK");
        }
        else if ( (status->metersSinceLastMapLocalization > maxDistanceSinceGraphLocalization_) || (voFailure_ && mapMatchFailure_)){
            signal_monitors[vo_monitor].set_monitor_desired_action(PAUSE_AND_RELOCALIZE);
        } else {
            signal_monitors[vo_monitor].set_monitor_desired_action(CONTINUE);
        }

        // Reset the acceptable VO distance and voLatch_ when we match against the map again
        if (!mapMatchFailure_ && status->metersSinceLastMapLocalization < blockDistance_){
//            blockDistance_ = initialBlockDistance_;
            voLatch_ = false;
        }

        // Don't trigger the obstruction monitor if we are obstructing due to VO failure
        if (status->isObstructed && !voLatch_){
            signal_monitors[obs_monitor].set_monitor_desired_action(HALT_AND_REPLAN);
        } else {
            signal_monitors[obs_monitor].set_monitor_desired_action(CONTINUE);
        }

        /** Update matched features monitor **/
        // Compute filtered estimate of matched features
/*
        ros::Duration t_lastMsgRos = ros::Time::now() - lastMatchedFeaturesMsgTime_;
        double t_lastMsg = std::min(t_lastMsgRos.toSec(), filterWindowTime_);

        lastMatchedFeaturesMsgTime_ = ros::Time::now();

        //ROS_INFO("t_lastMsg: %f, filterWindow: %f \n", t_lastMsg, filterWindowTime_);

        filteredMatchedFeatures_ = ((filterWindowTime_ - t_lastMsg)*filteredMatchedFeatures_ + t_lastMsg*mapMatchCount_)/filterWindowTime_;
        bool sufficient_features = filteredMatchedFeatures_ > minMatchedFeatures_;

        // Compute linear interp for matched features
        double m = (absoluteMaxSpeed_ - minMatchedFeaturesSpeed_)/(maxMatchedFeatures_-minMatchedFeatures_);
        double b = absoluteMaxSpeed_ - m*maxMatchedFeatures_ */


        /** Update matched features monitor **/
        // Compute filtered estimate of matched features
        ros::Duration t_lastMsgRos = ros::Time::now() - lastMatchedFeaturesMsgTime_;
        double t_lastMsg = std::min(t_lastMsgRos.toSec(), filterWindowTime_);

        filteredMatchedFeatures_ = ((filterWindowTime_ - t_lastMsg)*filteredMatchedFeatures_ + t_lastMsg*mapMatchCount_)/filterWindowTime_;
        bool sufficient_features = filteredMatchedFeatures_ > maxMatchedFeatures_;

        // Compute speed limit considering the matched features
        double m = (absoluteMaxSpeed_ - minMatchedFeaturesSpeed_)/(maxMatchedFeatures_-minMatchedFeatures_);
        double b = absoluteMaxSpeed_ - m*maxMatchedFeatures_;

        double new_spd_limit = std::min(absoluteMaxSpeed_, std::max(minMatchedFeaturesSpeed_, m*filteredMatchedFeatures_+b));
        signal_monitors[matched_feature_monitor].set_max_allowed_speed(new_spd_limit);

        if (sufficient_features == true){
            signal_monitors[matched_feature_monitor].set_monitor_desired_action(CONTINUE);
            //ROS_INFO("Sufficient features (%f): %f", filteredMatchedFeatures_, new_spd_limit);
        } else {
            signal_monitors[matched_feature_monitor].set_monitor_desired_action(SLOW);
            //ROS_INFO("Requesting spd for insufficient features (%f): %f", filteredMatchedFeatures_, new_spd_limit);
        }

    // } else if (status->state == std::string("Obstructed") {
    //     // Ensure we stay paused until GraphNav finds and publishes a new path
    //     signal_monitors[obs_monitor].set_monitor_desired_action(PAUSE);

    } else {
        signal_monitors[vo_monitor].set_monitor_desired_action(CONTINUE);
        signal_monitors[obs_monitor].set_monitor_desired_action(CONTINUE);
        signal_monitors[matched_feature_monitor].set_monitor_desired_action(CONTINUE);
    }

    lastMatchedFeaturesMsgTime_ = ros::Time::now();
}

bool asrl::safetyMonitor::vtr_monitor_input::pausePathTrackerCallback(asrl__messages::PausePath::Request& req,
                                                       asrl__messages::PausePath::Response& resp)
{
    signal_monitors[desired_action_monitor].set_monitor_desired_action(PAUSE);
    return true;
}


bool asrl::safetyMonitor::vtr_monitor_input::resumePathTrackerCallback(asrl__control__path_tracker::ResumePath::Request& req,
                                                        asrl__control__path_tracker::ResumePath::Response& resp)
{
    signal_monitors[desired_action_monitor].set_monitor_desired_action(CONTINUE);
    return true;
}
