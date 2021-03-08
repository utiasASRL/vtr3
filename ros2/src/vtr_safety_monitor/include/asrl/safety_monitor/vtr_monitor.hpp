#ifndef ASRL_VTR_MONITOR_HPP
#define ASRL_VTR_MONITOR_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>

// Definition of the graph navigation status message
#include <asrl__mapping__graph_navigation/Status.h>
#include <asrl__mapping__graph_navigation/KeyFrameNavigationStatus.h>
#include <asrl__messages/PausePath.h>
#include <asrl__control__path_tracker/ResumePath.h>

namespace asrl {
  namespace safetyMonitor {

    class vtr_monitor_input : public safetyMonitorInput
    {
    public :
        vtr_monitor_input(ros::NodeHandle nh);
        void statusCallback(const asrl__mapping__graph_navigation::StatusConstPtr& status);
        void keyframeNavigationStatusCallback(const asrl__mapping__graph_navigation::KeyFrameNavigationStatusConstPtr & status);

    private :

        // Monitor indices
        int vo_monitor;
        int obs_monitor;
        int matched_feature_monitor;
        int desired_action_monitor;

        bool pausePathTrackerCallback(asrl__control__path_tracker::PausePath::Request& req,
                                            asrl__control__path_tracker::PausePath::Response& resp);
        bool resumePathTrackerCallback(asrl__control__path_tracker::ResumePath::Request& req,
                                             asrl__control__path_tracker::ResumePath::Response& resp);

        ros::Subscriber statusSubscriber_;
        ros::Subscriber keyFrameNavigationStatusSubscriber_;
        ros::ServiceServer pausePathTrackerService_;
        ros::ServiceServer resumePathTrackerService_;

        bool voFailure_;
        bool mapMatchFailure_;
        int mapMatchCount_;
        double blockDistance_;

        bool voLatch_;

        // Parameters
        double maxDistanceSinceGraphLocalization_;
        double slowDistanceSinceGraphLocalization_;

        double minMatchedFeatures_, maxMatchedFeatures_;
        double filterWindowTime_, filteredMatchedFeatures_;
        double minMatchedFeaturesSpeed_;
        double absoluteMaxSpeed_;
        ros::Time lastMatchedFeaturesMsgTime_;

        double initialBlockDistance_;
        double stepBlockDistance_;

/*        double minMatchedFeatures_, maxMatchedFeatures_;
        double filterWindowTime_, filteredMatchedFeatures_;
        double minMatchedFeaturesSpeed_;
        double absoluteMaxSpeed_;
        ros::Time lastMatchedFeaturesMsgTime_;
   */
    };

  } // safetyLayer
} // asrl

#include <asrl/safety_monitor/implementation/vtr_monitor.cpp>


#endif  // ASRL_VTR_MONITOR_HPP
