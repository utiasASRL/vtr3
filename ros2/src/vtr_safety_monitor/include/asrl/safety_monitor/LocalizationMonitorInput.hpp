#ifndef ASRL_localization_MONITOR_HPP
#define ASRL_localization_MONITOR_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>

// Definition of the graph navigation status message
#include <asrl__messages/TrackingStatus.h>
#include <Eigen/Dense>
namespace asrl {
namespace safetyMonitor {

class LocalizationMonitorInput : public safetyMonitorInput {
 public :
  LocalizationMonitorInput(ros::NodeHandle nh);
  ~LocalizationMonitorInput() = default;

 private :
  void statusCallback(const asrl__messages::TrackingStatusConstPtr & status);
  // Subscriber to the tracking status message
  ros::Subscriber statusSubscriber_;

  // 1-sigma uncertainty limits for each dimension (x,y,z,r,p,y)
  Eigen::Matrix<double,6,1> uncertainty_limits;

  /// @brief The last time a status message was recieved
  ros::Time last_status_msg_time_;
};

}}

#include <asrl/safety_monitor/implementation/LocalizationMonitorInput.inl>


#endif  // ASRL_localization_MONITOR_HPP
