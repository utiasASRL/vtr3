#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vtr_messages/msg/robot_status.hpp>

#include <vtr_safety_monitor/base/safety_monitor_input_base.hpp>

#include <Eigen/Dense>

using RobotStatus = vtr_messages::msg::RobotStatus;

namespace vtr {
namespace safety_monitor {

class LocalizationMonitorInput : public SafetyMonitorInput {
 public :
  /** \brief Constructor */
  explicit LocalizationMonitorInput(std::shared_ptr<rclcpp::Node> node);
  ~LocalizationMonitorInput() = default;

 private :
  /** \brief Checks that current localization covariance is under limits if doing Repeat */
  void statusCallback(RobotStatus::SharedPtr status);

  /** \brief Checks T_leaf_twig distance aka distance travelled since localization */
  void voCallback(const std_msgs::msg::Int32::SharedPtr msg);

  /** \brief Subscriber to the tracking status message */
  rclcpp::Subscription<RobotStatus>::SharedPtr status_subscriber_;

  /** \brief Subscriber to the tracking status message */
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr vo_subscriber_;

  /** \brief 1-sigma uncertainty limits for each dimension (x,y,yaw) */
  Eigen::Vector3d uncertainty_limits;


  /** \brief If we've dead-reckoned farther than this, slow down */
  int max_frames_on_vo_slow_;

  /** \brief If we've dead-reckoned farther than this, stop */
  int max_frames_on_vo_stop_;
};

} // safety_monitor
} // vtr
