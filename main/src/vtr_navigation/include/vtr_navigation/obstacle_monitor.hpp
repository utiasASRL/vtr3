// Copyright 2025
// Minimal obstacle monitor for VTR: subscribes to a local traversability
// occupancy grid and a planned path, checks the next N segments, and publishes
// a boolean status indicating whether the path ahead is blocked.

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace vtr {
namespace navigation {

class ObstacleMonitorNode : public rclcpp::Node {
 public:
  explicit ObstacleMonitorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  // Callbacks
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  // Core logic
  void evaluateBlocking();
  bool isSegmentBlocked(const geometry_msgs::msg::Pose &p0,
                        const geometry_msgs::msg::Pose &p1) const;
  bool worldToCostmap(double wx, double wy, int &mx, int &my) const;

  // Inputs
  nav_msgs::msg::OccupancyGrid::SharedPtr last_costmap_;
  nav_msgs::msg::Path::SharedPtr last_path_;

  // TF to transform path into costmap frame if needed
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  int lookahead_edges_;
  int occupancy_threshold_;
  double sample_step_m_;  // along-segment sampling distance
  double lookahead_distance_m_;  // if > 0, supersedes lookahead_edges_
  std::string costmap_frame_override_;
  std::string path_frame_override_;
  mutable bool printed_frames_ = false;
  bool last_blocked_ = false;  // rising-edge event emission

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
};

}  // namespace navigation
}  // namespace vtr


