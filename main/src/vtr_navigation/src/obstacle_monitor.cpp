#include "vtr_navigation/obstacle_monitor.hpp"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace vtr {
namespace navigation {

ObstacleMonitorNode::ObstacleMonitorNode(const rclcpp::NodeOptions &options)
    : Node("obstacle_monitor", options) {
  // Parameters with sensible defaults
  lookahead_edges_ = this->declare_parameter<int>("lookahead_edges", 10);
  occupancy_threshold_ = this->declare_parameter<int>("occupancy_threshold", 50);
  sample_step_m_ = this->declare_parameter<double>("sample_step_m", 0.1);
  lookahead_distance_m_ = this->declare_parameter<double>("lookahead_distance_m", 5.0);
  costmap_frame_override_ = this->declare_parameter<std::string>("costmap_frame", "");
  path_frame_override_ = this->declare_parameter<std::string>("path_frame", "");

  status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vtr/obstacle_status", 10);

  auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = cb_group;

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/output_costmap", rclcpp::QoS(10),
      std::bind(&ObstacleMonitorNode::costmapCallback, this, std::placeholders::_1), sub_opt);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/vtr/planned_path", rclcpp::QoS(10),
      std::bind(&ObstacleMonitorNode::pathCallback, this, std::placeholders::_1), sub_opt);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ObstacleMonitorNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  last_costmap_ = msg;
  evaluateBlocking();
}

void ObstacleMonitorNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  last_path_ = msg;
  evaluateBlocking();
}

void ObstacleMonitorNode::evaluateBlocking() {
  if (!last_costmap_ || !last_path_ || last_path_->poses.size() < 2) return;

  // Ensure path is in costmap frame. If frame differs, transform each pose.
  const std::string cm_frame = costmap_frame_override_.empty() ? last_costmap_->header.frame_id : costmap_frame_override_;
  const std::string path_frame = path_frame_override_.empty() ? last_path_->header.frame_id : path_frame_override_;

  if (!printed_frames_) {
    RCLCPP_INFO(this->get_logger(), "ObstacleMonitor frames: costmap='%s', planned_path='%s'", cm_frame.c_str(), path_frame.c_str());
    printed_frames_ = true;
  }

  std::vector<geometry_msgs::msg::Pose> poses_cm;
  poses_cm.reserve(last_path_->poses.size());
  if (cm_frame == path_frame || path_frame.empty()) {
    for (const auto &ps : last_path_->poses) poses_cm.push_back(ps.pose);
  } else {
    try {
      auto T = tf_buffer_->lookupTransform(cm_frame, path_frame, tf2::TimePointZero);
      for (const auto &ps : last_path_->poses) {
        geometry_msgs::msg::PoseStamped out;
        tf2::doTransform(ps, out, T);
        poses_cm.push_back(out.pose);
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF transform %s->%s failed: %s. You can override with params costmap_frame/path_frame.",
                           path_frame.c_str(), cm_frame.c_str(), e.what());
      return;
    }
  }

  // Check up to next N segments
  size_t seg_count = poses_cm.size() - 1;
  if (lookahead_distance_m_ > 0.0) {
    // compute how many segments fit in the distance cap
    double acc = 0.0;
    size_t capped = 0;
    for (size_t i = 0; i + 1 < poses_cm.size(); ++i) {
      const double dx = poses_cm[i + 1].position.x - poses_cm[i].position.x;
      const double dy = poses_cm[i + 1].position.y - poses_cm[i].position.y;
      acc += std::hypot(dx, dy);
      ++capped;
      if (acc >= lookahead_distance_m_) break;
    }
    seg_count = std::min(seg_count, capped);
  } else {
    seg_count = std::min<size_t>(lookahead_edges_, seg_count);
  }
  bool blocked = false;
  for (size_t i = 0; i < seg_count; ++i) {
    if (isSegmentBlocked(poses_cm[i], poses_cm[i + 1])) {
      blocked = true;
      break;
    }
  }

  std_msgs::msg::Bool msg;
  msg.data = blocked;
  status_pub_->publish(msg);

  // One-shot WARN log on rising edge (no extra topic)
  if (blocked && !last_blocked_) {
    RCLCPP_WARN(this->get_logger(), "ObstacleDetected: path ahead blocked");
  }
  last_blocked_ = blocked;
}

bool ObstacleMonitorNode::isSegmentBlocked(const geometry_msgs::msg::Pose &p0,
                                           const geometry_msgs::msg::Pose &p1) const {
  if (!last_costmap_) return false;

  const auto &cm = *last_costmap_;

  // Sample along segment every sample_step_m_
  const double dx = p1.position.x - p0.position.x;
  const double dy = p1.position.y - p0.position.y;
  const double length = std::hypot(dx, dy);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / sample_step_m_)));

  for (int s = 0; s <= steps; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(steps);
    const double wx = p0.position.x + t * dx;
    const double wy = p0.position.y + t * dy;

    int mx = 0, my = 0;
    if (!worldToCostmap(wx, wy, mx, my)) continue;  // out of map → ignore
    const size_t idx = static_cast<size_t>(my) * cm.info.width + static_cast<size_t>(mx);
    if (idx >= cm.data.size()) continue;
    const int8_t occ = cm.data[idx];
    if (occ >= occupancy_threshold_) return true;
  }
  return false;
}

bool ObstacleMonitorNode::worldToCostmap(double wx, double wy, int &mx, int &my) const {
  if (!last_costmap_) return false;
  const auto &cm = *last_costmap_;
  const double res = cm.info.resolution;
  const double ox = cm.info.origin.position.x;
  const double oy = cm.info.origin.position.y;

  const double dx = wx - ox;
  const double dy = wy - oy;
  if (dx < 0.0 || dy < 0.0) return false;

  mx = static_cast<int>(dx / res);
  my = static_cast<int>(dy / res);

  if (mx < 0 || my < 0) return false;
  if (mx >= static_cast<int>(cm.info.width) || my >= static_cast<int>(cm.info.height)) return false;
  return true;
}

}  // namespace navigation
}  // namespace vtr

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vtr::navigation::ObstacleMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


