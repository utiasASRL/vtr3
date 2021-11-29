#pragma once

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "vtr_tactic/tactic_v2.hpp"

namespace vtr {
namespace tactic {

using ROSPathMsg = nav_msgs::msg::Path;

class TacticCallback : public TacticCallbackInterface {
 public:
  TacticCallback(const rclcpp::Node::SharedPtr& node) {
    tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    odo_path_pub_ = node->create_publisher<ROSPathMsg>("odo_path", 10);
    loc_path_pub_ = node->create_publisher<ROSPathMsg>("loc_path", 10);

    // world offset for localization path visualization
    tf_static_bc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    Eigen::Vector3d vis_loc_path_offset;
    vis_loc_path_offset << 0.0, 0.0, -10.0;
    Eigen::Affine3d T(Eigen::Translation3d{vis_loc_path_offset});
    auto msg = tf2::eigenToTransform(T);
    msg.header.frame_id = "world";
    msg.child_frame_id = "world (offset)";
    tf_static_bc_->sendTransform(msg);
  }

  /// for visualization in ROS
  void publishOdometryRviz(const TacticV2& tactic,
                           const QueryCache& qdata) override {
    auto lock = tactic.chain_->guard();

    /// Publish the latest keyframe estimate in world frame
    Eigen::Affine3d T(tactic.T_w_m_odo_.matrix());
    auto msg = tf2::eigenToTransform(T);
    msg.header.frame_id = "world";
    msg.header.stamp = *(qdata.rcl_stamp);
    msg.child_frame_id = "odometry keyframe";
    tf_bc_->sendTransform(msg);

    /// Publish the teach path
    ROSPathMsg path;
    path.header.frame_id = "world";
    path.header.stamp = *(qdata.rcl_stamp);
    path.poses = tactic.keyframe_poses_;
    odo_path_pub_->publish(path);

    /// Publish the current frame
    Eigen::Affine3d T2(qdata.T_r_m_odo->inverse().matrix());
    auto msg2 = tf2::eigenToTransform(T2);
    msg2.header.frame_id = "odometry keyframe";
    msg2.header.stamp = *(qdata.rcl_stamp);
    msg2.child_frame_id = *(qdata.robot_frame);
    tf_bc_->sendTransform(msg2);
    if (*(qdata.robot_frame) != "robot") {
      msg2.child_frame_id = "robot";
      tf_bc_->sendTransform(msg2);
    }
  }

  void publishPathRviz(const TacticV2& tactic) override {
    std::vector<Eigen::Affine3d> eigen_poses;
    /// publish the repeat path in
    for (unsigned i = 0; i < tactic.chain_->size(); i++) {
      eigen_poses.push_back(Eigen::Affine3d(tactic.chain_->pose(i).matrix()));
    }

    /// Publish the repeat path with an offset
    ROSPathMsg path;
    path.header.frame_id = "world (offset)";
    auto& poses = path.poses;
    for (const auto& pose : eigen_poses) {
      PoseStampedMsg ps;
      ps.pose = tf2::toMsg(pose);
      poses.push_back(ps);
    }
    loc_path_pub_->publish(path);
  }

  void publishLocalizationRviz(const TacticV2& tactic,
                               const QueryCache& qdata) override {
    auto lock = tactic.chain_->guard();

    /// Publish the current frame localized against in world frame
    Eigen::Affine3d T(tactic.T_w_m_loc_.matrix());
    auto msg = tf2::eigenToTransform(T);
    msg.header.stamp = *qdata.rcl_stamp;
    msg.header.frame_id = "world";
    msg.child_frame_id = "localization keyframe";
    tf_bc_->sendTransform(msg);

    // apply an offset to separate odometry and localization
    msg.header.frame_id = "world (offset)";
    msg.child_frame_id = "localization keyframe (offset)";
    tf_bc_->sendTransform(msg);
  }

 private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_bc_;
  rclcpp::Publisher<ROSPathMsg>::SharedPtr odo_path_pub_;
  rclcpp::Publisher<ROSPathMsg>::SharedPtr loc_path_pub_;
};

}  // namespace tactic
}  // namespace vtr