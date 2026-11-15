/**
 * \file visual_map_maintenance_module.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/odometry/visual_map_maintenance_module.hpp"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

namespace vtr {
namespace lidar {

using namespace tactic;

auto VisualMapMaintenanceModule::Config::fromROS(
    const rclcpp::Node::SharedPtr& node, const std::string& param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->feature_merge_gate = node->declare_parameter<float>(param_prefix + ".feature_merge_gate", config->feature_merge_gate);
  config->feature_merge_radius = node->declare_parameter<float>(param_prefix + ".feature_merge_radius", config->feature_merge_radius);
  config->default_feature_sigma = node->declare_parameter<float>(param_prefix + ".default_feature_sigma", config->default_feature_sigma);
  config->feature_life_time = node->declare_parameter<float>(param_prefix + ".feature_life_time", config->feature_life_time);
  config->descriptor_merge_dist = node->declare_parameter<float>(param_prefix + ".descriptor_merge_dist", config->descriptor_merge_dist);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void VisualMapMaintenanceModule::run_(QueryCache& qdata0, OutputCache&,
                                      const Graph::Ptr&,
                                      const TaskExecutor::Ptr&) {
  auto& qdata = dynamic_cast<LidarQueryCache&>(qdata0);

  if (config_->visualize && !publisher_initialized_) {
    map_pub_ =
        qdata.node->create_publisher<PointCloudMsg>("feature_map_odo", 5);
    publisher_initialized_ = true;
  }

  // construct the feature map if it does not exist
  if (!qdata.sliding_feature_map_odo)
    qdata.sliding_feature_map_odo.emplace(config_->feature_merge_gate,
                                          config_->feature_merge_radius,
                                          config_->descriptor_merge_dist);

  // do not update the map if registration failed
  if (!(*qdata.odo_success)) {
    CLOG(WARNING, "lidar.visual_map_maintenance")
        << "Point cloud registration failed - not updating the feature map.";
    return;
  }

  // no visual features this frame (e.g., use_visual disabled)
  if (!qdata.undistorted_intensity_features.valid()) return;
  const auto& features = *qdata.undistorted_intensity_features;
  if (features.empty()) return;

  auto& feature_map = *qdata.sliding_feature_map_odo;

  // Features are in the sensor frame, motion-undistorted to the scan end.
  // T_r_m_odo_prior is the robot pose at the scan end (timestamp_prior), so
  // the map-frame positions are p_m = (T_s_r · T_r_m_odo_prior)⁻¹ · p_s.
  const auto& T_s_r = *qdata.T_s_r;
  const auto& T_r_m = *qdata.T_r_m_odo_prior;
  const Eigen::Matrix4d T_m_s = (T_s_r.matrix() * T_r_m.matrix()).inverse();

  Eigen::Matrix<double, 3, Eigen::Dynamic> points_m =
      (T_m_s.block<3, 3>(0, 0) * features.points_3d).colwise() +
      T_m_s.block<3, 1>(0, 3);

  // covariances rotated into the map frame the same way (isotropic default
  // if the features come without one)
  const Eigen::Matrix3d R_m_s = T_m_s.block<3, 3>(0, 0);
  const int num_features = static_cast<int>(points_m.cols());
  std::vector<Eigen::Matrix3d> covs_m(num_features);
  const bool has_covs = features.hasCovs();
  const double default_var = static_cast<double>(config_->default_feature_sigma) *
                             static_cast<double>(config_->default_feature_sigma);
  for (int i = 0; i < num_features; ++i) {
    if (has_covs) {
      covs_m[i] = R_m_s * features.covs[i] * R_m_s.transpose();
      covs_m[i] = 0.5 * (covs_m[i] + covs_m[i].transpose());
    } else {
      covs_m[i] = default_var * Eigen::Matrix3d::Identity();
    }
  }

  std::vector<float> responses(features.keypoints.size());
  for (size_t i = 0; i < features.keypoints.size(); ++i)
    responses[i] = features.keypoints[i].response;

  // landmark that odometry matched each feature to (-1 = none). The indices
  // refer to the map as it was during odometry: it is only changed here.
  std::vector<int> matched_landmark;
  if (qdata.feature_matched_landmark.valid() &&
      static_cast<int>(qdata.feature_matched_landmark->size()) == num_features)
    matched_landmark = *qdata.feature_matched_landmark;

  feature_map.update(points_m, covs_m, features.descriptors, responses,
                     matched_landmark, config_->feature_life_time);
  const auto stats = feature_map.last_update();
  if (config_->feature_life_time > 0.0) feature_map.age();

  CLOG(DEBUG, "lidar.visual_map_maintenance")
      << "Updated feature map size is: " << feature_map.size()
      << " (matched " << stats.matched << ", merged " << stats.merged
      << ", added " << stats.added << ")";

  /// \note like the lidar submap visualization, this converts landmarks from
  /// the map frame to the vertex frame (shared with sliding_map_odo)
  if (config_->visualize) {
    pcl::PointCloud<pcl::PointXYZI> landmark_cloud;
    landmark_cloud.reserve(feature_map.size());
    Eigen::Matrix4f T_v_m = Eigen::Matrix4f::Identity();
    if (qdata.sliding_map_odo)
      T_v_m = qdata.sliding_map_odo->T_vertex_this().matrix().cast<float>();
    for (const auto& lm : feature_map.landmarks()) {
      const Eigen::Vector4f p_v =
          T_v_m * lm.p.homogeneous().cast<float>();
      pcl::PointXYZI pt;
      pt.x = p_v(0), pt.y = p_v(1), pt.z = p_v(2);
      pt.intensity = static_cast<float>(lm.num_obs);
      landmark_cloud.emplace_back(pt);
    }
    PointCloudMsg pc2_msg;
    pcl::toROSMsg(landmark_cloud, pc2_msg);
    pc2_msg.header.frame_id = "odo vertex frame";
    pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    map_pub_->publish(pc2_msg);
  }
}

}  // namespace lidar
}  // namespace vtr
