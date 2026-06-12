/**
 * \file localization_map_recall_module_lv.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Recalls the point map from the teach graph (same as the base module)
 * and additionally loads intensity features from the teach vertex,
 * populating qdata.map_intensity_features for visual localization.
 */
#include "vtr_lidar/modules/localization/localization_map_recall_module_lv.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "vtr_lidar/data_types/intensity_feature_map.hpp"
#include "vtr_lidar/data_types/intensity_features.hpp"
#include "vtr_lidar/data_types/pointmap_pointer.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto LocalizationMapRecallModuleLV::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->map_version = node->declare_parameter<std::string>(param_prefix + ".map_version", config->map_version);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void LocalizationMapRecallModuleLV::run_(QueryCache &qdata0, OutputCache &,
                                         const Graph::Ptr &graph,
                                         const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  /// Create publishers for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    map_pub_ = qdata.node->create_publisher<PointCloudMsg>("submap_loc", 5);
    test_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("submap_loc_test", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  /// Input
  const auto &vid_loc = *qdata.vid_loc;

  // ════════════════════════════════════════════════════════════════════════
  //  Recall point map (same as base module)
  // ════════════════════════════════════════════════════════════════════════
  const auto pointmap_ptr = [&] {
    const auto vertex = graph->at(vid_loc);
    const auto msg = vertex->retrieve<PointMapPointer>(
        "pointmap_ptr", "vtr_lidar_msgs/msg/PointMapPointer");
    auto locked_msg = msg->sharedLocked();
    return locked_msg.get().getData();
  }();

  CLOG(INFO, "lidar.localization_map_recall_lv")
      << "Loaded pointmap pointer with this_vid " << pointmap_ptr.this_vid
      << " and map_vid " << pointmap_ptr.map_vid;

  /// sanity check
  if (pointmap_ptr.this_vid != vid_loc) {
    CLOG(ERROR, "lidar.localization_map_recall_lv")
        << "pointmap pointer this_vid mismatch.";
    throw std::runtime_error("pointmap pointer this_vid mismatch.");
  }

  /// load the submap if we have switched to a new one
  if (qdata.submap_loc &&
      qdata.submap_loc->vertex_id() == pointmap_ptr.map_vid) {
    CLOG(INFO, "lidar.localization_map_recall_lv")
        << "Map already loaded, simply return. Map size is: "
        << qdata.submap_loc->size();
    qdata.submap_loc_changed.emplace(false);
  } else {
    auto vertex = graph->at(pointmap_ptr.map_vid);
    CLOG(INFO, "lidar.localization_map_recall_lv")
        << "Loading map " << config_->map_version << " from vertex "
        << vid_loc;
    const auto specified_map_msg = vertex->retrieve<PointMap<PointWithInfo>>(
        config_->map_version, "vtr_lidar_msgs/msg/PointMap");
    if (specified_map_msg == nullptr) {
      CLOG(ERROR, "lidar.localization_map_recall_lv")
          << "Could not find map " << config_->map_version << " at vertex "
          << vid_loc;
      throw std::runtime_error("Could not find map " + config_->map_version +
                               " at vertex " + std::to_string(vid_loc));
    }
    auto locked_specified_map_msg = specified_map_msg->sharedLocked();
    qdata.submap_loc = std::make_shared<PointMap<PointWithInfo>>(
        locked_specified_map_msg.get().getData());
    qdata.submap_loc_changed.emplace(true);
  }

  /// update the submap to vertex transformation
  qdata.T_v_m_loc.emplace(pointmap_ptr.T_v_this_map *
                           qdata.submap_loc->T_vertex_this());

  // ════════════════════════════════════════════════════════════════════════
  //  Recall the merged visual feature submap from the map vertex
  //  (same vertex as the point submap; falls back to the legacy per-vertex
  //  single-frame intensity features when the teach graph has no feature
  //  submap, e.g. graphs taught before feature mapping existed)
  // ════════════════════════════════════════════════════════════════════════
  bool feature_map_recalled = false;
  try {
    /// load the feature submap if we have switched to a new one
    if (qdata.feature_map_loc &&
        qdata.feature_map_loc->vertex_id() == pointmap_ptr.map_vid) {
      feature_map_recalled = true;
    } else {
      auto map_vertex = graph->at(pointmap_ptr.map_vid);
      const auto feature_map_msg = map_vertex->retrieve<IntensityFeatureMap>(
          "intensity_feature_map", "vtr_lidar_msgs/msg/IntensityFeatureMap");
      if (feature_map_msg != nullptr) {
        auto locked_feature_map_msg = feature_map_msg->sharedLocked();
        qdata.feature_map_loc = std::make_shared<IntensityFeatureMap>(
            locked_feature_map_msg.get().getData());
        feature_map_recalled = true;
        CLOG(INFO, "lidar.localization_map_recall_lv")
            << "Loaded feature submap with " << qdata.feature_map_loc->size()
            << " landmarks from vertex " << pointmap_ptr.map_vid;
      }
    }
  } catch (const std::exception& e) {
    CLOG(WARNING, "lidar.localization_map_recall_lv")
        << "Feature submap recall failed: " << e.what();
    feature_map_recalled = false;
  }

  if (feature_map_recalled) {
    /// update the feature-submap to vertex transformation (landmarks live in
    /// the teach odometry map frame, anchored like the point submap)
    qdata.T_v_m_feature_loc.emplace(
        pointmap_ptr.T_v_this_map * qdata.feature_map_loc->T_vertex_this());
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Legacy fallback: recall single-frame intensity features from the
  //  current localization vertex
  // ════════════════════════════════════════════════════════════════════════
  if (!feature_map_recalled) {
    auto vertex = graph->at(vid_loc);
    const auto feat_msg = vertex->retrieve<IntensityFeatures>(
        "intensity_features", "vtr_lidar_msgs/msg/IntensityFeatures");

    if (feat_msg != nullptr) {
      auto locked_feat_msg = feat_msg->sharedLocked();
      const auto &features = locked_feat_msg.get().getData();

      // Store as single-element vector (one vertex worth of features)
      // The features' 3D points are stored in the vertex frame.
      std::vector<IntensityFeatures> map_features;
      map_features.push_back(features);
      qdata.map_intensity_features.emplace(std::move(map_features));

      CLOG(DEBUG, "lidar.localization_map_recall_lv")
          << "Recalled " << features.size()
          << " intensity features from vertex " << vid_loc;
    } else {
      CLOG(WARNING, "lidar.localization_map_recall_lv")
          << "No intensity features found at vertex " << vid_loc;
      // Emplace an empty vector so downstream modules know recall was attempted
      qdata.map_intensity_features.emplace(std::vector<IntensityFeatures>{});
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Visualization (same as base module)
  // ════════════════════════════════════════════════════════════════════════
  if (config_->visualize) {
    // clang-format off
    const auto T_v_m = qdata.T_v_m_loc->matrix();
    auto point_map = qdata.submap_loc->point_cloud();  // makes a copy
    auto map_point_mat = point_map.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::cartesian_offset());

    Eigen::Matrix3f C_v_m = (T_v_m.block<3, 3>(0, 0)).cast<float>();
    Eigen::Vector3f r_m_v_in_v = (T_v_m.block<3, 1>(0, 3)).cast<float>();
    map_point_mat = (C_v_m * map_point_mat).colwise() + r_m_v_in_v;

    PointCloudMsg pc2_msg;
    pcl::toROSMsg(point_map, pc2_msg);
    pc2_msg.header.frame_id = "loc vertex frame (offset)";
    pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    map_pub_->publish(pc2_msg);
    // clang-format on
  }
}

}  // namespace lidar
}  // namespace vtr
