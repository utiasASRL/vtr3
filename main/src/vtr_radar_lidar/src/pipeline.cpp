// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file pipeline.cpp
 * \author Keenan Burnett, Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar_lidar/pipeline.hpp"

#include "vtr_radar/data_types/pointmap_pointer.hpp"
#include "vtr_tactic/modules/factory.hpp"

namespace vtr {
namespace radar_lidar {

using namespace tactic;

auto RadarLidarPipeline::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                         const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // modules
  config->preprocessing = node->declare_parameter<std::vector<std::string>>(param_prefix + ".preprocessing", config->preprocessing);
  config->odometry = node->declare_parameter<std::vector<std::string>>(param_prefix + ".odometry", config->odometry);
  config->localization = node->declare_parameter<std::vector<std::string>>(param_prefix + ".localization", config->localization);
  // submap creation thresholds
  config->submap_translation_threshold = node->declare_parameter<double>(param_prefix + ".submap_translation_threshold", config->submap_translation_threshold);
  config->submap_rotation_threshold = node->declare_parameter<double>(param_prefix + ".submap_rotation_threshold", config->submap_rotation_threshold);
  
  config->save_raw_point_cloud = node->declare_parameter<bool>(param_prefix + ".save_raw_point_cloud", config->save_raw_point_cloud);
  // clang-format on
  return config;
}

RadarLidarPipeline::RadarLidarPipeline(
    const Config::ConstPtr &config,
    const std::shared_ptr<ModuleFactory> &module_factory,
    const std::string &name)
    : BasePipeline(module_factory, name), config_(config) {
  // preprocessing
  for (auto module : config_->preprocessing)
    preprocessing_.push_back(factory()->get("preprocessing." + module));
  // odometry
  for (auto module : config_->odometry)
    odometry_.push_back(factory()->get("odometry." + module));
  // localization
  for (auto module : config_->localization)
    localization_.push_back(factory()->get("localization." + module));
}

OutputCache::Ptr RadarLidarPipeline::createOutputCache() const {
  return std::make_shared<RadarLidarOutputCache>();
}

void RadarLidarPipeline::reset() {
  // reset modules
  for (const auto &module : preprocessing_) module->reset();
  for (const auto &module : odometry_) module->reset();
  for (const auto &module : localization_) module->reset();
  // odometry cached data
  sliding_map_odo_ = nullptr;
  timestamp_odo_ = nullptr;
  T_r_m_odo_ = nullptr;
  w_m_r_in_r_odo_ = nullptr;
  submap_vid_odo_ = tactic::VertexId::Invalid();
  T_sv_m_odo_ = tactic::EdgeTransform(true);
  // localization cached data
  submap_loc_ = nullptr;
}

void RadarLidarPipeline::preprocess_(const QueryCache::Ptr &qdata0,
                                     const OutputCache::Ptr &output0,
                                     const Graph::Ptr &graph,
                                     const TaskExecutor::Ptr &executor) {
  for (const auto &module : preprocessing_)
    module->run(*qdata0, *output0, graph, executor);
}

void RadarLidarPipeline::runOdometry_(const QueryCache::Ptr &qdata0,
                                      const OutputCache::Ptr &output0,
                                      const Graph::Ptr &graph,
                                      const TaskExecutor::Ptr &executor) {
  auto qdata = std::dynamic_pointer_cast<radar::RadarQueryCache>(qdata0);

  /// set the current sliding map for odometry
  if (sliding_map_odo_ != nullptr) {
    qdata->sliding_map_odo = sliding_map_odo_;
    qdata->timestamp_odo = timestamp_odo_;
    qdata->T_r_m_odo = T_r_m_odo_;
    qdata->w_m_r_in_r_odo = w_m_r_in_r_odo_;
  }

  for (const auto &module : odometry_)
    module->run(*qdata0, *output0, graph, executor);

  // store the current sliding map for odometry
  if (qdata->sliding_map_odo) {
    sliding_map_odo_ = qdata->sliding_map_odo.ptr();
    timestamp_odo_ = qdata->timestamp_odo.ptr();
    T_r_m_odo_ = qdata->T_r_m_odo.ptr();
    w_m_r_in_r_odo_ = qdata->w_m_r_in_r_odo.ptr();
  }
}

void RadarLidarPipeline::runLocalization_(const QueryCache::Ptr &qdata0,
                                          const OutputCache::Ptr &output0,
                                          const Graph::Ptr &graph,
                                          const TaskExecutor::Ptr &executor) {
  auto qdata = std::dynamic_pointer_cast<lidar::LidarQueryCache>(qdata0);

  // set the current map for localization
  if (submap_loc_ != nullptr) qdata->submap_loc = submap_loc_;

  for (const auto &module : localization_)
    module->run(*qdata0, *output0, graph, executor);

  /// store the current map for localization
  if (qdata->submap_loc) submap_loc_ = qdata->submap_loc.ptr();
}

void RadarLidarPipeline::onVertexCreation_(const QueryCache::Ptr &qdata0,
                                           const OutputCache::Ptr &,
                                           const Graph::Ptr &graph,
                                           const TaskExecutor::Ptr &) {
  const auto qdata = std::dynamic_pointer_cast<radar::RadarQueryCache>(qdata0);
  auto vertex = graph->at(*qdata->vid_odo);

  /// update current map vertex id and transform
  sliding_map_odo_->T_vertex_this() = *T_r_m_odo_;
  sliding_map_odo_->vertex_id() = *qdata->vid_odo;

  /// store the live frame point cloud
  // motion compensated point cloud
  {
    auto scan_odo = std::make_shared<radar::PointScan<radar::PointWithInfo>>();
    scan_odo->point_cloud() = *qdata->undistorted_point_cloud;
    scan_odo->T_vertex_this() = qdata->T_s_r->inverse();
    scan_odo->vertex_id() = *qdata->vid_odo;
    //
    using PointScanLM =
        storage::LockableMessage<radar::PointScan<radar::PointWithInfo>>;
    auto scan_odo_msg = std::make_shared<PointScanLM>(scan_odo, *qdata->stamp);
    vertex->insert<radar::PointScan<radar::PointWithInfo>>(
        "radar_filtered_point_cloud", "vtr_radar_msgs/msg/PointScan",
        scan_odo_msg);
  }
  // raw point cloud
  if (config_->save_raw_point_cloud)
  {
    auto raw_scan_odo =
        std::make_shared<radar::PointScan<radar::PointWithInfo>>();
    raw_scan_odo->point_cloud() = *qdata->raw_point_cloud;
    raw_scan_odo->T_vertex_this() = qdata->T_s_r->inverse();
    raw_scan_odo->vertex_id() = *qdata->vid_odo;
    //
    using PointScanLM =
        storage::LockableMessage<radar::PointScan<radar::PointWithInfo>>;
    auto raw_scan_odo_msg =
        std::make_shared<PointScanLM>(raw_scan_odo, *qdata->stamp);
    vertex->insert<radar::PointScan<radar::PointWithInfo>>(
        "radar_raw_point_cloud", "vtr_radar_msgs/msg/PointScan",
        raw_scan_odo_msg);
    CLOG(DEBUG, "radar.pipeline") << "Saved raw pointcloud to vertex" << vertex;
  }

  /// save the sliding map as vertex submap if we have traveled far enough
  const bool create_submap = [&] {
    //
    if (!submap_vid_odo_.isValid()) return true;
    //
    const auto T_sv_r = T_sv_m_odo_ * T_r_m_odo_->inverse();
    auto T_sv_r_vec = T_sv_r.vec();
    auto dtran = T_sv_r_vec.head<3>().norm();
    auto drot = T_sv_r_vec.tail<3>().norm() * 57.29577;  // 180/pi
    if (dtran > config_->submap_translation_threshold ||
        drot > config_->submap_rotation_threshold) {
      return true;
    }
    //
    return false;
  }();
  if (create_submap) {
    CLOG(DEBUG, "radar.pipeline")
        << "Create a submap for vertex " << *qdata->vid_odo;
    // copy the current sliding map
    auto submap_odo = std::make_shared<radar::PointMap<radar::PointWithInfo>>(
        *sliding_map_odo_);
    // save the submap
    using PointMapLM =
        storage::LockableMessage<radar::PointMap<radar::PointWithInfo>>;
    auto submap_msg = std::make_shared<PointMapLM>(submap_odo, *qdata->stamp);
    vertex->insert<radar::PointMap<radar::PointWithInfo>>(
        "radar_pointmap", "vtr_radar_msgs/msg/PointMap", submap_msg);
    // save a copy
    auto submap2_msg = std::make_shared<PointMapLM>(submap_odo, *qdata->stamp);
    vertex->insert<radar::PointMap<radar::PointWithInfo>>(
        "radar_pointmap_v" + std::to_string(submap_odo->version()),
        "vtr_radar_msgs/msg/PointMap", submap2_msg);

    // save the submap vertex id and transform
    submap_vid_odo_ = *qdata->vid_odo;
    T_sv_m_odo_ = *T_r_m_odo_;
  }

  /// save a pointer to the latest submap
  const auto submap_ptr = std::make_shared<radar::PointMapPointer>();
  submap_ptr->this_vid = *qdata->vid_odo;
  submap_ptr->map_vid = submap_vid_odo_;
  submap_ptr->T_v_this_map = (*T_r_m_odo_) * T_sv_m_odo_.inverse();
  //
  CLOG(DEBUG, "radar.pipeline")
      << "Saving submap pointer from this vertex " << *qdata->vid_odo
      << " to map vertex " << submap_vid_odo_ << " with transform T_v_map_this "
      << submap_ptr->T_v_this_map.inverse().vec().transpose();
  using PointMapPointerLM = storage::LockableMessage<radar::PointMapPointer>;
  auto submap_ptr_msg =
      std::make_shared<PointMapPointerLM>(submap_ptr, *qdata->stamp);
  vertex->insert<radar::PointMapPointer>("radar_pointmap_ptr",
                                         "vtr_radar_msgs/msg/PointMapPointer",
                                         submap_ptr_msg);
}

}  // namespace radar_lidar
}  // namespace vtr
