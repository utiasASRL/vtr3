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
#include "vtr_tactic/modules/factory.hpp"

namespace vtr {
namespace radar_lidar {

using namespace tactic;

auto RadarLidarPipeline::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                         const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->preprocessing = node->declare_parameter<std::vector<std::string>>(param_prefix + ".preprocessing", config->preprocessing);
  config->odometry = node->declare_parameter<std::vector<std::string>>(param_prefix + ".odometry", config->odometry);
  config->localization = node->declare_parameter<std::vector<std::string>>(param_prefix + ".localization", config->localization);
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
  cartesian_odo_ = nullptr;
  point_cloud_odo_ = nullptr;
  point_map_odo_ = nullptr;
  timestamp_odo_ = nullptr;
  T_r_m_odo_ = nullptr;
  w_m_r_in_r_odo_ = nullptr;
  new_scan_odo_.clear();
#if false
  new_raw_scan_odo_.clear();
#endif
  curr_map_loc_ = nullptr;
}

void RadarLidarPipeline::preprocess_(const QueryCache::Ptr &qdata0,
                                     const OutputCache::Ptr &output0,
                                     const Graph::Ptr &graph,
                                     const TaskExecutor::Ptr &executor) {
  for (auto module : preprocessing_)
    module->run(*qdata0, *output0, graph, executor);
}

void RadarLidarPipeline::runOdometry_(const QueryCache::Ptr &qdata0,
                                      const OutputCache::Ptr &output0,
                                      const Graph::Ptr &graph,
                                      const TaskExecutor::Ptr &executor) {
  auto qdata = std::dynamic_pointer_cast<radar::RadarQueryCache>(qdata0);

  // set the current map for odometry
  if (point_map_odo_ != nullptr) {
    qdata->cartesian_odo = cartesian_odo_;
    qdata->point_cloud_odo = point_cloud_odo_;
    qdata->point_map_odo = point_map_odo_;
    qdata->timestamp_odo = timestamp_odo_;
    qdata->T_r_m_odo = T_r_m_odo_;
    qdata->w_m_r_in_r_odo = w_m_r_in_r_odo_;
  }

  for (auto module : odometry_) module->run(*qdata0, *output0, graph, executor);

  // store the current map for odometry to avoid reloading
  if (qdata->point_map_odo) {
    cartesian_odo_ = qdata->cartesian_odo.ptr();
    point_cloud_odo_ = qdata->point_cloud_odo.ptr();
    point_map_odo_ = qdata->point_map_odo.ptr();
    timestamp_odo_ = qdata->timestamp_odo.ptr();
    T_r_m_odo_ = qdata->T_r_m_odo.ptr();
    w_m_r_in_r_odo_ = qdata->w_m_r_in_r_odo.ptr();
  }

  /// Store the scan if decided to store it
  const auto &T_s_r = *qdata->T_s_r;
  const auto &T_r_m = *qdata->T_r_m_odo;
  const auto T_s_m = T_s_r * T_r_m;
  bool store_scan = false;
  if (!(*qdata->odo_success))
    store_scan = false;
  else if (new_scan_odo_.empty())
    store_scan = true;
  else {
    const auto &T_m_sm1 = new_scan_odo_.rbegin()->second->T_vertex_this();
    // T_<robot>_<vid odo> * T_<vid odo>_<sensor-1> * T_<sensor>_<robot>
    auto T_s_sm1_vec = (T_s_m * T_m_sm1).vec();
    auto dtran = T_s_sm1_vec.head<3>().norm();
    auto drot = T_s_sm1_vec.tail<3>().norm() * 57.29577;  // 180/pi
    CLOG(DEBUG, "radar.pipeline")
        << "Relative motion since first radar scan: tran: " << dtran
        << ", rot: " << drot << " with map size: " << new_scan_odo_.size();
    /// \todo parameters
    if (dtran > 0.3 || drot > 5.0) store_scan = true;
  }
  if (store_scan) {
#if false  /// store raw point cloud
    // raw scan
    auto new_raw_scan_odo = std::make_shared<radar::PointScan<radar::PointWithInfo>>();
    new_raw_scan_odo->point_cloud() = *qdata->undistorted_raw_point_cloud;
    new_raw_scan_odo->T_vertex_this() = T_s_m.inverse();
    new_raw_scan_odo_.try_emplace(*qdata->stamp, new_raw_scan_odo);
#endif
    // preprocessed scan
    auto new_scan_odo =
        std::make_shared<radar::PointScan<radar::PointWithInfo>>();
    new_scan_odo->point_cloud() = *qdata->undistorted_point_cloud;
    new_scan_odo->T_vertex_this() = T_s_m.inverse();
    new_scan_odo_.try_emplace(*qdata->stamp, new_scan_odo);
  }
}

void RadarLidarPipeline::runLocalization_(const QueryCache::Ptr &qdata0,
                                          const OutputCache::Ptr &output0,
                                          const Graph::Ptr &graph,
                                          const TaskExecutor::Ptr &executor) {
  auto qdata = std::dynamic_pointer_cast<lidar::LidarQueryCache>(qdata0);

  // set the current map for localization
  if (curr_map_loc_ != nullptr) qdata->curr_map_loc = curr_map_loc_;

  for (auto module : localization_)
    module->run(*qdata0, *output0, graph, executor);

  /// store the current map for localization
  if (qdata->curr_map_loc) curr_map_loc_ = qdata->curr_map_loc.ptr();
}

void RadarLidarPipeline::onVertexCreation_(const QueryCache::Ptr &qdata0,
                                           const OutputCache::Ptr &,
                                           const Graph::Ptr &graph,
                                           const TaskExecutor::Ptr &) {
  const auto qdata = std::dynamic_pointer_cast<radar::RadarQueryCache>(qdata0);
  const auto vid_odo = *qdata->vid_odo;

  // update current map vertex id and transform
  point_map_odo_->T_vertex_this() = *T_r_m_odo_;
  point_map_odo_->vertex_id() = vid_odo;

  /// Prepare to save map and scans
  auto vertex = graph->at(vid_odo);
  using PointScanLM =
      storage::LockableMessage<radar::PointScan<radar::PointWithInfo>>;
  using PointMapLM =
      storage::LockableMessage<radar::PointMap<radar::PointWithInfo>>;

  // save the point cloud map (including a copy of the point cloud)
  auto point_map_odo =
      std::make_shared<radar::PointMap<radar::PointWithInfo>>(*point_map_odo_);
  auto map_msg = std::make_shared<PointMapLM>(point_map_odo, *qdata->stamp);
  vertex->insert<radar::PointMap<radar::PointWithInfo>>(
      "radar_point_map", "vtr_radar_msgs/msg/PointMap", map_msg);
  auto map_copy_msg =
      std::make_shared<PointMapLM>(point_map_odo, *qdata->stamp);
  vertex->insert<radar::PointMap<radar::PointWithInfo>>(
      "radar_point_map_v" + std::to_string(point_map_odo->version()),
      "vtr_radar_msgs/msg/PointMap", map_copy_msg);

  // Save the accumulated radar scans (need to correct transform before saving)
#if false
  // raw scans
  for (auto it = new_raw_scan_odo_.begin(); it != new_raw_scan_odo_.end();
       it++) {
    // correct transform to the vid odo - this is essentially:
    // T_<vid odo>_<scan> = T_<vid odo>_<last vid odo> * T_<last vid odo>_<scan>
    it->second->T_vertex_this() =
        point_map_odo_->T_vertex_this() * it->second->T_vertex_this();
    it->second->vertex_id() = vid_odo;
    // save the point scan
    auto scan_msg = std::make_shared<PointScanLM>(it->second, it->first);
    vertex->insert<radar::PointScan<radar::PointWithInfo>>("radar_raw_point_scan", scan_msg);
  }
  new_raw_scan_odo_.clear();
#endif
  // down-sampled scans
  for (auto it = new_scan_odo_.begin(); it != new_scan_odo_.end(); it++) {
    // correct transform to the vid odo - this is essentially:
    // T_<vid odo>_<scan> = T_<vid odo>_<last vid odo> * T_<last vid odo>_<scan>
    it->second->T_vertex_this() =
        point_map_odo_->T_vertex_this() * it->second->T_vertex_this();
    it->second->vertex_id() = vid_odo;
    // save the point scan
    auto scan_msg = std::make_shared<PointScanLM>(it->second, it->first);
    vertex->insert<radar::PointScan<radar::PointWithInfo>>(
        "radar_point_scan", "vtr_radar_msgs/msg/PointScan", scan_msg);
  }
  new_scan_odo_.clear();
}

}  // namespace radar_lidar
}  // namespace vtr
