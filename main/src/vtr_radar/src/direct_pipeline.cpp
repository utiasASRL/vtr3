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
#include "vtr_radar/direct_pipeline.hpp"

#include "vtr_radar/data_types/pointmap_pointer.hpp"
#include "vtr_tactic/modules/factory.hpp"

namespace vtr {
namespace radar {

using namespace tactic;

auto RadarDirectPipeline::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                    const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // modules
  if (!node->has_parameter(param_prefix + ".preprocessing"))config->preprocessing = node->declare_parameter<std::vector<std::string>>(param_prefix + ".preprocessing", config->preprocessing);
  else node->get_parameter(param_prefix + ".preprocessing", config->preprocessing);
  if (!node->has_parameter(param_prefix + ".odometry")) config->odometry = node->declare_parameter<std::vector<std::string>>(param_prefix + ".odometry", config->odometry);
  else node->get_parameter(param_prefix + ".odometry", config->odometry);
  if (!node->has_parameter(param_prefix + ".localization")) config->localization = node->declare_parameter<std::vector<std::string>>(param_prefix + ".localization", config->localization);
  else node->get_parameter(param_prefix + ".localization", config->localization);
  // submap creation thresholds
  config->submap_translation_threshold = node->declare_parameter<double>(param_prefix + ".submap_translation_threshold", config->submap_translation_threshold);
  config->submap_rotation_threshold = node->declare_parameter<double>(param_prefix + ".submap_rotation_threshold", config->submap_rotation_threshold);
  
  // clang-format on
  return config;
}

RadarDirectPipeline::RadarDirectPipeline(
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

OutputCache::Ptr RadarDirectPipeline::createOutputCache() const {
  return std::make_shared<RadarOutputCache>();
}

void RadarDirectPipeline::reset() {
  // reset modules
  for (const auto &module : preprocessing_) module->reset();
  for (const auto &module : odometry_) module->reset();
  for (const auto &module : localization_) module->reset();
  T_v_odo_submap_v_ = tactic::EdgeTransform(true);
  
}

void RadarDirectPipeline::preprocess_(const QueryCache::Ptr &qdata0,
                                const OutputCache::Ptr &output0,
                                const Graph::Ptr &graph,
                                const TaskExecutor::Ptr &executor) {

  for (const auto &module : preprocessing_)
    module->run(*qdata0, *output0, graph, executor);
}

void RadarDirectPipeline::runOdometry_(const QueryCache::Ptr &qdata0,
                                 const OutputCache::Ptr &output0,
                                 const Graph::Ptr &graph,
                                 const TaskExecutor::Ptr &executor) {
  auto qdata = std::dynamic_pointer_cast<RadarQueryCache>(qdata0);

  
  for (const auto &module : odometry_)
    module->run(*qdata0, *output0, graph, executor);

  if (*qdata0->vertex_test_result == VertexTestResult::CREATE_VERTEX) {
    T_v_odo_submap_v_ = *qdata0->T_r_v_odo * T_v_odo_submap_v_;
  }

}

void RadarDirectPipeline::runLocalization_(const QueryCache::Ptr &qdata0,
                                     const OutputCache::Ptr &output0,
                                     const Graph::Ptr &graph,
                                     const TaskExecutor::Ptr &executor) {
  auto qdata = std::dynamic_pointer_cast<RadarQueryCache>(qdata0);
  
  for (const auto &module : localization_)
    module->run(*qdata0, *output0, graph, executor);

}

void RadarDirectPipeline::onVertexCreation_(const QueryCache::Ptr &qdata0,
                                      const OutputCache::Ptr &,
                                      const Graph::Ptr &graph,
                                      const TaskExecutor::Ptr &) {
  const auto qdata = std::dynamic_pointer_cast<RadarQueryCache>(qdata0);
  auto vertex = graph->at(*qdata->vid_odo);
  
  // Right now we save every image as a submap
  cv_bridge::CvImage smoothed_scan_msg;
  smoothed_scan_msg.header.frame_id = "radar";
  smoothed_scan_msg.header.stamp = qdata->scan_msg->b_scan_img.header.stamp;
  smoothed_scan_msg.encoding = "32FC1";
  
  using Image_LockMsg = storage::LockableMessage<ImageMsg>;
  auto locked_image_msg =
          std::make_shared<Image_LockMsg>(smoothed_scan_msg.toImageMsg(), *qdata->stamp);
  vertex->insert<ImageMsg>("smoothed_scan", "sensor_msgs/msg/Image", locked_image_msg);

  // save the sliding map as vertex submap if we have traveled far enough
  const bool create_submap = [&] {
    //
    if (!submap_vid_odo_.isValid()) return true;
    //
    auto T_submap_r_vec = T_v_odo_submap_v_.vec();
    auto dtran = T_submap_r_vec.head<3>().norm();
    auto drot = T_submap_r_vec.tail<3>().norm() * 57.29577;  // 180/pi
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
    auto submap_odo =
        std::make_shared<PointMap<PointWithInfo>>(*qdata->sliding_map_odo);
    // save the submap
    using PointMapLM = storage::LockableMessage<PointMap<PointWithInfo>>;
    auto submap_msg = std::make_shared<PointMapLM>(submap_odo, *qdata->stamp);
    vertex->insert<PointMap<PointWithInfo>>(
        "pointmap", "vtr_radar_msgs/msg/PointMap", submap_msg);
    // save a copy
    auto submap2_msg = std::make_shared<PointMapLM>(submap_odo, *qdata->stamp);
    vertex->insert<PointMap<PointWithInfo>>(
        "pointmap_v" + std::to_string(submap_odo->version()),
        "vtr_radar_msgs/msg/PointMap", submap2_msg);

    // save the submap vertex id and transform
    submap_vid_odo_ = *qdata->vid_odo;
    T_v_odo_submap_v_ = tactic::EdgeTransform(true);
  }

  /// save a pointer to the latest submap
  const auto submap_ptr = std::make_shared<PointMapPointer>();
  submap_ptr->this_vid = *qdata->vid_odo;
  submap_ptr->map_vid = submap_vid_odo_;
  submap_ptr->T_v_this_map = T_v_odo_submap_v_;
  //
  CLOG(DEBUG, "radar.pipeline")
      << "Saving submap pointer from this vertex " << *qdata->vid_odo
      << " to map vertex " << submap_vid_odo_ << " with transform T_v_map_this "
      << submap_ptr->T_v_this_map.inverse().vec().transpose();
  using PointMapPointerLM = storage::LockableMessage<PointMapPointer>;
  auto submap_ptr_msg =
      std::make_shared<PointMapPointerLM>(submap_ptr, *qdata->stamp);
  vertex->insert<PointMapPointer>(
      "pointmap_ptr", "vtr_radar_msgs/msg/PointMapPointer", submap_ptr_msg);
}



}  // namespace radar
}  // namespace vtr
