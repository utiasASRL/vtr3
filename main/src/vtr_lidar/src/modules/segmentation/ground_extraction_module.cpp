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
 * \file ground_extraction_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 * \brief GroundExtractionModule class methods definition
 */
#include "vtr_lidar/modules/segmentation/ground_extraction_module.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto GroundExtractionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // himmelsbach
  config->z_offset = node->declare_parameter<float>(param_prefix + ".z_offset", config->z_offset);
  config->alpha = node->declare_parameter<float>(param_prefix + ".alpha", config->alpha);
  config->tolerance = node->declare_parameter<float>(param_prefix + ".tolerance", config->tolerance);
  config->Tm = node->declare_parameter<float>(param_prefix + ".Tm", config->Tm);
  config->Tm_small = node->declare_parameter<float>(param_prefix + ".Tm_small", config->Tm_small);
  config->Tb = node->declare_parameter<float>(param_prefix + ".Tb", config->Tb);
  config->Trmse = node->declare_parameter<float>(param_prefix + ".Trmse", config->Trmse);
  config->Tdprev = node->declare_parameter<float>(param_prefix + ".Tdprev", config->Tdprev);
  config->rmin = node->declare_parameter<float>(param_prefix + ".rmin", config->rmin);
  config->num_bins_small = (size_t)node->declare_parameter<int>(param_prefix + ".num_bins_small", config->num_bins_small);
  config->bin_size_small = node->declare_parameter<float>(param_prefix + ".bin_size_small", config->bin_size_small);
  config->num_bins_large = (size_t)node->declare_parameter<int>(param_prefix + ".num_bins_large", config->num_bins_large);
  config->bin_size_large = node->declare_parameter<float>(param_prefix + ".bin_size_large", config->bin_size_large);

  config->run_async = node->declare_parameter<bool>(param_prefix + ".run_async", config->run_async);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

GroundExtractionModule::GroundExtractionModule(
    const Config::ConstPtr &config,
    const std::shared_ptr<tactic::ModuleFactory> &module_factory,
    const std::string &name)
    : tactic::BaseModule{module_factory, name}, config_(config) {
  // configure himmelsbach
  himmelsbach_.z_offset = config_->z_offset;
  himmelsbach_.alpha = config_->alpha;
  himmelsbach_.tolerance = config_->tolerance;
  himmelsbach_.Tm = config_->Tm;
  himmelsbach_.Tm_small = config_->Tm_small;
  himmelsbach_.Tb = config_->Tb;
  himmelsbach_.Trmse = config_->Trmse;
  himmelsbach_.Tdprev = config_->Tdprev;
  himmelsbach_.rmin = config_->rmin;
  himmelsbach_.num_bins_small = config_->num_bins_small;
  himmelsbach_.bin_size_small = config_->bin_size_small;
  himmelsbach_.num_bins_large = config_->num_bins_large;
  himmelsbach_.bin_size_large = config_->bin_size_large;
}

void GroundExtractionModule::runImpl(QueryCache &qdata0, OutputCache &output0,
                                     const Graph::Ptr &graph,
                                     const TaskExecutor::Ptr &executor) {
  if (config_->run_async)
    executor->dispatch(
        std::make_shared<Task>(shared_from_this(), qdata0.shared_from_this()));
  else
    runAsyncImpl(qdata0, output0, graph, executor, Task::Priority(-1),
                 Task::DepId());
}

void GroundExtractionModule::runAsyncImpl(
    QueryCache &qdata0, OutputCache &output0, const Graph::Ptr &graph,
    const TaskExecutor::Ptr &, const Task::Priority &, const Task::DepId &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!publisher_initialized_) {
      // clang-format off
      map_pub_ = qdata.node->create_publisher<PointCloudMsg>("ground_extraction", 5);
      // clang-format on
      publisher_initialized_ = true;
    }
  }

  // input
  const auto &target_vid = *qdata.ground_extraction_async;

  // retrieve the local map
  CLOG(INFO, "lidar.ground_extraction")
      << "Ground Extraction for vertex: " << target_vid;
  auto vertex = graph->at(target_vid);
  const auto map_msg = vertex->retrieve<PointMap<PointWithInfo>>(
      "point_map", "vtr_lidar_msgs/msg/PointMap");
  auto locked_map_msg_ref = map_msg->locked();  // lock the msg
  auto &locked_map_msg = locked_map_msg_ref.get();

  // get a copy of the map
  const auto &map = locked_map_msg.getData();

  // convert to vertex frame
  auto point_cloud = map.point_map();  // copy
  const auto &T_v_m = map.T_vertex_map().matrix();
  // eigen mapping
  // clang-format off
  auto points_mat = point_cloud.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto normal_mat = point_cloud.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::normal_offset());
  // clang-format on
  // transform to the local frame of this vertex
  Eigen::Matrix3f R_tot = (T_v_m.block<3, 3>(0, 0)).cast<float>();
  Eigen::Vector3f T_tot = (T_v_m.block<3, 1>(0, 3)).cast<float>();
  points_mat = ((R_tot * points_mat).colwise() + T_tot).eval();
  normal_mat = (R_tot * normal_mat).eval();

  // ground extraction
  const auto ground_idx = himmelsbach_(point_cloud);

  for (auto &&point : point_cloud) point.flex11 = 0;
  for (const auto &idx : ground_idx) point_cloud[idx].flex11 = 1;

  PointCloudMsg pc2_msg;
  pcl::toROSMsg(point_cloud, pc2_msg);
  pc2_msg.header.frame_id = "world";
  // pc2_msg.header.stamp = 0;
  map_pub_->publish(pc2_msg);

  CLOG(INFO, "lidar.ground_extraction")
      << "Ground Extraction for vertex: " << target_vid << " - DONE!";
}

}  // namespace lidar
}  // namespace vtr