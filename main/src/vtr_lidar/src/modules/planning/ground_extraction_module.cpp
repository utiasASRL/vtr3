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
 */
#include "vtr_lidar/modules/planning/ground_extraction_module.hpp"

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
  // occupancy grid
  config->resolution = node->declare_parameter<float>(param_prefix + ".resolution", config->resolution);
  config->size_x = node->declare_parameter<float>(param_prefix + ".size_x", config->size_x);
  config->size_y = node->declare_parameter<float>(param_prefix + ".size_y", config->size_y);

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

void GroundExtractionModule::run_(QueryCache &qdata0, OutputCache &output0,
                                  const Graph::Ptr &graph,
                                  const TaskExecutor::Ptr &executor) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  // auto &output = dynamic_cast<LidarOutputCache &>(output0);

  const auto &map_id = *qdata.map_id;
  if (qdata.curr_map_loc_changed && (!(*qdata.curr_map_loc_changed))) {
    CLOG(DEBUG, "lidar.ground_extraction")
        << "Ground extraction for vertex " << map_id << " is already done.";
    return;
  }

  if (config_->run_async)
    executor->dispatch(std::make_shared<Task>(
        shared_from_this(), qdata.shared_from_this(), 0, Task::DepIdSet{},
        Task::DepId{}, "Ground Extraction", map_id));
  else
    runAsync_(qdata0, output0, graph, executor, Task::Priority(-1),
              Task::DepId());
}

void GroundExtractionModule::runAsync_(QueryCache &qdata0, OutputCache &output0,
                                       const Graph::Ptr &,
                                       const TaskExecutor::Ptr &,
                                       const Task::Priority &,
                                       const Task::DepId &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  auto &output = dynamic_cast<LidarOutputCache &>(output0);

  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!publisher_initialized_) {
      // clang-format off
      tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(*qdata.node);
      map_pub_ = qdata.node->create_publisher<PointCloudMsg>("ground_extraction", 5);
      ogm_pub_ = qdata.node->create_publisher<OccupancyGridMsg>("ground_extraction_ogm", 5);
      // clang-format on
      publisher_initialized_ = true;
    }
  }

  if (output.chain.valid() && qdata.map_sid.valid() &&
      output.chain->trunkSequenceId() != *qdata.map_sid) {
    CLOG(INFO, "lidar.ground_extraction")
        << "Trunk id has changed, skip change detection for this scan";
    return;
  }

  // input
  const auto &loc_vid = *qdata.map_id;
  const auto &loc_sid = *qdata.map_sid;
  const auto &point_map = *qdata.curr_map_loc;
  const auto &T_lv_pm = point_map.T_vertex_map().matrix();
  auto point_cloud = point_map.point_map();  // copy for changing

  CLOG(INFO, "lidar.ground_extraction")
      << "Ground Extraction for vertex: " << loc_vid;

  // transform into vertex frame
  // clang-format off
  auto points_mat = point_cloud.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto normal_mat = point_cloud.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::normal_offset());
  // clang-format on
  Eigen::Matrix3f C_lv_pm = (T_lv_pm.block<3, 3>(0, 0)).cast<float>();
  Eigen::Vector3f r_lv_pm = (T_lv_pm.block<3, 1>(0, 3)).cast<float>();
  points_mat = ((C_lv_pm * points_mat).colwise() + r_lv_pm).eval();
  normal_mat = (C_lv_pm * normal_mat).eval();

  // ground extraction
  const auto ground_idx = himmelsbach_(point_cloud);

  // construct occupancy grid map
  std::vector<PointWithInfo> ground_points;
  ground_points.reserve(ground_idx.size());
  std::vector<float> scores;
  scores.reserve(ground_idx.size());
  for (const auto &idx : ground_idx) {
    ground_points.emplace_back(point_cloud[idx]);
    /// \note use a range of 0.05 to 1.0 for visualization
    /// normal of 1 maps to 0.05, 0.95- maps to 1.0
    scores.emplace_back(
        0.05 +
        0.95 * std::clamp((20.0 * (1.0 - point_cloud[idx].normal_z)), 0., 1.));
  }
  // project to 2d and construct the grid map
  const auto ogm = std::make_shared<SparseCostMap>(
      config_->resolution, config_->size_x, config_->size_y);
  ogm->update(ground_points, scores);  /// \todo currently just average scores
  ogm->T_vertex_this() = tactic::EdgeTransform(true);  // already transformed
  ogm->vertex_id() = loc_vid;
  ogm->vertex_sid() = loc_sid;

  // convert to dense for smoothing
  const auto dense_ogm = std::make_shared<DenseCostMap>(ogm->toDense());

  /// publish the transformed pointcloud
  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);
    //
    const auto T_w_lv = (output.chain.valid() && qdata.map_sid.valid())
                            ? output.chain->pose(*qdata.map_sid)
                            : EdgeTransform(true);  // offline

    // publish the occupancy grid origin
    Eigen::Affine3d T(T_w_lv.matrix());
    auto msg = tf2::eigenToTransform(T);
    msg.header.frame_id = "world (offset)";
    // msg.header.stamp = rclcpp::Time(*qdata.stamp);
    msg.child_frame_id = "ground extraction";
    tf_bc_->sendTransform(msg);

    if (!(output.chain.valid() && qdata.map_sid.valid())) {
      // color the points for visualization
      for (auto &&point : point_cloud) point.flex11 = 0;
      for (const auto &idx : ground_idx) point_cloud[idx].flex11 = 1;
      // publish the transformed map (now in vertex frame)
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(point_cloud, pc2_msg);
      pc2_msg.header.frame_id = "world (offset)";
      // pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
      map_pub_->publish(pc2_msg);
    }

    // publish the occupancy grid
    auto grid_msg = dense_ogm->toStorable();
    grid_msg.header.frame_id = "ground extraction";
    // grid_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    ogm_pub_->publish(grid_msg);
  }

  /// output
  auto ground_extraction_ogm_ref = output.ground_extraction_ogm.locked();
  auto &ground_extraction_ogm = ground_extraction_ogm_ref.get();
  ground_extraction_ogm = dense_ogm;

  CLOG(INFO, "lidar.ground_extraction")
      << "Ground Extraction for vertex: " << loc_vid << " - DONE!";
}

}  // namespace lidar
}  // namespace vtr