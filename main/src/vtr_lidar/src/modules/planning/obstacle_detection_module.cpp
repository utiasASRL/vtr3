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
 * \file obstacle_detection_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/planning/obstacle_detection_module.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto ObstacleDetectionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // himmelsbach
  config->z_min = node->declare_parameter<float>(param_prefix + ".z_min", config->z_min);
  config->z_max = node->declare_parameter<float>(param_prefix + ".z_max", config->z_max);
  // occupancy grid
  config->resolution = node->declare_parameter<float>(param_prefix + ".resolution", config->resolution);
  config->size_x = node->declare_parameter<float>(param_prefix + ".size_x", config->size_x);
  config->size_y = node->declare_parameter<float>(param_prefix + ".size_y", config->size_y);

  config->run_async = node->declare_parameter<bool>(param_prefix + ".run_async", config->run_async);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

ObstacleDetectionModule::ObstacleDetectionModule(
    const Config::ConstPtr &config,
    const std::shared_ptr<tactic::ModuleFactory> &module_factory,
    const std::string &name)
    : tactic::BaseModule{module_factory, name}, config_(config) {}

void ObstacleDetectionModule::run_(QueryCache &qdata0, OutputCache &output0,
                                   const Graph::Ptr &graph,
                                   const TaskExecutor::Ptr &executor) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  // auto &output = dynamic_cast<LidarOutputCache &>(output0);

  const auto &map_id = *qdata.map_id;
  if (qdata.curr_map_loc_changed && (!(*qdata.curr_map_loc_changed))) {
    CLOG(DEBUG, "lidar.obstacle_detection")
        << "Obstacle detection for vertex " << map_id << " is already done.";
    return;
  }

  if (config_->run_async)
    executor->dispatch(std::make_shared<Task>(
        shared_from_this(), qdata.shared_from_this(), 0, Task::DepIdSet{},
        Task::DepId{}, "Obstacle Detection", map_id));
  else
    runAsync_(qdata0, output0, graph, executor, Task::Priority(-1),
              Task::DepId());
}

void ObstacleDetectionModule::runAsync_(
    QueryCache &qdata0, OutputCache &output0, const Graph::Ptr &,
    const TaskExecutor::Ptr &, const Task::Priority &, const Task::DepId &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  auto &output = dynamic_cast<LidarOutputCache &>(output0);

  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!publisher_initialized_) {
      // clang-format off
      tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(*qdata.node);
      map_pub_ = qdata.node->create_publisher<PointCloudMsg>("obstacle_detection", 5);
      ogm_pub_ = qdata.node->create_publisher<OccupancyGridMsg>("obstacle_detection_costmap", 5);
      // clang-format on
      publisher_initialized_ = true;
    }
  }

  if (output.chain.valid() && qdata.map_sid.valid() &&
      output.chain->trunkSequenceId() != *qdata.map_sid) {
    CLOG(INFO, "lidar.obstacle_detection")
        << "Trunk id has changed, skip change detection for this scan";
    return;
  }

  // input
  const auto &loc_vid = *qdata.map_id;
  const auto &point_map = *qdata.curr_map_loc;
  const auto &T_lv_pm = point_map.T_vertex_map().matrix();
  auto point_cloud = point_map.point_map();  // copy for changing

  CLOG(INFO, "lidar.obstacle_detection")
      << "Obstacle Detection for vertex: " << loc_vid;

  // transform into vertex frame
  // clang-format off
  auto points_mat = point_cloud.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto normal_mat = point_cloud.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::normal_offset());
  // clang-format on
  Eigen::Matrix3f C_lv_pm = (T_lv_pm.block<3, 3>(0, 0)).cast<float>();
  Eigen::Vector3f r_lv_pm = (T_lv_pm.block<3, 1>(0, 3)).cast<float>();
  points_mat = ((C_lv_pm * points_mat).colwise() + r_lv_pm).eval();
  normal_mat = (C_lv_pm * normal_mat).eval();

  // obstacle detection
  std::vector<size_t> obstacle_idx;
  obstacle_idx.reserve(point_cloud.size());
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    if (point_cloud[i].z > config_->z_min && point_cloud[i].z < config_->z_max)
      obstacle_idx.emplace_back(i);
  }

  // construct occupancy grid map
  std::vector<PointWithInfo> obstacle_points;
  obstacle_points.reserve(obstacle_idx.size());
  std::vector<float> scores;
  scores.reserve(obstacle_idx.size());
  for (const auto &idx : obstacle_idx) {
    obstacle_points.emplace_back(point_cloud[idx]);
    scores.emplace_back(1.0f);
  }
  // project to 2d and construct the grid map
  const auto ogm = std::make_shared<SparseCostMap>(
      config_->resolution, config_->size_x, config_->size_y);
  ogm->update(obstacle_points, scores);  /// \todo currently just average scores
  ogm->T_vertex_this() = tactic::EdgeTransform(true);  // already transformed
  ogm->vertex_id() = loc_vid;

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
    msg.child_frame_id = "obstacle detection";
    tf_bc_->sendTransform(msg);

    if (!(output.chain.valid() && qdata.map_sid.valid())) {
      // color the points for visualization
      for (auto &&point : point_cloud) point.flex11 = 0;
      for (const auto &idx : obstacle_idx) point_cloud[idx].flex11 = 1;
      // publish the transformed map (now in vertex frame)
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(point_cloud, pc2_msg);
      pc2_msg.header.frame_id = "world (offset)";
      // pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
      map_pub_->publish(pc2_msg);
    }

    // publish the occupancy grid
    auto grid_msg = ogm->toCostMapMsg();
    grid_msg.header.frame_id = "obstacle detection";
    // grid_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    ogm_pub_->publish(grid_msg);
  }

  /// output
  auto obstacle_detection_costmap_ref =
      output.obstacle_detection_costmap.locked();
  auto &obstacle_detection_costmap = obstacle_detection_costmap_ref.get();
  obstacle_detection_costmap = ogm;

  CLOG(INFO, "lidar.obstacle_detection")
      << "Obstacle Detection for vertex: " << loc_vid << " - DONE!";
}

}  // namespace lidar
}  // namespace vtr