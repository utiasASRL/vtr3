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
 * \file intra_exp_merging_module_v2.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/pointmap/intra_exp_merging_module_v2.hpp"

#include "vtr_lidar/data_structures/pointmap.hpp"
#include "vtr_pose_graph/path/pose_cache.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto IntraExpMergingModuleV2::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // merge
  config->depth = node->declare_parameter<int>(param_prefix + ".depth", config->depth);
  // point map
  config->map_voxel_size = node->declare_parameter<float>(param_prefix + ".map_voxel_size", config->map_voxel_size);
  config->crop_range_front = node->declare_parameter<float>(param_prefix + ".crop_range_front", config->crop_range_front);
  config->back_over_front_ratio = node->declare_parameter<float>(param_prefix + ".back_over_front_ratio", config->back_over_front_ratio);
  // general
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void IntraExpMergingModuleV2::run_(QueryCache &qdata0, OutputCache &,
                                   const Graph::Ptr &,
                                   const TaskExecutor::Ptr &executor) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (qdata.live_id->isValid() &&
      qdata.live_id->minorId() >= (unsigned)config_->depth &&
      *qdata.keyframe_test_result == KeyframeTestResult::CREATE_VERTEX) {
    const auto target_vid =
        VertexId(qdata.live_id->majorId(),
                 qdata.live_id->minorId() - (unsigned)config_->depth);

    auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
    qdata.intra_exp_merging_async.emplace(target_vid);
    executor->dispatch(std::make_shared<Task>(
        shared_from_this(), qdata.shared_from_this(), 0, Task::DepIdSet{},
        Task::DepId{}, "Intra Exp Merging", target_vid));
  }
}

void IntraExpMergingModuleV2::runAsync_(QueryCache &qdata0, OutputCache &,
                                        const Graph::Ptr &graph,
                                        const TaskExecutor::Ptr &,
                                        const Task::Priority &,
                                        const Task::DepId &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!publisher_initialized_) {
      // clang-format off
      old_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("intra_exp_merging_old", 5);
      new_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("intra_exp_merging_new", 5);
      // clang-format on
      publisher_initialized_ = true;
    }
  }

  // input
  const auto &target_vid = *qdata.intra_exp_merging_async;

  CLOG(INFO, "lidar.intra_exp_merging")
      << "Intra-Experience Merging for vertex: " << target_vid;

  // check if intra-exp merging is done already
  {
    auto vertex = graph->at(target_vid);
    const auto map_msg = vertex->retrieve<PointMap<PointWithInfo>>(
        "point_map", "vtr_lidar_msgs/msg/PointMap");
    auto locked_map_msg_ref = map_msg->locked();  // lock the msg
    auto &locked_map_msg = locked_map_msg_ref.get();

    // Check if this map has been updated already
    const auto curr_map_version = locked_map_msg.getData().version();
    if (curr_map_version >= PointMap<PointWithInfo>::INTRA_EXP_MERGED) {
      CLOG(WARNING, "lidar.intra_exp_merging")
          << "Intra-Experience Merging for vertex: " << target_vid
          << " - ALREADY COMPLETED!";
      return;
    }
  }

  // Perform the map update

  // start a new map with the same voxel size
  PointMap<PointWithInfo> updated_map(config_->map_voxel_size);

  // Get the subgraph of interest to work on (thread safe)
  const auto tempeval = std::make_shared<TemporalEvaluator<GraphBase>>(*graph);
  const auto subgraph =
      config_->depth ? graph->getSubgraph(target_vid, config_->depth, tempeval)
                     : graph->getSubgraph(std::vector<VertexId>({target_vid}));

  // cache all the transforms so we only calculate them once
  pose_graph::PoseCache<GraphBase> pose_cache(subgraph, target_vid);

  auto itr = subgraph->begin(target_vid);
  for (; itr != subgraph->end(); itr++) {
    const auto vertex = itr->v();

    // get target vertex to current vertex transformation
    auto T_target_curr = pose_cache.T_root_query(vertex->id());
    CLOG(DEBUG, "lidar.intra_exp_merging")
        << "T_target_curr is " << T_target_curr.vec().transpose();

    /// Retrieve point map from this vertex
    const auto map_msg = vertex->retrieve<PointMap<PointWithInfo>>(
        "point_map", "vtr_lidar_msgs/msg/PointMap");

    auto point_map = map_msg->sharedLocked().get().getData();  // COPY!
    const auto &T_v_s = (T_target_curr * point_map.T_vertex_map()).matrix();
    auto &point_cloud = point_map.point_map();
    // eigen mapping
    auto scan_mat = point_cloud.getMatrixXfMap(
        3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
    auto scan_normal_mat = point_cloud.getMatrixXfMap(
        3, PointWithInfo::size(), PointWithInfo::normal_offset());
    // transform to the local frame of this vertex
    Eigen::Matrix3f C_v_s = (T_v_s.block<3, 3>(0, 0)).cast<float>();
    Eigen::Vector3f r_s_v_in_v = (T_v_s.block<3, 1>(0, 3)).cast<float>();
    scan_mat = (C_v_s * scan_mat).colwise() + r_s_v_in_v;
    scan_normal_mat = C_v_s * scan_normal_mat;
    // store this scan into the updatd map;
    updated_map.update(point_cloud);
  }

  if (updated_map.size() == 0) {
    std::string err{"The merged map has size 0."};
    CLOG(ERROR, "lidar.intra_exp_merging") << err;
    throw std::runtime_error{err};
  }

  // crop box filter
  updated_map.crop(Eigen::Matrix4f::Identity(), config_->crop_range_front,
                   config_->back_over_front_ratio);

  // update transform
  updated_map.T_vertex_map() = tactic::EdgeTransform(true);
  updated_map.vertex_id() = target_vid;
  // update version
  updated_map.version() = PointMap<PointWithInfo>::INTRA_EXP_MERGED;

  /// update the point map of this vertex
  auto vertex = graph->at(target_vid);
  const auto map_msg = vertex->retrieve<PointMap<PointWithInfo>>(
      "point_map", "vtr_lidar_msgs/msg/PointMap");
  auto locked_map_msg_ref = map_msg->locked();  // lock the msg
  auto &locked_map_msg = locked_map_msg_ref.get();

  // store a copy of the original map for visualization
  auto old_map_copy = locked_map_msg.getData();

  // update the map
  locked_map_msg.setData(updated_map);

  // Store a copy of the updated map for debugging
  {
    using PointMapLM = storage::LockableMessage<PointMap<PointWithInfo>>;
    auto updated_map_copy =
        std::make_shared<PointMap<PointWithInfo>>(updated_map);
    auto updated_map_copy_msg = std::make_shared<PointMapLM>(
        updated_map_copy, locked_map_msg.getTimestamp());
    vertex->insert<PointMap<PointWithInfo>>(
        "point_map_v" + std::to_string(updated_map_copy->version()),
        "vtr_lidar_msgs/msg/PointMap", updated_map_copy_msg);
  }

  /// publish the transformed pointcloud
  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);

    // publish the old map
    {
      auto point_cloud = old_map_copy.point_map();  // COPY!
      const auto &T_v_m = old_map_copy.T_vertex_map().matrix();
      // eigen mapping
      auto points_mat = point_cloud.getMatrixXfMap(
          3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
      // transform to the local frame of this vertex
      Eigen::Matrix3f C_v_m = (T_v_m.block<3, 3>(0, 0)).cast<float>();
      Eigen::Vector3f r_m_v_in_v = (T_v_m.block<3, 1>(0, 3)).cast<float>();
      points_mat = (C_v_m * points_mat).colwise() + r_m_v_in_v;

      PointCloudMsg pc2_msg;
      pcl::toROSMsg(point_cloud, pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      old_map_pub_->publish(pc2_msg);
    }

    // publish the updated map (will already be in vertex frame)
    {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(updated_map.point_map(), pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      new_map_pub_->publish(pc2_msg);
    }
  }
  CLOG(INFO, "lidar.intra_exp_merging")
      << "Intra-Experience Merging for vertex: " << target_vid << " - DONE!";
}

}  // namespace lidar
}  // namespace vtr