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

#include "vtr_lidar/data_types/pointmap.hpp"
#include "vtr_lidar/data_types/pointmap_pointer.hpp"
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
  config->depth = node->declare_parameter<double>(param_prefix + ".depth", config->depth);
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

  if (qdata.vid_odo->isValid() &&
      qdata.vid_odo->minorId() >= (unsigned)config_->depth &&
      *qdata.vertex_test_result == VertexTestResult::CREATE_VERTEX) {
    const auto target_vid =
        VertexId(qdata.vid_odo->majorId(),
                 qdata.vid_odo->minorId() - (unsigned)config_->depth);

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

  /// input
  const auto &target_vid = *qdata.intra_exp_merging_async;
  const auto target_vertex = graph->at(target_vid);

  CLOG(INFO, "lidar.intra_exp_merging")
      << "Intra-Experience Merging for vertex: " << target_vid;

  /// check if we have a map for this vertex
  {
    const auto msg = target_vertex->retrieve<PointMapPointer>(
        "pointmap_ptr", "vtr_lidar_msgs/msg/PointMapPointer");
    auto locked_msg = msg->sharedLocked();
    const auto &pointmap_ptr = locked_msg.get().getData();
    //
    if (pointmap_ptr.map_vid != target_vid) {
      CLOG(INFO, "lidar.intra_exp_merging")
          << "This vertex does not have an associated submap, skipped.";
      return;
    }
  }

  /// load the map and check if it is already merged
  {
    const auto map_msg = target_vertex->retrieve<PointMap<PointWithInfo>>(
        "pointmap", "vtr_lidar_msgs/msg/PointMap");
    auto locked_map_msg_ref = map_msg->sharedLocked();  // lock the msg
    auto &locked_map_msg = locked_map_msg_ref.get();

    // check if this map has been updated already
    const auto curr_map_version = locked_map_msg.getData().version();
    if (curr_map_version >= PointMap<PointWithInfo>::INTRA_EXP_MERGED) {
      CLOG(WARNING, "lidar.intra_exp_merging")
          << "Intra-Experience Merging for vertex: " << target_vid
          << " - ALREADY COMPLETED!";
      return;
    }
  }

  /// Perform the map update

  // start a new map with the same voxel size
  PointMap<PointWithInfo> updated_map(config_->map_voxel_size);

  // get the subgraph of interest to work on (thread safe)
  const auto tempeval = std::make_shared<TemporalEvaluator<GraphBase>>(*graph);
  const auto disteval = std::make_shared<DistanceEvaluator<GraphBase>>(*graph);
  const auto subgraph = graph->dijkstraTraverseToDepth(
      target_vid, config_->depth, disteval, tempeval);

  // cache all the transforms so we only calculate them once
  pose_graph::PoseCache<GraphBase> pose_cache(subgraph, target_vid);

  size_t num_map_merged = 0;
  auto itr = subgraph->begin(target_vid);
  for (; itr != subgraph->end(); itr++) {
    //
    const auto vertex = itr->v();

    // check if this vertex has a map
    {
      const auto msg = vertex->retrieve<PointMapPointer>(
          "pointmap_ptr", "vtr_lidar_msgs/msg/PointMapPointer");
      auto locked_msg = msg->sharedLocked();
      const auto &pointmap_ptr = locked_msg.get().getData();
      if (pointmap_ptr.map_vid != vertex->id()) continue;
    }

    // get target vertex to current vertex transformation
    auto T_target_curr = pose_cache.T_root_query(vertex->id());
    CLOG(DEBUG, "lidar.intra_exp_merging")
        << "T_target_curr is " << T_target_curr.vec().transpose();

    // retrieve point map v0 (initial map) from this vertex
    const auto map_msg = vertex->retrieve<PointMap<PointWithInfo>>(
        "pointmap_v0", "vtr_lidar_msgs/msg/PointMap");

    auto pointmap = map_msg->sharedLocked().get().getData();
    const auto &T_v_m = (T_target_curr * pointmap.T_vertex_this()).matrix();
    auto &point_cloud = pointmap.point_cloud();

    // transform to the local frame of this vertex
    auto scan_mat = point_cloud.getMatrixXfMap(
        4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
    auto scan_normal_mat = point_cloud.getMatrixXfMap(
        4, PointWithInfo::size(), PointWithInfo::normal_offset());
    scan_mat = T_v_m.cast<float>() * scan_mat;
    scan_normal_mat = T_v_m.cast<float>() * scan_normal_mat;

    // store this scan into the updatd map;
    updated_map.update(point_cloud);

    //
    ++num_map_merged;
  }
  CLOG(DEBUG, "lidar.intra_exp_merging")
      << "Number of map merged: " << num_map_merged;

  // sanity check
  if (updated_map.size() == 0) {
    std::string err{"The merged map has size 0."};
    CLOG(ERROR, "lidar.intra_exp_merging") << err;
    throw std::runtime_error{err};
  }

  // crop box filter
  /// \todo double check correctness
  auto crop_filter_cb =
      [&range = config_->crop_range_front,
       &ratio = config_->back_over_front_ratio](PointWithInfo &query_pt) {
        const float rho = query_pt.getVector3fMap().norm();
        const float phi = std::atan2(query_pt.y, query_pt.x);
        /// \note assuming x is front, y is left, z is up
        float ratio_w_phi = ratio + (1 - std::abs(phi) / M_PI) * (1 - ratio);
        return bool(rho <= range * ratio_w_phi);
      };
  updated_map.filter(crop_filter_cb);

  // update transform
  updated_map.T_vertex_this() = tactic::EdgeTransform(true);
  updated_map.vertex_id() = target_vid;
  // update version
  updated_map.version() = PointMap<PointWithInfo>::INTRA_EXP_MERGED;

  // update the point map of this vertex
  {
    const auto map_msg = target_vertex->retrieve<PointMap<PointWithInfo>>(
        "pointmap", "vtr_lidar_msgs/msg/PointMap");
    auto locked_map_msg_ref = map_msg->locked();  // lock the msg
    auto &locked_map_msg = locked_map_msg_ref.get();
    locked_map_msg.setData(updated_map);
  }

  // store a copy of the updated map for debugging
  {
    using PointMapLM = storage::LockableMessage<PointMap<PointWithInfo>>;
    auto updated_map_copy =
        std::make_shared<PointMap<PointWithInfo>>(updated_map);
    auto updated_map_copy_msg = std::make_shared<PointMapLM>(
        updated_map_copy, target_vertex->vertexTime());
    target_vertex->insert<PointMap<PointWithInfo>>(
        "pointmap_v" + std::to_string(updated_map_copy->version()),
        "vtr_lidar_msgs/msg/PointMap", updated_map_copy_msg);
  }

  /// publish the transformed pointcloud
  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);

    // publish the old map
    {
      // load old map for reference
      const auto map_msg = target_vertex->retrieve<PointMap<PointWithInfo>>(
          "pointmap_v0", "vtr_lidar_msgs/msg/PointMap");
      auto locked_map_msg_ref = map_msg->sharedLocked();  // lock the msg
      auto &locked_map_msg = locked_map_msg_ref.get();
      auto pointmap = locked_map_msg.getData();

      auto point_cloud = pointmap.point_cloud();  // COPY!
      const auto &T_v_m = pointmap.T_vertex_this().matrix();

      // transform to the local frame of this vertex
      auto points_mat = point_cloud.getMatrixXfMap(
          4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
      points_mat = T_v_m.cast<float>() * points_mat;

      PointCloudMsg pc2_msg;
      pcl::toROSMsg(point_cloud, pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      old_map_pub_->publish(pc2_msg);
    }

    // publish the updated map (will already be in vertex frame)
    {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(updated_map.point_cloud(), pc2_msg);
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