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
 * \file intra_exp_merging_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/pointmap/intra_exp_merging_module.hpp"

#include "vtr_lidar/data_structures/pointmap.hpp"
#include "vtr_pose_graph/path/pose_cache.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto IntraExpMergingModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                            const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->depth = node->declare_parameter<int>(param_prefix + ".depth", config->depth);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void IntraExpMergingModule::run_(QueryCache &qdata0, OutputCache &,
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

void IntraExpMergingModule::runAsync_(QueryCache &qdata0, OutputCache &,
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
      scan_pub_ = qdata.node->create_publisher<PointCloudMsg>("intra_exp_scan_check", 5);
      // clang-format on
      publisher_initialized_ = true;
    }
  }

  // input
  const auto &target_vid = *qdata.intra_exp_merging_async;

  CLOG(INFO, "lidar.intra_exp_merging")
      << "Intra-Experience Merging for vertex: " << target_vid;
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

  // Store a copy of the original map for visualization
  auto old_map_copy = locked_map_msg.getData();

  // Perform the map update

  // start a new map with the same voxel size
  PointMap<PointWithInfo> updated_map(locked_map_msg.getData().dl());

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

    /// Retrieve point scans from this vertex
    const auto time_range = vertex->timeRange();
    const auto scan_msgs = vertex->retrieve<PointScan<PointWithInfo>>(
        "point_scan", "vtr_lidar_msgs/msg/PointMap", time_range.first,
        time_range.second);

    CLOG(DEBUG, "lidar.intra_exp_merging")
        << "Retrieved scan size assocoated with vertex " << vertex->id()
        << " is: " << scan_msgs.size();

    /// simply return if there's no scan to work on
    if (scan_msgs.empty()) continue;

    for (const auto scan_msg : scan_msgs) {
      auto point_scan = scan_msg->sharedLocked().get().getData();  // COPY!
      const auto &T_v_s = (T_target_curr * point_scan.T_vertex_this()).matrix();
      auto &point_cloud = point_scan.point_cloud();
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
  }
  if (updated_map.size() == 0) {
    std::string err{"The merged map has size 0."};
    CLOG(ERROR, "lidar.intra_exp_merging") << err;
    throw std::runtime_error{err};
  }
  // update transform
  updated_map.T_vertex_this() = tactic::EdgeTransform(true);
  updated_map.vertex_id() = locked_map_msg.getData().vertex_id();
  // update version
  updated_map.version() = PointMap<PointWithInfo>::INTRA_EXP_MERGED;
  // save the updated point map
  locked_map_msg.setData(updated_map);

  // Store a copy of the original map
  using PointMapLM = storage::LockableMessage<PointMap<PointWithInfo>>;
  auto updated_map_copy =
      std::make_shared<PointMap<PointWithInfo>>(updated_map);
  auto updated_map_copy_msg = std::make_shared<PointMapLM>(
      updated_map_copy, locked_map_msg.getTimestamp());
  vertex->insert<PointMap<PointWithInfo>>(
      "point_map_v" + std::to_string(updated_map_copy->version()),
      "vtr_lidar_msgs/msg/PointMap", updated_map_copy_msg);

  /// publish the transformed pointcloud
  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);

    // publish the old map
    {
      auto point_cloud = old_map_copy.point_cloud();  // COPY!
      const auto &T_v_m = old_map_copy.T_vertex_this().matrix();
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