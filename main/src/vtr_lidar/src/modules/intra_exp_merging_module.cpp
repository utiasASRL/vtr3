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
 * \brief IntraExpMergingModule class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/intra_exp_merging_module.hpp"

#include "vtr_lidar/pointmap/pointmap_v2.hpp"
#include "vtr_pose_graph/path/pose_cache.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

void IntraExpMergingModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->depth = node->declare_parameter<int>(param_prefix + ".depth", config_->depth);
  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void IntraExpMergingModule::runImpl(QueryCache &qdata,
                                    const Graph::ConstPtr &) {
  if (!task_queue_) return;

  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    map_pub_ = qdata.node->create_publisher<PointCloudMsg>("intra_exp_merging", 5);
    scan_pub_ = qdata.node->create_publisher<PointCloudMsg>("intra_exp_scan_check", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  if (qdata.live_id->isValid() &&
      qdata.live_id->minorId() >= (unsigned)config_->depth &&
      *qdata.keyframe_test_result == KeyframeTestResult::CREATE_VERTEX) {
    const auto target_vid =
        VertexId(qdata.live_id->majorId(),
                 qdata.live_id->minorId() - (unsigned)config_->depth);

    task_queue_->dispatch(std::make_shared<Task>(
        shared_from_base<IntraExpMergingModule>(), config_, target_vid));
  }
}

void IntraExpMergingModule::Task::run(const AsyncTaskExecutor::Ptr &,
                                      const Graph::Ptr &graph) {
  CLOG(INFO, "lidar.intra_exp_merging")
      << "Intra-Experience Merging for vertex: " << target_vid_;
  auto vertex = graph->at(target_vid_);
  const auto map_msg = vertex->retrieve<PointMap<PointWithInfo>>("pointmap2");
  auto locked_map_msg_ref = map_msg->locked();  // lock the msg
  auto &locked_map_msg = locked_map_msg_ref.get();

  // Check if this map has been updated already
  const auto curr_map_version = locked_map_msg.getData().version();
  if (curr_map_version >= PointMap<PointWithInfo>::INTRA_EXP_MERGED) {
    CLOG(WARNING, "lidar.intra_exp_merging")
        << "Intra-Experience Merging for vertex: " << target_vid_
        << " - ALREADY COMPLETED!";
    return;
  }

  // Perform the map update

  // start a new map with the same voxel size
  PointMap<PointWithInfo> updated_map(locked_map_msg.getData().dl());

  // Get the subgraph of interest to work on (thread safe)
  using namespace pose_graph;
  const auto tempeval = std::make_shared<TemporalEvaluator<RCGraphBase>>();
  tempeval->setGraph((void *)graph.get());
  const auto subgraph =
      config_->depth ? graph->getSubgraph(target_vid_, config_->depth, tempeval)
                     : graph->getSubgraph(std::vector<VertexId>({target_vid_}));

  // cache all the transforms so we only calculate them once
  PoseCache<RCGraphBase> pose_cache(subgraph, target_vid_);

  auto itr = subgraph->begin(target_vid_);
  for (; itr != subgraph->end(); itr++) {
    const auto vertex = itr->v();

    // get target vertex to current vertex transformation
    auto T_target_curr = pose_cache.T_root_query(vertex->id());
    CLOG(DEBUG, "lidar.intra_exp_merging")
        << "T_target_curr is " << T_target_curr.vec().transpose();

    /// Retrieve point scans from this vertex
    const auto time_range = vertex->timeRange();
    const auto scan_msgs = vertex->retrieve<PointScan<PointWithInfo>>(
        "pointscan", time_range.first, time_range.second);

    CLOG(DEBUG, "lidar.intra_exp_merging")
        << "Retrieved scan size assocoated with vertex " << vertex->id()
        << " is: " << scan_msgs.size();

    /// simply return if there's no scan to work on
    if (scan_msgs.empty()) continue;

    for (const auto scan_msg : scan_msgs) {
      auto point_scan = scan_msg->sharedLocked().get().getData();  // COPY!
      const auto &T_v_s = (T_target_curr * point_scan.T_vertex_map()).matrix();
      auto &point_cloud = point_scan.point_map();
      // eigen mapping
      auto scan_mat = point_cloud.getMatrixXfMap(3, 16, 0);
      auto scan_normal_mat = point_cloud.getMatrixXfMap(3, 16, 4);
      // transform to the local frame of this vertex
      Eigen::Matrix3f R_tot = (T_v_s.block<3, 3>(0, 0)).cast<float>();
      Eigen::Vector3f T_tot = (T_v_s.block<3, 1>(0, 3)).cast<float>();
      scan_mat = (R_tot * scan_mat).colwise() + T_tot;
      scan_normal_mat = R_tot * scan_normal_mat;
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
  updated_map.T_vertex_map() = PointMap<PointWithInfo>::TransformType(true);
  updated_map.vertex_id() = locked_map_msg.getData().vertex_id();
  // update version
  updated_map.version() = PointMap<PointWithInfo>::INTRA_EXP_MERGED;
  // save the updated point map
  locked_map_msg.setData(updated_map);

  /// publish the transformed pointcloud
  auto mdl = module_.lock();
  if (mdl && config_->visualize) {
    std::unique_lock<std::mutex> lock(mdl->mutex_);

    PointCloudMsg pc2_msg;
    pcl::toROSMsg(updated_map.point_map(), pc2_msg);
    pc2_msg.header.frame_id = "world";
    // pc2_msg.header.stamp = 0;
    mdl->map_pub_->publish(pc2_msg);
  }
  CLOG(INFO, "lidar.intra_exp_merging")
      << "Intra-Experience Merging for vertex: " << target_vid_ << " - DONE!";
}

}  // namespace lidar
}  // namespace vtr