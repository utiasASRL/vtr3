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
 * \file dynamic_detection_module.cpp
 * \brief DynamicDetectionModule class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/dynamic_detection_module.hpp"
#include "vtr_lidar/modules/intra_exp_merging_module.hpp"
#include "vtr_lidar/pointmap/pointmap_v2.hpp"
#include "vtr_lidar/ray_tracing/dynamic_objects.hpp"
#include "vtr_pose_graph/path/pose_cache.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

void DynamicDetectionModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                           const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->depth = node->declare_parameter<int>(param_prefix + ".depth", config_->depth);

  config_->horizontal_resolution = node->declare_parameter<float>(param_prefix + ".horizontal_resolution", config_->horizontal_resolution);
  config_->vertical_resolution = node->declare_parameter<float>(param_prefix + ".vertical_resolution", config_->vertical_resolution);
  config_->max_num_observations = node->declare_parameter<int>(param_prefix + ".max_num_observations", config_->max_num_observations);
  config_->min_num_observations = node->declare_parameter<int>(param_prefix + ".min_num_observations", config_->min_num_observations);
  config_->dynamic_threshold = node->declare_parameter<float>(param_prefix + ".dynamic_threshold", config_->dynamic_threshold);

  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void DynamicDetectionModule::runImpl(QueryCache &qdata,
                                     const Graph::ConstPtr &) {
  if (!task_queue_) return;

  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    old_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("dynamic_detection_old", 5);
    new_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("dynamic_detection_new", 5);
    scan_pub_ = qdata.node->create_publisher<PointCloudMsg>("dynamic_detection_tmp", 5);
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
        shared_from_base<DynamicDetectionModule>(), config_, target_vid));
  }
}

void DynamicDetectionModule::Task::run(const AsyncTaskExecutor::Ptr &executor,
                                       const Graph::Ptr &graph) {
  CLOG(INFO, "lidar.dynamic_detection")
      << "Short-Term Dynamics Detection for vertex: " << target_vid_;
  auto vertex = graph->at(target_vid_);
  const auto map_msg = vertex->retrieve<PointMap<PointWithInfo>>(
      "point_map", "vtr_lidar_msgs/msg/PointMap");
  auto locked_map_msg_ref = map_msg->locked();  // lock the msg
  auto &locked_map_msg = locked_map_msg_ref.get();

  // Check if this map has been updated already
  const auto curr_map_version = locked_map_msg.getData().version();
  if (curr_map_version >= PointMap<PointWithInfo>::DYNAMIC_REMOVED) {
    CLOG(WARNING, "lidar.dynamic_detection")
        << "Short-Term Dynamics Detection for vertex: " << target_vid_
        << " - ALREADY COMPLETED!";
    return;
  }
  // Check if there's dependency not met
  else if (curr_map_version < PointMap<PointWithInfo>::INTRA_EXP_MERGED) {
    const auto config = std::make_shared<IntraExpMergingModule::Config>();
    config->visualize = false;
    config->depth = config_->depth;
    executor->tryDispatch(std::make_shared<IntraExpMergingModule::Task>(
        nullptr, config, target_vid_, priority + 1));
    executor->tryDispatch(shared_from_this());
    CLOG(WARNING, "lidar.dynamic_detection")
        << "Short-Term Dynamics Detection for vertex: " << target_vid_
        << " - REASSIGNED!";
    return;
  }

  // Store a copy of the original map for visualization
  auto old_map_copy = locked_map_msg.getData();

  // Perform the map update

  // get a copy of the current map for updating
  auto point_map = locked_map_msg.getData();
  // zero out the part of data we will be overwriting
  // this sets Flexible4D to zero (see types.hpp)
  point_map.point_map()
      .getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::flex1_offset())
      .setZero();

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
    CLOG(DEBUG, "lidar.dynamic_detection")
        << "T_target_curr is " << T_target_curr.vec().transpose();

    /// Retrieve point scans from this vertex
    const auto time_range = vertex->timeRange();
#if true
    const auto scan_msgs = vertex->retrieve<PointScan<PointWithInfo>>(
        "point_scan", "vtr_lidar_msgs/msg/PointScan", time_range.first,
        time_range.second);
#else  /// store raw point cloud
    const auto scan_msgs = vertex->retrieve<PointScan<PointWithInfo>>(
        "raw_point_scan", time_range.first, time_range.second);
#endif

    CLOG(DEBUG, "lidar.dynamic_detection")
        << "Retrieved scan size assocoated with vertex " << vertex->id()
        << " is: " << scan_msgs.size();

    /// simply return if there's no scan to work on
    if (scan_msgs.empty()) continue;

    for (const auto scan_msg : scan_msgs) {
      /// \note follow the convention to lock point map first then these scans.
      auto locked_scan_msg_ref = scan_msg->sharedLocked();  // lock the msg
      auto &locked_scan_msg = locked_scan_msg_ref.get();
      const auto &point_scan = locked_scan_msg.getData();
      //
      const auto &reference = point_scan.point_map();
      auto &query = point_map.point_map();
      const auto &T_ref_qry =
          (T_target_curr * point_scan.T_vertex_map()).inverse() *
          point_map.T_vertex_map();

      //
      detectDynamicObjects(
          reference, query, T_ref_qry, config_->horizontal_resolution,
          config_->vertical_resolution, config_->max_num_observations,
          config_->min_num_observations, config_->dynamic_threshold);
#if false
      // publish point cloud for comparison
      {
        // clang-format off
        const auto T_qry_ref = T_ref_qry.inverse();
        const auto T_qry_ref_mat = T_qry_ref.matrix();
        auto reference_tmp = reference;  // copy
        cart2pol(reference_tmp);
        auto points_mat = reference_tmp.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
        Eigen::Matrix3f R_tot = (T_qry_ref_mat.block<3, 3>(0, 0)).cast<float>();
        Eigen::Vector3f T_tot = (T_qry_ref_mat.block<3, 1>(0, 3)).cast<float>();
        points_mat = (R_tot * points_mat).colwise() + T_tot;
        auto mdl = module_.lock();
        if (mdl && config_->visualize) {
          std::unique_lock<std::mutex> lock(mdl->mutex_);
          {
            PointCloudMsg pc2_msg;
            pcl::toROSMsg(point_map.point_map(), pc2_msg);
            pc2_msg.header.frame_id = "world";
            // pc2_msg.header.stamp = 0;
            mdl->map_pub_->publish(pc2_msg);
          }
          {
            PointCloudMsg pc2_msg;
            pcl::toROSMsg(reference_tmp, pc2_msg);
            pc2_msg.header.frame_id = "world";
            // pc2_msg.header.stamp = 0;
            mdl->scan_pub_->publish(pc2_msg);
          }
        }
        // clang-format on
      }
#endif
    }
  }
  // update version
  point_map.version() = PointMap<PointWithInfo>::DYNAMIC_REMOVED;
  // save the updated point map
  locked_map_msg.setData(point_map);  // this also copies the data

  // Store a copy of the updated map (with dynamic part removed!!)
  using PointMapLM = storage::LockableMessage<PointMap<PointWithInfo>>;
  auto point_map_copy = std::make_shared<PointMap<PointWithInfo>>(
      point_map.dl(), point_map.version());
  point_map_copy->update(point_map.point_map(), true);
  point_map_copy->T_vertex_map() = point_map.T_vertex_map();
  point_map_copy->vertex_id() = point_map.vertex_id();

  auto point_map_copy_msg = std::make_shared<PointMapLM>(
      point_map_copy, locked_map_msg.getTimestamp());
  vertex->insert<PointMap<PointWithInfo>>(
      "point_map_v" + std::to_string(point_map_copy->version()),
      "vtr_lidar_msgs/msg/PointMap", point_map_copy_msg);

  /// publish the transformed pointcloud
  auto mdl = module_.lock();
  if (mdl && config_->visualize) {
    std::unique_lock<std::mutex> lock(mdl->mutex_);

    // publish the old map
    {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(old_map_copy.point_map(), pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      mdl->old_map_pub_->publish(pc2_msg);
    }

    // publish the updated map
    {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(point_map.point_map(), pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      mdl->new_map_pub_->publish(pc2_msg);
    }
  }
  CLOG(INFO, "lidar.dynamic_detection")
      << "Short-Term Dynamics Detection for vertex: " << target_vid_
      << " - DONE!";
}

}  // namespace lidar
}  // namespace vtr