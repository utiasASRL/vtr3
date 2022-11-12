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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/pointmap/dynamic_detection_module.hpp"

#include "vtr_tactic/modules/factory.hpp"
#include "vtr_tactic/task_queue.hpp"

#include "vtr_lidar/data_types/pointmap.hpp"
#include "vtr_lidar/data_types/pointmap_pointer.hpp"
#include "vtr_lidar/segmentation/ray_tracing.hpp"
#include "vtr_pose_graph/path/pose_cache.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto DynamicDetectionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->depth = node->declare_parameter<double>(param_prefix + ".depth", config->depth);
  //
  config->intra_exp_merging = node->declare_parameter<std::string>(param_prefix + ".intra_exp_merging", config->intra_exp_merging);

  config->horizontal_resolution = node->declare_parameter<float>(param_prefix + ".horizontal_resolution", config->horizontal_resolution);
  config->vertical_resolution = node->declare_parameter<float>(param_prefix + ".vertical_resolution", config->vertical_resolution);
  config->max_num_observations = node->declare_parameter<int>(param_prefix + ".max_num_observations", config->max_num_observations);
  config->min_num_observations = node->declare_parameter<int>(param_prefix + ".min_num_observations", config->min_num_observations);
  config->dynamic_threshold = node->declare_parameter<float>(param_prefix + ".dynamic_threshold", config->dynamic_threshold);
  // general
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void DynamicDetectionModule::run_(QueryCache &qdata0, OutputCache &,
                                  const Graph::Ptr &,
                                  const TaskExecutor::Ptr &executor) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (qdata.vid_odo->isValid() &&
      qdata.vid_odo->minorId() >= (unsigned)config_->depth &&
      *qdata.vertex_test_result == VertexTestResult::CREATE_VERTEX) {
    const auto target_vid =
        VertexId(qdata.vid_odo->majorId(),
                 qdata.vid_odo->minorId() - (unsigned)config_->depth);

    qdata.dynamic_detection_async.emplace(target_vid);
    executor->dispatch(std::make_shared<Task>(
        shared_from_this(), qdata.shared_from_this(), 0, Task::DepIdSet{},
        Task::DepId{}, "Dynamic Obstacle Detection", target_vid));
  }
}

void DynamicDetectionModule::runAsync_(QueryCache &qdata0, OutputCache &,
                                       const Graph::Ptr &graph,
                                       const TaskExecutor::Ptr & /* executor */,
                                       const Task::Priority & /* priority */,
                                       const Task::DepId & /* dep_id */) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!publisher_initialized_) {
      // clang-format off
      old_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("dynamic_detection_old", 5);
      new_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("dynamic_detection_new", 5);
      scan_pub_ = qdata.node->create_publisher<PointCloudMsg>("dynamic_detection_scan", 5);
      // clang-format on
      publisher_initialized_ = true;
    }
  }

  /// input
  const auto &target_vid = *qdata.dynamic_detection_async;
  const auto target_vertex = graph->at(target_vid);

  CLOG(INFO, "lidar.dynamic_detection")
      << "Dynamic Obstacle Detection for vertex: " << target_vid;

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
    if (curr_map_version >= PointMap<PointWithInfo>::DYNAMIC_REMOVED) {
      CLOG(WARNING, "lidar.dynamic_detection")
          << "Dynamic Obstacle Detection for vertex: " << target_vid
          << " - ALREADY COMPLETED!";
      return;
    }
    // Check if there's dependency not met
    else if (curr_map_version < PointMap<PointWithInfo>::INTRA_EXP_MERGED) {
#if false
      auto dep_module = factory()->get(config_->intra_exp_merging);
      qdata.intra_exp_merging_async.emplace(target_vid);
      auto dep_task = std::make_shared<Task>(
          dep_module, qdata.shared_from_this(), priority + 1);
      executor->dispatch(dep_task);

      // launch this task again with the same task and dep id
      auto task = std::make_shared<Task>(
          shared_from_this(), qdata.shared_from_this(), priority,
          std::initializer_list<Task::DepId>{dep_task->dep_id}, dep_id);
      executor->dispatch(task);
#endif
      CLOG(WARNING, "lidar.dynamic_detection")
          << "Dynamic Obstacle Detection for vertex: " << target_vid
          << " - REASSIGNED!";
      return;
    }
  }

  /// Perform the map update

  // get a copy of the current map for updating
  const auto map_msg = target_vertex->retrieve<PointMap<PointWithInfo>>(
      "pointmap", "vtr_lidar_msgs/msg/PointMap");
  auto updated_map = map_msg->sharedLocked().get().getData();

  // initialize dynamic observation
  updated_map.point_cloud()
      .getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::flex1_offset())
      .setZero();

#if true  // debugging
  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);

    // publish the old map
    {
      auto point_cloud = updated_map.point_cloud();  // COPY!

      PointCloudMsg pc2_msg;
      pcl::toROSMsg(point_cloud, pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      old_map_pub_->publish(pc2_msg);
    }
  }
#endif

  // get the subgraph of interest to work on (thread safe)
  const auto tempeval = std::make_shared<TemporalEvaluator<GraphBase>>(*graph);
  const auto disteval = std::make_shared<DistanceEvaluator<GraphBase>>(*graph);
  const auto subgraph = graph->dijkstraTraverseToDepth(
      target_vid, config_->depth, disteval, tempeval);

  // cache all the transforms so we only calculate them once
  pose_graph::PoseCache<GraphBase> pose_cache(subgraph, target_vid);

  size_t num_scan_used = 0;
  auto itr = subgraph->begin(target_vid);
  for (; itr != subgraph->end(); itr++) {
    //
    const auto vertex = itr->v();

    // get target vertex to current vertex transformation
    auto T_target_curr = pose_cache.T_root_query(vertex->id());
    CLOG(DEBUG, "lidar.dynamic_detection")
        << "T_target_curr is " << T_target_curr.vec().transpose();

    // retrieve point scan from this vertex
    const auto scan_msg = vertex->retrieve<PointScan<PointWithInfo>>(
        "filtered_point_cloud", "vtr_lidar_msgs/msg/PointScan");

    /// \note follow the convention to lock point map first then these scans.
    auto pointscan = scan_msg->sharedLocked().get().getData();

    //
    const auto &reference = pointscan.point_cloud();
    auto &query = updated_map.point_cloud();
    const auto T_ref_qry =
        (T_target_curr * pointscan.T_vertex_this()).inverse() *
        updated_map.T_vertex_this();

    //
    detectDynamicObjects(
        reference, query, T_ref_qry, config_->horizontal_resolution,
        config_->vertical_resolution, config_->max_num_observations,
        config_->min_num_observations, config_->dynamic_threshold);

#if false  // debugging
    if (config_->visualize) {
      const auto T_qry_ref = T_ref_qry.inverse().matrix().cast<float>();
      auto reference_tmp = reference;
      auto reference_mat = reference_tmp.getMatrixXfMap(
          4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
      reference_mat = T_qry_ref * reference_mat;

      std::unique_lock<std::mutex> lock(mutex_);
      {
        PointCloudMsg pc2_msg;
        pcl::toROSMsg(reference_tmp, pc2_msg);
        pc2_msg.header.frame_id = "world";
        // pc2_msg.header.stamp = 0;
        scan_pub_->publish(pc2_msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
#endif
    //
    ++num_scan_used;
  }
  CLOG(DEBUG, "lidar.dynamic_detection")
      << "Number of scan used: " << num_scan_used;

  // update version
  updated_map.version() = PointMap<PointWithInfo>::DYNAMIC_REMOVED;

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
#if false
    // publish the old map
    {
      // load old map for reference
      const auto map_msg = target_vertex->retrieve<PointMap<PointWithInfo>>(
          "pointmap_v1", "vtr_lidar_msgs/msg/PointMap");
      auto locked_map_msg_ref = map_msg->sharedLocked();  // lock the msg
      auto &locked_map_msg = locked_map_msg_ref.get();
      auto pointmap = locked_map_msg.getData();

      auto point_cloud = pointmap.point_cloud();  // COPY!

      PointCloudMsg pc2_msg;
      pcl::toROSMsg(point_cloud, pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      old_map_pub_->publish(pc2_msg);
    }
#endif
    // publish the updated map
    {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(updated_map.point_cloud(), pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      new_map_pub_->publish(pc2_msg);
    }
  }

  CLOG(INFO, "lidar.dynamic_detection")
      << "Dynamic Obstacle Detection for vertex: " << target_vid << " - DONE!";
}

}  // namespace lidar
}  // namespace vtr