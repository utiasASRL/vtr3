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
 * \file inter_exp_merging_module.cpp
 * \brief InterExpMergingModule class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/inter_exp_merging_module.hpp"

#include "vtr_lidar/pointmap/pointmap_v2.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto InterExpMergingModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                            const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->dynamic_detection = node->declare_parameter<std::string>(param_prefix + ".dynamic_detection", config->dynamic_detection);

  config->depth = node->declare_parameter<int>(param_prefix + ".depth", config->depth);

  config->horizontal_resolution = node->declare_parameter<float>(param_prefix + ".horizontal_resolution", config->horizontal_resolution);
  config->vertical_resolution = node->declare_parameter<float>(param_prefix + ".vertical_resolution", config->vertical_resolution);
  config->max_num_observations = node->declare_parameter<int>(param_prefix + ".max_num_observations", config->max_num_observations);

  config->max_num_experiences = node->declare_parameter<int>(param_prefix + ".max_num_experiences", config->max_num_experiences);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void InterExpMergingModule::runImpl(QueryCache &qdata0, OutputCache &,
                                    const Graph::Ptr &graph,
                                    const TaskExecutor::Ptr &executor) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    old_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("inter_exp_merging_old", 5);
    new_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("inter_exp_merging_new", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  if (qdata.live_id->isValid() &&
      qdata.live_id->minorId() >= (unsigned)config_->depth &&
      *qdata.keyframe_test_result == KeyframeTestResult::CREATE_VERTEX) {
    const auto target_vid =
        VertexId(qdata.live_id->majorId(),
                 qdata.live_id->minorId() - (unsigned)config_->depth);

    auto spatial_neighbors = graph->at(target_vid)->spatialNeighbours();
    if (spatial_neighbors.empty()) {
      /// \todo figure out whether this is a teach or a repeat, the current way
      /// does not work at branching/merging vertices
      CLOG(INFO, "lidar.inter_exp_merging")
          << "The target vertex has no spatial neighbors - default to teach "
             "run.";
      qdata.inter_exp_merging_async.emplace(target_vid, VertexId::Invalid());
      executor->dispatch(
          std::make_shared<Task>(shared_from_this(), qdata.shared_from_this()));

    } else {
      CLOG(WARNING, "lidar.inter_exp_merging")
          << "Spatial neighbors of " << target_vid << " are "
          << spatial_neighbors << ", launching for repeat run.";

      for (const auto &map_vid : spatial_neighbors) {
        auto temp_qdata = std::make_shared<LidarQueryCache>(qdata);
        temp_qdata->inter_exp_merging_async.emplace(target_vid, map_vid);
        executor->dispatch(
            std::make_shared<Task>(shared_from_this(), temp_qdata));
      }
    }
  }
}

void InterExpMergingModule::runAsyncImpl(QueryCache &qdata0, OutputCache &,
                                         const Graph::Ptr &graph,
                                         const TaskExecutor::Ptr &executor,
                                         const Task::Priority &priority,
                                         const Task::DepId &dep_id) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  const auto &live_vid = qdata.inter_exp_merging_async->first;
  const auto &map_vid = qdata.inter_exp_merging_async->second;

  CLOG(INFO, "lidar.inter_exp_merging")
      << "Inter-Experience Merging for vertex: " << live_vid
      << " against vertex: " << map_vid;

  /// Check if this vertex has dynamic removed map
  {
    auto vertex = graph->at(live_vid);
    const auto live_map_msg = vertex->retrieve<PointMap<PointWithInfo>>(
        "point_map", "vtr_lidar_msgs/msg/PointMap");
    auto locked_live_map_msg_ref =
        live_map_msg->sharedLocked();  // lock the msg
    auto &locked_live_map_msg = locked_live_map_msg_ref.get();
    const auto curr_map_version = locked_live_map_msg.getData().version();
    if (curr_map_version >= PointMap<PointWithInfo>::INTER_EXP_MERGED) {
      CLOG(WARNING, "lidar.dynamic_detection")
          << "Inter-Experience Merging for vertex: " << live_vid
          << " against vertex: " << map_vid << " - ALREADY COMPLETED!";
      return;
    }
    // Check if there's dependency not met
    else if (curr_map_version < PointMap<PointWithInfo>::DYNAMIC_REMOVED) {
      qdata.dynamic_detection_async.emplace(live_vid);
      auto dep_module = factory()->get(config_->dynamic_detection);
      auto dep_task = std::make_shared<Task>(
          dep_module, qdata.shared_from_this(), priority + 1);
      executor->dispatch(dep_task);

      // launch this task again with the same task and dep id
      auto task = std::make_shared<Task>(
          shared_from_this(), qdata.shared_from_this(), priority,
          std::initializer_list<Task::DepId>{dep_task->dep_id}, dep_id);
      executor->dispatch(task);

      CLOG(WARNING, "lidar.dynamic_detection")
          << "Inter-Experience Merging for vertex: " << live_vid
          << " - REASSIGNED!";
      return;
    }
  }

  /// Check if the map id already has a multi-exp map we can work on
  if (map_vid.isValid()) {
    auto vertex = graph->at(map_vid);
    const auto map_msg = vertex->retrieve<MultiExpPointMap<PointWithInfo>>(
        "multi_exp_point_map", "vtr_lidar_msgs/msg/PointMap");
    if (map_msg == nullptr) {
      auto temp_qdata = std::make_shared<LidarQueryCache>(qdata);
      temp_qdata->inter_exp_merging_async.emplace(map_vid, VertexId::Invalid());

      // launch this task again with the same task and dep id
      auto dep_task =
          std::make_shared<Task>(shared_from_this(), temp_qdata, priority + 1);
      executor->dispatch(dep_task);

      // launch this task again with the same task and dep id
      auto task = std::make_shared<Task>(
          shared_from_this(), qdata.shared_from_this(), priority,
          std::initializer_list<Task::DepId>{dep_task->dep_id}, dep_id);
      executor->dispatch(task);

      CLOG(WARNING, "lidar.dynamic_detection")
          << "Inter-Experience Merging for vertex: " << live_vid
          << " which connects to vertex: " << map_vid << " - REASSIGNED!";
      return;
    }
  }

  CLOG(WARNING, "lidar.dynamic_detection")
      << "Inter-Experience Merging for vertex: " << live_vid
      << " against vertex: " << map_vid
      << " has all dependency met, now ready to perform inter-exp merging.";

  // Store a copy of the original map for visualization
  auto old_map_copy = [&]() {
    auto vertex = graph->at(live_vid);
    const auto map_msg = vertex->retrieve<PointMap<PointWithInfo>>(
        "point_map", "vtr_lidar_msgs/msg/PointMap");
    auto locked_map_msg_ref = map_msg->sharedLocked();  // lock the msg
    auto &locked_map_msg = locked_map_msg_ref.get();
    return locked_map_msg.getData();
  }();

  // Perform the map update

  // initialize a new multi-exp map
  if (!map_vid.isValid()) {
    auto vertex = graph->at(live_vid);
    const auto live_map_msg = vertex->retrieve<PointMap<PointWithInfo>>(
        "point_map", "vtr_lidar_msgs/msg/PointMap");
    auto locked_live_map_msg_ref = live_map_msg->locked();
    auto &locked_live_map_msg = locked_live_map_msg_ref.get();
    auto live_map = locked_live_map_msg.getData();
    // create a new multi experience map
    auto multi_exp_map = std::make_shared<MultiExpPointMap<PointWithInfo>>(
        live_map.dl(), config_->max_num_experiences);
    // update the map with this single exp map
    multi_exp_map->update(live_map);
    // save the multi-exp map
    using MultiExpPointMapLM =
        storage::LockableMessage<MultiExpPointMap<PointWithInfo>>;
    auto multi_exp_map_msg = std::make_shared<MultiExpPointMapLM>(
        multi_exp_map, locked_live_map_msg.getTimestamp());
    vertex->insert<MultiExpPointMap<PointWithInfo>>(
        "multi_exp_point_map", "vtr_lidar_msgs/msg/PointMap",
        multi_exp_map_msg);

    // update the single exp map version so that we know it has been merged
    live_map.version() = PointMap<PointWithInfo>::INTER_EXP_MERGED;
    locked_live_map_msg.setData(live_map);

    // store a copy of the new multi-exp map in case we need it
    auto multi_exp_map_copy =
        std::make_shared<MultiExpPointMap<PointWithInfo>>(*multi_exp_map);
    auto multi_exp_map_copy_msg = std::make_shared<MultiExpPointMapLM>(
        multi_exp_map_copy, locked_live_map_msg.getTimestamp());
    vertex->insert<MultiExpPointMap<PointWithInfo>>(
        "point_map_v" + std::to_string(multi_exp_map_copy->version()) + "_r" +
            std::to_string(live_vid.majorId()),
        "vtr_lidar_msgs/msg/PointMap", multi_exp_map_copy_msg);
  } else {
    auto live_vertex = graph->at(live_vid);
    auto map_vertex = graph->at(map_vid);
    // get transform from live vertex to map vertex
    const auto &T_mv_lv = graph->at(map_vid, live_vid)->T();
    // retrieve messages
    const auto live_msg = live_vertex->retrieve<PointMap<PointWithInfo>>(
        "point_map", "vtr_lidar_msgs/msg/PointMap");
    const auto map_msg = map_vertex->retrieve<MultiExpPointMap<PointWithInfo>>(
        "multi_exp_point_map", "vtr_lidar_msgs/msg/PointMap");
    // lock both messages at the same time
    using LockType = std::unique_lock<std::shared_mutex>;
    LockType live_lock(live_msg->mutex(), std::defer_lock);
    LockType map_lock(map_msg->mutex(), std::defer_lock);
    std::lock(live_lock, map_lock);
    // get data_ref to work on
    auto &live_ref = live_msg->unlocked().get();
    auto &map_ref = map_msg->unlocked().get();
    auto live = live_ref.getData();  // copy!
    auto map = map_ref.getData();    // copy!
    // check if we have already updated the map with this run
    if (map.experiences().back().majorId() >= live_vid.majorId()) {
      CLOG(WARNING, "lidar.dynamic_detection")
          << "Inter-Experience Merging for " << map_vid
          << "has already been updated using a map from this run. Skip "
             "updating.";
      return;
    }
    // transform live pointmap points to map pointmap frame
    const auto &T_map_live =
        (map.T_vertex_map().inverse()) * T_mv_lv * live.T_vertex_map();
    const auto T_map_live_mat = T_map_live.matrix();
    auto &live_point_cloud = live.point_map();
    // get eigen mapping
    auto points_mat = live_point_cloud.getMatrixXfMap(
        3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
    auto normal_mat = live_point_cloud.getMatrixXfMap(
        3, PointWithInfo::size(), PointWithInfo::normal_offset());
    // transform
    Eigen::Matrix3f R_tot = (T_map_live_mat.block<3, 3>(0, 0)).cast<float>();
    Eigen::Vector3f T_tot = (T_map_live_mat.block<3, 1>(0, 3)).cast<float>();
    points_mat = (R_tot * points_mat).colwise() + T_tot;
    normal_mat = R_tot * normal_mat;
    // update the map and save it
    map.update(live);
    map_ref.setData(map);

    // update the single exp map version so that we know it has been merged
    live.version() = PointMap<PointWithInfo>::INTER_EXP_MERGED;
    live_ref.setData(live);

    // store a copy of the new multi-exp map in case we need it
    auto map_copy = std::make_shared<MultiExpPointMap<PointWithInfo>>(map);
    using MultiExpPointMapLM =
        storage::LockableMessage<MultiExpPointMap<PointWithInfo>>;
    auto map_copy_msg =
        std::make_shared<MultiExpPointMapLM>(map_copy, map_ref.getTimestamp());
    map_vertex->insert<MultiExpPointMap<PointWithInfo>>(
        "point_map_v" + std::to_string(map_copy->version()) + "_r" +
            std::to_string(live_vid.majorId()),
        "vtr_lidar_msgs/msg/PointMap", map_copy_msg);
  }

  /// retrieve and lock the multi-exp map to be visualized
  auto vertex = map_vid.isValid() ? graph->at(map_vid) : graph->at(live_vid);
  const auto multi_exp_map_msg =
      vertex->retrieve<MultiExpPointMap<PointWithInfo>>(
          "multi_exp_point_map", "vtr_lidar_msgs/msg/PointMap");
  auto locked_multi_exp_map_msg_ref = multi_exp_map_msg->sharedLocked();
  const auto &multi_exp_map = locked_multi_exp_map_msg_ref.get().getData();

  /// publish the transformed pointcloud
  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);
    // publish the old map
    {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(old_map_copy.point_map(), pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      old_map_pub_->publish(pc2_msg);
    }
    // publish the new map
    {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(multi_exp_map.point_map(), pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      new_map_pub_->publish(pc2_msg);
    }
  }
  CLOG(INFO, "lidar.dynamic_detection")
      << "Inter-Experience Merging for vertex: " << live_vid << " - DONE!";
}
}  // namespace lidar
}  // namespace vtr