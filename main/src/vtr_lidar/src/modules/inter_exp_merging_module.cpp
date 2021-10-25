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

#include "vtr_lidar/modules/dynamic_detection_module.hpp"
#include "vtr_lidar/pointmap/pointmap_v2.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

void InterExpMergingModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->depth = node->declare_parameter<int>(param_prefix + ".depth", config_->depth);

  config_->horizontal_resolution = node->declare_parameter<float>(param_prefix + ".horizontal_resolution", config_->horizontal_resolution);
  config_->vertical_resolution = node->declare_parameter<float>(param_prefix + ".vertical_resolution", config_->vertical_resolution);
  config_->max_num_observations = node->declare_parameter<int>(param_prefix + ".max_num_observations", config_->max_num_observations);

  config_->max_num_experiences = node->declare_parameter<int>(param_prefix + ".max_num_experiences", config_->max_num_experiences);

  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void InterExpMergingModule::runImpl(QueryCache &qdata,
                                    const Graph::ConstPtr &graph) {
  if (!task_queue_) return;

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
      task_queue_->dispatch(std::make_shared<Task>(
          shared_from_base<InterExpMergingModule>(), config_, target_vid));

    } else {
      CLOG(WARNING, "lidar.inter_exp_merging")
          << "Spatial neighbors of " << target_vid << " are "
          << spatial_neighbors << ", launching for repeat run.";

      for (const auto &map_vid : spatial_neighbors)
        task_queue_->dispatch(
            std::make_shared<Task>(shared_from_base<InterExpMergingModule>(),
                                   config_, target_vid, map_vid));
    }
  }
}
void InterExpMergingModule::Task::run(const AsyncTaskExecutor::Ptr &executor,
                                      const Graph::Ptr &graph) {
  CLOG(INFO, "lidar.inter_exp_merging")
      << "Inter-Experience Merging for vertex: " << live_vid_
      << " against vertex: " << map_vid_;

  /// Check if this vertex has dynamic removed map
  {
    auto vertex = graph->at(live_vid_);
    const auto live_map_msg =
        vertex->retrieve<PointMap<PointWithInfo>>("point_map");
    auto locked_live_map_msg_ref =
        live_map_msg->sharedLocked();  // lock the msg
    auto &locked_live_map_msg = locked_live_map_msg_ref.get();
    const auto curr_map_version = locked_live_map_msg.getData().version();
    if (curr_map_version >= PointMap<PointWithInfo>::INTER_EXP_MERGED) {
      CLOG(WARNING, "lidar.dynamic_detection")
          << "Inter-Experience Merging for vertex: " << live_vid_
          << " against vertex: " << map_vid_ << " - ALREADY COMPLETED!";
      return;
    }
    // Check if there's dependency not met
    else if (curr_map_version < PointMap<PointWithInfo>::DYNAMIC_REMOVED) {
      const auto config = std::make_shared<DynamicDetectionModule::Config>();
      config->visualize = false;
      config->depth = config_->depth;
      config->horizontal_resolution = config_->horizontal_resolution;
      config->vertical_resolution = config_->vertical_resolution;
      config->max_num_observations = config_->max_num_observations;

      executor->tryDispatch(std::make_shared<DynamicDetectionModule::Task>(
          nullptr, config, live_vid_, priority + 1));
      executor->tryDispatch(shared_from_this());
      CLOG(WARNING, "lidar.dynamic_detection")
          << "Inter-Experience Merging for vertex: " << live_vid_
          << " - REASSIGNED!";
      return;
    }
  }

  /// Check if the map id already has a multi-exp map we can work on
  if (map_vid_.isValid()) {
    auto vertex = graph->at(map_vid_);
    const auto map_msg = vertex->retrieve<MultiExpPointMap<PointWithInfo>>(
        "multi_exp_point_map");
    if (map_msg == nullptr) {
      executor->tryDispatch(std::make_shared<InterExpMergingModule::Task>(
          nullptr, config_, map_vid_, VertexId::Invalid(), priority + 1));
      executor->tryDispatch(shared_from_this());
      CLOG(WARNING, "lidar.dynamic_detection")
          << "Inter-Experience Merging for vertex: " << live_vid_
          << " which connects to vertex: " << map_vid_ << " - REASSIGNED!";
      return;
    }
  }

  CLOG(WARNING, "lidar.dynamic_detection")
      << "Inter-Experience Merging for vertex: " << live_vid_
      << " against vertex: " << map_vid_
      << " has all dependency met, now ready to perform inter-exp merging.";

  // Store a copy of the original map
  auto map_copy = [&]() {
    auto vertex = graph->at(live_vid_);
    const auto map_msg = vertex->retrieve<PointMap<PointWithInfo>>("point_map");
    auto locked_map_msg_ref = map_msg->sharedLocked();  // lock the msg
    auto &locked_map_msg = locked_map_msg_ref.get();
    auto map_copy =
        std::make_shared<PointMap<PointWithInfo>>(locked_map_msg.getData());
    using PointMapLM = storage::LockableMessage<PointMap<PointWithInfo>>;
    auto map_copy_msg =
        std::make_shared<PointMapLM>(map_copy, locked_map_msg.getTimestamp());
    vertex->insert<PointMap<PointWithInfo>>(
        "point_map_v" + std::to_string(map_copy->version()), map_copy_msg);
    return map_copy;
  }();

  // Perform the map update

  // initialize a new multi-exp map
  if (!map_vid_.isValid()) {
    auto vertex = graph->at(live_vid_);
    const auto live_map_msg =
        vertex->retrieve<PointMap<PointWithInfo>>("point_map");
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
    vertex->insert<MultiExpPointMap<PointWithInfo>>("multi_exp_point_map",
                                                    multi_exp_map_msg);
    // update the single exp map version so that we know it has been merged
    live_map.version() = PointMap<PointWithInfo>::INTER_EXP_MERGED;
    locked_live_map_msg.setData(live_map);
  } else {
    auto live_vertex = graph->at(live_vid_);
    auto map_vertex = graph->at(map_vid_);
    // get transform from live vertex to map vertex
    const auto &T_mv_lv = graph->at(map_vid_, live_vid_)->T();
    // retrieve messages
    const auto live_msg =
        live_vertex->retrieve<PointMap<PointWithInfo>>("point_map");
    const auto map_msg = map_vertex->retrieve<MultiExpPointMap<PointWithInfo>>(
        "multi_exp_point_map");
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
    if (map.experiences().back().majorId() >= live_vid_.majorId()) {
      CLOG(WARNING, "lidar.dynamic_detection")
          << "Inter-Experience Merging for " << map_vid_
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
  }

  /// retrieve and lock the multi-exp map to be visualized
  auto vertex = map_vid_.isValid() ? graph->at(map_vid_) : graph->at(live_vid_);
  const auto multi_exp_map_msg =
      vertex->retrieve<MultiExpPointMap<PointWithInfo>>("multi_exp_point_map");
  auto locked_multi_exp_map_msg_ref = multi_exp_map_msg->sharedLocked();
  const auto &multi_exp_map = locked_multi_exp_map_msg_ref.get().getData();

  /// publish the transformed pointcloud
  auto mdl = module_.lock();
  if (mdl && config_->visualize) {
    std::unique_lock<std::mutex> lock(mdl->mutex_);
    // publish the old map
    {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(map_copy->point_map(), pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      mdl->old_map_pub_->publish(pc2_msg);
    }
    // publish the new map
    {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(multi_exp_map.point_map(), pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      mdl->new_map_pub_->publish(pc2_msg);
    }
  }
  CLOG(INFO, "lidar.dynamic_detection")
      << "Inter-Experience Merging for vertex: " << live_vid_ << " - DONE!";
}
}  // namespace lidar
}  // namespace vtr