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
 * \file inter_exp_merging_module_v2.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/pointmap/inter_exp_merging_module_v2.hpp"

#include "vtr_lidar/data_types/multi_exp_pointmap.hpp"
#include "vtr_lidar/data_types/pointmap.hpp"
#include "vtr_lidar/data_types/pointmap_pointer.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"
#include "vtr_pose_graph/path/pose_cache.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto InterExpMergingModuleV2::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // point map
  config->map_voxel_size = node->declare_parameter<float>(param_prefix + ".map_voxel_size", config->map_voxel_size);
  config->max_num_exps = (size_t)node->declare_parameter<int>(param_prefix + ".max_num_exps", config->max_num_exps);
  config->distance_threshold = node->declare_parameter<float>(param_prefix + ".distance_threshold", config->distance_threshold);
  config->planar_threshold = node->declare_parameter<float>(param_prefix + ".planar_threshold", config->planar_threshold);
  config->normal_threshold = node->declare_parameter<float>(param_prefix + ".normal_threshold", config->normal_threshold);
  config->dynamic_obs_threshold = node->declare_parameter<int>(param_prefix + ".dynamic_obs_threshold", config->dynamic_obs_threshold);
  // general
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void InterExpMergingModuleV2::run_(QueryCache & /* qdata0 */,
                                   OutputCache & /* output0 */,
                                   const Graph::Ptr & /* graph */,
                                   const TaskExecutor::Ptr & /* executor */) {}

void InterExpMergingModuleV2::runAsync_(QueryCache &qdata0, OutputCache &,
                                        const Graph::Ptr &graph,
                                        const TaskExecutor::Ptr &,
                                        const Task::Priority &,
                                        const Task::DepId &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!publisher_initialized_) {
      // clang-format off
      multi_exp_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("inter_exp_merging_priv", 5);
      single_exp_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("inter_exp_merging_curr", 5);
      // clang-format on
      publisher_initialized_ = true;
    }
  }

  /// input
  const auto [priv_vid, curr_vid, T_priv_curr] = *qdata.inter_exp_merging_async;
  const auto priv_vertex = graph->at(priv_vid);
  const auto curr_vertex = graph->at(curr_vid);

  CLOG(INFO, "lidar.inter_exp_merging")
      << "Inter-Experience Merging for vertex: " << priv_vid;

  /// check if we have a map for the curr vertex
  {
    const auto msg = curr_vertex->retrieve<PointMapPointer>(
        "pointmap_ptr", "vtr_lidar_msgs/msg/PointMapPointer");

    if (msg == nullptr) { 
      CLOG(WARNING, "lidar.inter_exp_merging")
          << "Pointmap pointer, skipped.";
      return;
    }

    auto locked_msg = msg->sharedLocked();
    const auto &pointmap_ptr = locked_msg.get().getData();
    //
    if (pointmap_ptr.map_vid != curr_vid) {
      CLOG(INFO, "lidar.inter_exp_merging")
          << "This vertex does not have an associated submap, skipped.";
      return;
    }
  }

  /// retrieve the map for the curr vertex
  const auto curr_map_msg = curr_vertex->retrieve<PointMap<PointWithInfo>>(
      "pointmap", "vtr_lidar_msgs/msg/PointMap");
  if (curr_map_msg == nullptr) { 
      CLOG(WARNING, "lidar.inter_exp_merging")
          << "Pointmap pointer, skipped.";
      return;
    }
  auto pointmap = curr_map_msg->sharedLocked().get().getData();
  auto &pointcloud = pointmap.point_cloud();

#if true
  // remove dynamic points
  auto filter_cb = [&config = config_](PointWithInfo &query_pt) {
    return bool(query_pt.dynamic_obs <= config->dynamic_obs_threshold);
  };
  pointmap.filter(filter_cb);
#endif

  // transform to the local frame of this vertex
  // clang-format off
  auto point_mat = pointcloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto normal_mat = pointcloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  const auto T_v_m = (T_priv_curr * pointmap.T_vertex_this()).matrix().cast<float>();
  // clang-format on
  point_mat = T_v_m * point_mat;
  normal_mat = T_v_m * normal_mat;

  // initialize bits and multi exp obs
  std::for_each(pointcloud.begin(), pointcloud.end(), [&](auto &p) {
    p.bits = 1;
    p.multi_exp_obs = 1.0;
  });

  /// check if we have the multiexp map for priv vertex
  using MultiExpPointMapLM =
      storage::LockableMessage<MultiExpPointMap<PointWithInfo>>;
  const auto mepointmap_msg = [&]() -> std::shared_ptr<MultiExpPointMapLM> {
    auto mepointmap_msg =
        priv_vertex->retrieve<MultiExpPointMap<PointWithInfo>>(
            "mepointmap", "vtr_lidar_msgs/msg/MultiExpPointMap");
    if (mepointmap_msg != nullptr) return mepointmap_msg;

    // create the map
    auto mepointmap = std::make_shared<MultiExpPointMap<PointWithInfo>>(
        config_->map_voxel_size, config_->max_num_exps);
    mepointmap_msg = std::make_shared<MultiExpPointMapLM>(
        mepointmap, priv_vertex->vertexTime());
    priv_vertex->insert<MultiExpPointMap<PointWithInfo>>(
        "mepointmap", "vtr_lidar_msgs/msg/MultiExpPointMap", mepointmap_msg);
    CLOG(INFO, "lidar.inter_exp_merging")
        << "Created a new map for vertex: " << priv_vid;
    return mepointmap_msg;
  }();
  auto mepointmap = mepointmap_msg->sharedLocked().get().getData();

  /// Perform the map update
  auto &exps = mepointmap.exps();
  auto &mepointcloud = mepointmap.point_cloud();

  // update transform
  mepointmap.T_vertex_this() = tactic::EdgeTransform(true);
  mepointmap.vertex_id() = priv_vid;

  // update the experiences
  if (exps.empty() || exps.back() != curr_vid.majorId()) {
    // add a new experience
    exps.push_back(curr_vid.majorId());
    // remove oldest experience
    if (exps.size() > mepointmap.max_num_exps()) exps.pop_front();
    //
    std::for_each(mepointmap.point_cloud().begin(),
                  mepointmap.point_cloud().end(),
                  [&](auto &p) { p.bits <<= 1; });
    /// \todo filter out points with no observations

    CLOG(INFO, "lidar.inter_exp_merging")
        << "Added a new experience with id: " << curr_vid.majorId();
  }

  // update existing point observations
  NanoFLANNAdapter<PointWithInfo> adapter(mepointcloud);
  KDTreeSearchParams search_params;
  KDTreeParams tree_params(/* max leaf */ 10);
  auto kdtree =
      std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  const auto sq_radius =
      config_->distance_threshold * config_->distance_threshold;
  for (const auto &pt : pointcloud) {
    std::vector<float> dists;
    std::vector<int> indices;
    NanoFLANNRadiusResultSet<float, int> result(sq_radius, dists, indices);
    kdtree->radiusSearchCustomCallback(pt.data, result, search_params);

    for (const auto &idx : indices) {
      auto &pt2 = mepointcloud[idx];

      // check point to plane distance - todo: replace by
      const auto diff = pt.getVector3fMap() - pt2.getVector3fMap();
      const auto planar_dist = std::abs(pt2.getNormalVector3fMap().dot(diff));
      if (planar_dist > config_->planar_threshold) continue;

      // check normal consistency
      const auto normal_dist =
          std::abs(pt2.getNormalVector3fMap().dot(pt.getNormalVector3fMap()));
      if (normal_dist < config_->normal_threshold) continue;

      // updated the point
      if ((pt2.bits & 1) == 0) {
        pt2.bits++;
        pt2.multi_exp_obs += 1.0;
      }
    }
  }

  // update the map with new points
  mepointmap.update(pointcloud);

  // update the point map of this vertex
  mepointmap_msg->locked().get().setData(mepointmap);

  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);

    // publish the curr map
    {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(pointmap.point_cloud(), pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      single_exp_map_pub_->publish(pc2_msg);
    }

    // publish the updated map (will already be in vertex frame)
    {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(mepointmap.point_cloud(), pc2_msg);
      pc2_msg.header.frame_id = "world";
      // pc2_msg.header.stamp = 0;
      multi_exp_map_pub_->publish(pc2_msg);
    }
  }

  CLOG(INFO, "lidar.inter_exp_merging")
      << "Inter-Experience Merging for vertex: " << priv_vid << " - DONE!";
}

}  // namespace lidar
}  // namespace vtr