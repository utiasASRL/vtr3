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
 * \file windowed_map_recall_module.cpp
 * \brief WindowedMapRecallModule class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/modules/windowed_map_recall_module.hpp>

namespace vtr {
namespace lidar {

namespace {
void retrievePointMap(const PointMapMsg::SharedPtr &map_msg,
                      std::vector<PointXYZ> &points,
                      std::vector<PointXYZ> &normals,
                      std::vector<float> &scores,
                      std::vector<std::pair<int, int>> &movabilities) {
  auto N = map_msg->points.size();
  points.reserve(N);
  normals.reserve(N);
  scores.reserve(N);
  movabilities.reserve(N);

  for (unsigned i = 0; i < N; i++) {
    const auto &point = map_msg->points[i];
    const auto &normal = map_msg->normals[i];
    const auto &score = map_msg->scores[i];
    const auto &mb = map_msg->movabilities[i];
    // Add all points to the vector container
    points.push_back(PointXYZ(point.x, point.y, point.z));
    normals.push_back(PointXYZ(normal.x, normal.y, normal.z));
    scores.push_back(score);
    movabilities.push_back(std::pair<int, int>(mb.dynamic_obs, mb.total_obs));
  }
}

void migratePoints(const lgmath::se3::TransformationWithCovariance &T,
                   std::vector<PointXYZ> &points,
                   std::vector<PointXYZ> &normals) {
  /// Transform subsampled points into the map frame
  const auto T_mat = T.matrix();
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat(
      (float *)points.data(), 3, points.size());
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> norms_mat(
      (float *)normals.data(), 3, normals.size());
  Eigen::Matrix3f R_tot = (T_mat.block<3, 3>(0, 0)).cast<float>();
  Eigen::Vector3f T_tot = (T_mat.block<3, 1>(0, 3)).cast<float>();
  pts_mat = (R_tot * pts_mat).colwise() + T_tot;
  norms_mat = R_tot * norms_mat;
}

}  // namespace

using namespace tactic;

void WindowedMapRecallModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                            const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->single_exp_map_voxel_size = node->declare_parameter<float>(param_prefix + ".single_exp_map_voxel_size", config_->single_exp_map_voxel_size);
  config_->multi_exp_map_voxel_size = node->declare_parameter<float>(param_prefix + ".multi_exp_map_voxel_size", config_->multi_exp_map_voxel_size);

  config_->remove_short_term_dynamic = node->declare_parameter<bool>(param_prefix + ".remove_short_term_dynamic", config_->remove_short_term_dynamic);
  config_->short_term_min_num_observations = node->declare_parameter<int>(param_prefix + ".short_term_min_num_observations", config_->short_term_min_num_observations);
  config_->short_term_min_movability = node->declare_parameter<float>(param_prefix + ".short_term_min_movability", config_->short_term_min_movability);

  config_->depth = node->declare_parameter<int>(param_prefix + ".depth", config_->depth);
  config_->num_additional_exps = node->declare_parameter<int>(param_prefix + ".num_additional_exps", config_->num_additional_exps);
  config_->long_term_min_num_observations = node->declare_parameter<int>(param_prefix + ".long_term_min_num_observations", config_->long_term_min_num_observations);

  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void WindowedMapRecallModule::runImpl(QueryCache &qdata0,
                                      const Graph::ConstPtr &graph) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    observation_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("loc_map_pts_obs", 5);
    experience_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("loc_map_pts_exp", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  /// Input
  const auto &live_id = *qdata.live_id;
  const auto &map_id = *qdata.map_id;

  if (qdata.current_map_loc_vid && *qdata.current_map_loc_vid == map_id) {
    CLOG(DEBUG, "lidar.windowed_map_recall")
        << "Map already loaded, simply return. Map size is: "
        << (*qdata.current_map_loc).size();
    return;
  }

  /// Get a subgraph containing all experiences with specified window
  std::set<RunId> selected_exps;
  std::set<VertexId> vertices;
  if (config_->depth == 0) {
    /// \todo add spatial neighbors
    vertices.insert(map_id);
    selected_exps.insert(map_id.majorId());
  } else {
    const auto &current_run = live_id.majorId();
    if (*qdata.pipeline_mode == tactic::PipelineMode::Following) {
      CLOG(DEBUG, "lidar.windowed_map_recall")
          << "In path following, looking at the localization chain.";
      // Acquire both locks since we are querying the localization chain while
      // traversing through the graph
      std::lock(qdata.loc_chain->mutex(), graph->mutex());
      const auto &map_sid = qdata.loc_chain->branchSequenceId();
      if (qdata.loc_chain->sequence().at(map_sid) != map_id) {
        std::stringstream err;
        err << "Localization chain branch vid and map vid must always match - "
               "branch vid: "
            << qdata.loc_chain->sequence().at(map_sid)
            << ", map vid: " << map_id;
        CLOG(ERROR, "lidar.windowed_map_recall") << err.str();
        throw std::runtime_error{err.str()};
      }
      for (unsigned sid = map_sid; sid < qdata.loc_chain->size() &&
                                   sid < map_sid + (unsigned)config_->depth;
           sid++) {
        const auto &vid = qdata.loc_chain->sequence().at(sid);
        vertices.insert(vid);
        selected_exps.insert(vid.majorId());
        // add the spatial neighbours
        const auto &spatial_vids = graph->at(vid)->spatialNeighbours();
        for (const auto &spatial_vid : spatial_vids) {
          // don't add the live run to the localization map
          if (spatial_vid.majorId() == current_run) continue;
          // now that the checks have passed, add it to the list
          vertices.insert(spatial_vid);
          selected_exps.insert(spatial_vid.majorId());
        }
      }
      graph->unlock();
      qdata.loc_chain->unlock();
    } else {
      graph->lock();
      PrivilegedEvaluator::Ptr evaluator(new PrivilegedEvaluator());
      evaluator->setGraph((void *)graph.get());
      auto itr = graph->beginDfs(map_id, config_->depth, evaluator);
      for (; itr != graph->end(); ++itr) {
        auto current_vertex = itr->v();
        /// \todo separate backward and forward depth
        /// simply continue will cause issue when there is loop closure
        // if (current_vertex->id().minorId() < map_id.minorId()) continue;
        // add the current, privileged vertex.
        vertices.insert(current_vertex->id());
        selected_exps.insert(current_vertex->id().majorId());
        // add the spatial neighbours
        const auto &spatial_vids = current_vertex->spatialNeighbours();
        for (const auto &spatial_vid : spatial_vids) {
          // don't add the live run to the localization map
          if (spatial_vid.majorId() == current_run) continue;
          // now that the checks have passed, add it to the list
          vertices.insert(spatial_vid);
          selected_exps.insert(spatial_vid.majorId());
        }
      }
      graph->unlock();
    }
  }
  auto sub_graph = graph->getSubgraph(
      std::vector<VertexId>(vertices.begin(), vertices.end()));
  CLOG(DEBUG, "lidar.windowed_map_recall")
      << "Looking at spatial vertices: " << vertices;

  /// \todo experience selection
  // for now we choose the latest n experiences plus previleged experience
  // note that previleged experience always has the lowest run id.
  if (selected_exps.size() > (size_t)config_->num_additional_exps + 1) {
    auto begin = selected_exps.begin();
    auto end = selected_exps.begin();
    std::advance(begin, 1);
    std::advance(end,
                 selected_exps.size() - (size_t)config_->num_additional_exps);
    selected_exps.erase(begin, end);
  }
  CLOG(DEBUG, "lidar.windowed_map_recall")
      << "Selected experience ids: " << selected_exps;

  /// Create single experience maps for the selected experiences
  std::unordered_map<RunId, std::shared_ptr<SingleExpPointMap>> single_exp_maps;

  // cache all the transforms so we only calculate them once
  pose_graph::PoseCache<pose_graph::RCGraphBase> pose_cache(sub_graph, map_id);

  for (const auto &vid : vertices) {
    if (selected_exps.count(vid.majorId()) == 0) continue;
    CLOG(DEBUG, "lidar.windowed_map_recall") << "Looking at vertex: " << vid;
    if (single_exp_maps.count(vid.majorId()) == 0) {
      // create a single experience map for this run
      single_exp_maps[vid.majorId()] = std::make_shared<SingleExpPointMap>(
          config_->single_exp_map_voxel_size,
          config_->remove_short_term_dynamic,
          config_->short_term_min_num_observations,
          config_->short_term_min_movability);
      // create vertex stream for this run
      auto run = sub_graph->run(vid.majorId());
      run->registerVertexStream<PointMapMsg>(
          "pointmap", true, pose_graph::RegisterMode::Existing);
    }
    // get transformation
    auto T_root_curr = pose_cache.T_root_query(vid);
    // migrate submaps
    auto vertex = sub_graph->at(vid);
    vertex->load("pointmap");  /// \todo should  be in retrieveKeyframeData?
    const auto &map_msg = vertex->retrieveKeyframeData<PointMapMsg>("pointmap");
    std::vector<PointXYZ> points;
    std::vector<PointXYZ> normals;
    std::vector<float> scores;
    std::vector<std::pair<int, int>> movabilities;
    retrievePointMap(map_msg, points, normals, scores, movabilities);
    migratePoints(T_root_curr, points, normals);
    single_exp_maps.at(vid.majorId())
        ->update(points, normals, scores, movabilities);
  }

  /// Create multi experience map from single experience maps
  auto multi_exp_map =
      std::make_shared<MultiExpPointMap>(config_->multi_exp_map_voxel_size);
  multi_exp_map->update(single_exp_maps);
  CLOG(DEBUG, "lidar.windowed_map_recall")
      << "Multi experience map with " << multi_exp_map->number_of_experiences
      << " experiences and size: " << multi_exp_map->size()
      << " before filtering.";

  /// \todo filter based on some clever criteria

  /// \note: -1 gives some buffer (may not need)
  int min_num_obs = (int)multi_exp_map->number_of_experiences - 1;
  min_num_obs = std::min(config_->long_term_min_num_observations, min_num_obs);
  multi_exp_map->filter(min_num_obs);
  CLOG(DEBUG, "lidar.windowed_map_recall")
      << "Multi experience map with " << multi_exp_map->number_of_experiences
      << " experiences and size: " << multi_exp_map->size()
      << " after filtering.";

  multi_exp_map->buildKDTree();
  CLOG(DEBUG, "lidar.windowed_map_recall")
      << "Created multi experience map with "
      << multi_exp_map->number_of_experiences
      << " experiences and size: " << multi_exp_map->size();

  if (config_->visualize) {
    {
      // publish map and number of observations of each point
      auto pc2_msg = std::make_shared<PointCloudMsg>();
      pcl::PointCloud<pcl::PointXYZI> cloud;
      cloud.reserve(multi_exp_map->cloud.pts.size());

      auto pcitr = multi_exp_map->cloud.pts.begin();
      auto ititr = multi_exp_map->observations.begin();
      for (; pcitr != multi_exp_map->cloud.pts.end(); pcitr++, ititr++) {
        pcl::PointXYZI pt;
        pt.x = pcitr->x;
        pt.y = pcitr->y;
        pt.z = pcitr->z;
        pt.intensity = *ititr;
        cloud.points.push_back(pt);
      }

      pcl::toROSMsg(cloud, *pc2_msg);
      pc2_msg->header.frame_id = "localization keyframe";
      pc2_msg->header.stamp = *qdata.rcl_stamp;

      observation_map_pub_->publish(*pc2_msg);
    }

    {
      // publish map and number of observations of each point
      auto pc2_msg = std::make_shared<PointCloudMsg>();
      pcl::PointCloud<pcl::PointXYZI> cloud;
      cloud.reserve(multi_exp_map->cloud.pts.size());

      auto pcitr = multi_exp_map->cloud.pts.begin();
      auto ititr = multi_exp_map->experiences.begin();
      for (; pcitr != multi_exp_map->cloud.pts.end(); pcitr++, ititr++) {
        pcl::PointXYZI pt;
        pt.x = pcitr->x;
        pt.y = pcitr->y;
        pt.z = pcitr->z;
        pt.intensity = *ititr;
        cloud.points.push_back(pt);
      }

      pcl::toROSMsg(cloud, *pc2_msg);
      pc2_msg->header.frame_id = "localization keyframe";
      pc2_msg->header.stamp = *qdata.rcl_stamp;

      experience_map_pub_->publish(*pc2_msg);
    }
  }

  /// Output
  qdata.current_map_loc = multi_exp_map;
  qdata.current_map_loc_vid.fallback(map_id);
}

void WindowedMapRecallModule::visualizeImpl(QueryCache &,
                                            const Graph::ConstPtr &) {}

}  // namespace lidar
}  // namespace vtr