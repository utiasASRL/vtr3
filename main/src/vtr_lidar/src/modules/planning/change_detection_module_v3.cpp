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
 * \file change_detection_module_v3.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/planning/change_detection_module_v3.hpp"

#include "pcl/features/normal_3d.h"

#include "vtr_lidar/data_types/costmap.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

namespace {

template <class PointT>
void computeCentroidAndNormal(const pcl::PointCloud<PointT> &points,
                              Eigen::Vector3f &centroid,
                              Eigen::Vector3f &normal, float &roughness) {
  // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
  Eigen::Vector4f centroid_homo;
  // Placeholder for the 3x3 covariance matrix at each surface patch
  Eigen::Matrix3f cov;

  // Estimate the XYZ centroid
  pcl::compute3DCentroid(points, centroid_homo);

  // Compute the 3x3 covariance matrix
  pcl::computeCovarianceMatrix(points, centroid_homo, cov);

  // Compute pca
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
  es.compute(cov);

  // save results
  centroid = centroid_homo.head<3>();
  normal = es.eigenvectors().col(0);
  roughness = es.eigenvalues()(0);  // variance
}

template <typename PointT>
class DetectChangeOp {
 public:
  DetectChangeOp(const pcl::PointCloud<PointT> &points, const float &d0,
                 const float &d1)
      : d0_(d0), d1_(d1), adapter_(points) {
    /// create kd-tree of the point cloud for radius search
    kdtree_ = std::make_unique<KDTree<PointT>>(2, adapter_,
                                               KDTreeParams(/* max leaf */ 10));
    kdtree_->buildIndex();
    // search params setup
    search_params_.sorted = false;
  }

  void operator()(const Eigen::Vector2f &q, float &v) const {
    size_t ind;
    float dist;
    KDTreeResultSet result_set(1);
    result_set.init(&ind, &dist);
    kdtree_->findNeighbors(result_set, q.data(), search_params_);

    // update the value of v
    dist = std::sqrt(dist);  // convert to distance
    v = std::max(1 - (dist - d1_) / d0_, 0.0f);
    v = std::min(v, 0.9f);  // 1 is bad for visualization
  }

 private:
  const float d0_;
  const float d1_;

  KDTreeSearchParams search_params_;
  NanoFLANNAdapter<PointT> adapter_;
  std::unique_ptr<KDTree<PointT>> kdtree_;
};

}  // namespace

using namespace tactic;

auto ChangeDetectionModuleV3::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // change detection
  config->detection_range = node->declare_parameter<float>(param_prefix + ".detection_range", config->detection_range);
  config->search_radius = node->declare_parameter<float>(param_prefix + ".search_radius", config->search_radius);
  config->negprob_threshold = node->declare_parameter<float>(param_prefix + ".negprob_threshold", config->negprob_threshold);
  // prior on roughness
  config->use_prior = node->declare_parameter<bool>(param_prefix + ".use_prior", config->use_prior);
  config->alpha0 = node->declare_parameter<float>(param_prefix + ".alpha0", config->alpha0);
  config->beta0 = node->declare_parameter<float>(param_prefix + ".beta0", config->beta0);
  // support
  config->use_support_filtering = node->declare_parameter<bool>(param_prefix + ".use_support_filtering", config->use_support_filtering);
  config->support_radius = node->declare_parameter<float>(param_prefix + ".support_radius", config->support_radius);
  config->support_variance = node->declare_parameter<float>(param_prefix + ".support_variance", config->support_variance);
  config->support_threshold = node->declare_parameter<float>(param_prefix + ".support_threshold", config->support_threshold);
  // cost map
  config->costmap_history_size = node->declare_parameter<int>(param_prefix + ".costmap_history_size", config->costmap_history_size);
  config->resolution = node->declare_parameter<float>(param_prefix + ".resolution", config->resolution);
  config->size_x = node->declare_parameter<float>(param_prefix + ".size_x", config->size_x);
  config->size_y = node->declare_parameter<float>(param_prefix + ".size_y", config->size_y);
  config->influence_distance = node->declare_parameter<float>(param_prefix + ".influence_distance", config->influence_distance);
  config->minimum_distance = node->declare_parameter<float>(param_prefix + ".minimum_distance", config->minimum_distance);
  // general
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void ChangeDetectionModuleV3::run_(QueryCache &qdata0, OutputCache &output0,
                                   const Graph::Ptr & /* graph */,
                                   const TaskExecutor::Ptr & /* executor */) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  auto &output = dynamic_cast<LidarOutputCache &>(output0);

  /// visualization setup
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    scan_pub_ = qdata.node->create_publisher<PointCloudMsg>("change_detection_scan", 5);
    costmap_pub_ = qdata.node->create_publisher<OccupancyGridMsg>("change_detection_costmap", 5);
    filtered_costmap_pub_ = qdata.node->create_publisher<OccupancyGridMsg>("filtered_change_detection_costmap", 5);
    costpcd_pub_ = qdata.node->create_publisher<PointCloudMsg>("change_detection_costpcd", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  // inputs
  const auto &stamp = *qdata.stamp;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &vid_loc = *qdata.vid_loc;
  const auto &sid_loc = *qdata.sid_loc;
  const auto &T_r_v_loc = *qdata.T_r_v_loc;
  const auto &points = *qdata.undistorted_point_cloud;
  const auto &submap_loc = *qdata.submap_loc;
  const auto &map_point_cloud = submap_loc.point_cloud();
  const auto &T_v_m_loc = *qdata.T_v_m_loc;

  // clang-format off
  CLOG(INFO, "lidar.change_detection") << "Change detection for lidar scan at stamp: " << stamp;

  // filter out points that are too far away
  std::vector<int> query_indices;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto &pt = points.at(i);
    if (pt.getVector3fMap().norm() < config_->detection_range)
      query_indices.emplace_back(i);
  }
  pcl::PointCloud<PointWithInfo> query_points(points, query_indices);

  // Eigen matrix of original data (only shallow copy of ref clouds)
  const auto query_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto query_norms_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  // retrieve the pre-processed scan and convert it to the local map frame
  pcl::PointCloud<PointWithInfo> aligned_points(query_points);
  auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  const auto T_m_s = (T_s_r * T_r_v_loc * T_v_m_loc).inverse().matrix();
  aligned_mat = T_m_s.cast<float>() * query_mat;
  aligned_norms_mat = T_m_s.cast<float>() * query_norms_mat;

  // create kd-tree of the map
  NanoFLANNAdapter<PointWithInfo> adapter(map_point_cloud);
  KDTreeSearchParams search_params;
  KDTreeParams tree_params(10 /* max leaf */);
  auto kdtree = std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  std::vector<long unsigned> nn_inds(aligned_points.size());
  std::vector<float> nn_dists(aligned_points.size(), -1.0f);
  // compute nearest neighbors and point to point distances
  for (size_t i = 0; i < aligned_points.size(); i++) {
    KDTreeResultSet result_set(1);
    result_set.init(&nn_inds[i], &nn_dists[i]);
    kdtree->findNeighbors(result_set, aligned_points[i].data, search_params);
  }
  // compute point to plane distance
  const auto sq_search_radius = config_->search_radius * config_->search_radius;
  std::vector<float> roughnesses(aligned_points.size(), 0.0f);
  std::vector<float> num_measurements(aligned_points.size(), 0.0f);
  for (size_t i = 0; i < aligned_points.size(); i++) {
    // radius search of the closest point
    std::vector<float> dists;
    std::vector<int> indices;
    NanoFLANNRadiusResultSet<float, int> result(sq_search_radius, dists, indices);
    kdtree->radiusSearchCustomCallback(map_point_cloud[nn_inds[i]].data, result, search_params);

    // filter based on neighbors in map /// \todo parameters
    if (indices.size() < 10) continue;

    //
    num_measurements[i] = static_cast<float>(indices.size());

    // compute the planar distance
    Eigen::Vector3f centroid, normal;
    computeCentroidAndNormal(pcl::PointCloud<PointWithInfo>(map_point_cloud, indices), centroid, normal, roughnesses[i]);

    const auto diff = aligned_points[i].getVector3fMap() - centroid;
    nn_dists[i] = std::abs(diff.dot(normal));
  }

  for (size_t i = 0; i < aligned_points.size(); i++) {
    aligned_points[i].flex23 = 0.0f;
    //
    const auto cost = [&]() -> float {
      // clang-format off
      if (config_->use_prior) {
        const float alpha_n = config_->alpha0 + num_measurements[i] / 2.0f;
        const float beta_n = config_->beta0 + roughnesses[i] * num_measurements[i] / 2.0f;
        const float roughness = beta_n / alpha_n;
        const float df = 2 * alpha_n;
        const float sqdists = nn_dists[i] * nn_dists[i] / roughness;
        return -std::log(std::pow(1 + sqdists / df, -(df + 1) / 2));
      } else {
        const float roughness = roughnesses[i];
        return (nn_dists[i] * nn_dists[i]) / (2 * roughness) + std::log(std::sqrt(roughness));
      }
      // clang-format on
    }();
    //
    if (nn_dists[i] < 0.0 || (cost > config_->negprob_threshold))
      aligned_points[i].flex23 = 1.0f;
  }

  // add support region
  if (config_->use_support_filtering) {
    // create kd-tree of the aligned points
    NanoFLANNAdapter<PointWithInfo> query_adapter(aligned_points);
    KDTreeSearchParams search_params;
    KDTreeParams tree_params(/* max leaf */ 10);
    auto query_kdtree =
        std::make_unique<KDTree<PointWithInfo>>(3, query_adapter, tree_params);
    query_kdtree->buildIndex();
    //
    std::vector<size_t> toremove;
    toremove.reserve(100);
    const float sq_support_radius = std::pow(config_->support_radius, 2);
    for (size_t i = 0; i < aligned_points.size(); i++) {
      // ignore non-change points
      if (aligned_points[i].flex23 == 0.0f) continue;

      //
      std::vector<std::pair<size_t, float>> inds_dists;
      inds_dists.reserve(10);
      query_kdtree->radiusSearch(aligned_points[i].data, sq_support_radius,
                                 inds_dists, search_params);
      //
      float support = 0.0f;
      for (const auto &ind_dist : inds_dists) {
        if (ind_dist.first == i) continue;
        support += aligned_points[ind_dist.first].flex23 *
                   std::exp(-ind_dist.second / (2 * config_->support_variance));
      }
      //
      if (support < config_->support_threshold) toremove.push_back(i);
    }
    // change back to non-change points
    for (const auto &i : toremove) aligned_points[i].flex23 = 0.0f;
  }

  // retrieve the pre-processed scan and convert it to the vertex frame
  pcl::PointCloud<PointWithInfo> aligned_points2(aligned_points);
  // clang-format off
  auto aligned_mat2 = aligned_points2.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat2 = aligned_points2.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  // clang-format on

  // transform to vertex frame
  aligned_mat2 = T_v_m_loc.matrix().cast<float>() * aligned_mat;
  aligned_norms_mat2 = T_v_m_loc.matrix().cast<float>() * aligned_norms_mat;

  // project to 2d and construct the grid map
  const auto costmap = std::make_shared<DenseCostMap>(
      config_->resolution, config_->size_x, config_->size_y);

  // filter out non-obstacle points
  std::vector<int> indices;
  indices.reserve(aligned_points2.size());
  for (size_t i = 0; i < aligned_points2.size(); ++i) {
    if (aligned_points2[i].flex23 > 0.5f) indices.emplace_back(i);
  }
  pcl::PointCloud<PointWithInfo> filtered_points(aligned_points2, indices);

  // update cost map based on change detection result
  DetectChangeOp<PointWithInfo> detect_change_op(
      filtered_points, config_->influence_distance, config_->minimum_distance);
  costmap->update(detect_change_op);
  // add transform to the localization vertex
  costmap->T_vertex_this() = tactic::EdgeTransform(true);
  costmap->vertex_id() = vid_loc;
  costmap->vertex_sid() = sid_loc;





  // Jordy Modifications for temporal costmap filtering (UNDER DEVELOPMENT)
  

  
  // declaration of the final costmap which we are outputting
  auto dense_costmap = std::make_shared<DenseCostMap>(config_->resolution, config_->size_x, config_->size_y);
  dense_costmap->T_vertex_this() = tactic::EdgeTransform(true);
  dense_costmap->vertex_id() = vid_loc;
  dense_costmap->vertex_sid() = sid_loc;
  
  // Create a sparse costmap and store it in a sliding window history
  const auto sparse_costmap = std::make_shared<DenseCostMap>(config_->resolution, config_->size_x, config_->size_y);
  sparse_costmap->update(detect_change_op);
  // add transform to the localization vertex
  sparse_costmap->T_vertex_this() = tactic::EdgeTransform(true);
  sparse_costmap->vertex_id() = vid_loc;
  sparse_costmap->vertex_sid() = sid_loc;

  // Get the localization chain transform (lets us transform from costmap frame to world frame):
  auto& chain = *output.chain;
  auto T_w_c  = chain.pose(sid_loc);
  auto T_c_w  = T_w_c.inverse();
  //CLOG(WARNING, "obstacle_detection.cbit") << "T_w_c: " << T_w_c; // debug

  // Initialize an unordered map to store the sparse world obstacle representations
  std::unordered_map<std::pair<float, float>, float>  sparse_world_map;

  // Filter non-obstacles
  vtr::lidar::BaseCostMap::XY2ValueMap sparse_obs_map = sparse_costmap->filter(0.01);

  // Iterate through the key value pairs, convert to a world frame unordered_map
  std::vector<std::pair<float, float>> keys;
  keys.reserve(sparse_obs_map.size());
  std::vector<float> vals;
  vals.reserve(sparse_obs_map.size());
  for(auto kv : sparse_obs_map) 
  {
    float key_x = kv.first.first;
    float key_y = kv.first.second;
    // Convert the centre of each obstacle key value pair into the world frame
    Eigen::Matrix<double, 4, 1> grid_pt({key_x+(config_->resolution/2), key_y+(config_->resolution/2), 0.0, 1});
    auto collision_pt = T_w_c * grid_pt;

    //CLOG(DEBUG, "obstacle_detection.cbit") << "Displaying Transform: " << (sparse_costmap->T_vertex_this().inverse());

    float world_key_x = floor(collision_pt[0] / config_->resolution) * config_->resolution;
    float world_key_y = floor(collision_pt[1] / config_->resolution) * config_->resolution;

    // Experimental debug! Im thinking that perhaps here these keys could be outside the legal area



    float world_value = kv.second;
    std::pair<float, float> world_keys(world_key_x, world_key_y);
    sparse_world_map.insert(std::make_pair(world_keys,world_value));

    //CLOG(DEBUG, "obstacle_detection.cbit") << "Displaying Key X: " << key_x;
    //CLOG(DEBUG, "obstacle_detection.cbit") << "Displaying Key Y: " << key_y;
    //CLOG(DEBUG, "obstacle_detection.cbit") << "Displaying World Key X: " << world_key_x;
    //CLOG(DEBUG, "obstacle_detection.cbit") << "Displaying World Key Y: " << world_key_y;

    
    keys.push_back(kv.first);
    vals.push_back(kv.second);  
  } 
  //CLOG(ERROR, "obstacle_detection.cbit") << "The size of each map is" << sparse_obs_map.size();
  //CLOG(ERROR, "obstacle_detection.cbit") << "Displaying all Keys: " << keys;
  //CLOG(ERROR, "obstacle_detection.cbit") << "Displaying all Values: " << vals;
  
  //CLOG(WARNING, "obstacle_detection.cbit") << "T_c_w " << T_c_w;
  //CLOG(WARNING, "obstacle_detection.cbit") << "T_m_s: " << T_m_s;
  //CLOG(WARNING, "obstacle_detection.cbit") << "T_s_r_inv: " << T_s_r.inverse();



  // if the costmap_history is smaller then some minimum value, just tack it on the end
  // TODO need to get rid of these magic numbers
  if (costmap_history.size() < config_->costmap_history_size)
  {
    costmap_history.push_back(sparse_world_map);
  }
  // After that point, we then do a sliding window using shift operations, moving out the oldest map and appending the newest one
  else
  {
    for (int i = 0; i < (config_->costmap_history_size-1); i++)
    {
      costmap_history[i] = costmap_history[i + 1];
    }
    costmap_history[(config_->costmap_history_size-1)] = sparse_world_map;
  

    //CLOG(WARNING, "obstacle_detection.cbit") << "costmap_history size " <<costmap_history.size();
    // Iterate through the stored costmaps and build a merged world frame obstacle costmap
    std::unordered_map<std::pair<float, float>, float>  merged_world_map = costmap_history[(config_->costmap_history_size-1)];
    //CLOG(WARNING, "obstacle_detection.cbit") << "merged world map size " <<merged_world_map.size();
    std::unordered_map<std::pair<float, float>, float>  filtered_world_map;
    for (int i = 0; i < (costmap_history.size()-1); i++)
    {
      merged_world_map.merge(costmap_history[i]);
      //CLOG(WARNING, "obstacle_detection.cbit") << "merged world map size " <<merged_world_map.size();
    }

  
    // For each key in this merged costmap, check how many of that key exist in the individual costmaps
    //std::vector<std::pair<float, float>> world_keys;
    //world_keys.reserve(merged_world_map.size());
    //std::vector<float> world_vals;
    //world_vals.reserve(merged_world_map.size());
    for(auto kv : merged_world_map) 
    {   
      //world_keys.push_back(kv.first);
      //world_vals.push_back(kv.second);  

      int key_vote_counter = 0;
      for (int i = 0; i < costmap_history.size(); i++)
      {
        if ((costmap_history[i]).find(kv.first) != (costmap_history[i]).end())
        {
          key_vote_counter = key_vote_counter + 1;
        }
      }
      
      // Store the filtered (both temporal and transient) points in a sparse filtered unordered_map
      if (key_vote_counter >= 3)
      {
        filtered_world_map.insert(std::make_pair(kv.first,kv.second));
      }
    }
  
    // Convert the filtered world map back to the current costmap frame (note, might actually just store the sparse world map, its more useful for me anyways)
    // Probably still want to convert back so we can republish it though
    // Iterate through the key value pairs, convert to a world frame unordered_map
    std::unordered_map<std::pair<float, float>, float>  filtered_loc_map;
    std::vector<std::pair<float, float>> filtered_keys;
    filtered_keys.reserve(filtered_world_map.size());
    std::vector<float> filtered_vals;
    filtered_vals.reserve(filtered_world_map.size());

    for(auto kv : filtered_world_map) 
    {
      float key_x = kv.first.first;
      float key_y = kv.first.second;
      // Convert the centre of each obstacle key value pair into the world frame
      Eigen::Matrix<double, 4, 1> grid_pt({key_x+(config_->resolution/2), key_y+(config_->resolution/2), 0.0, 1});
      auto collision_pt = T_c_w * grid_pt;

      //CLOG(DEBUG, "obstacle_detection.cbit") << "Displaying Transform: " << (sparse_costmap->T_vertex_this().inverse());

      float loc_key_x = floor(collision_pt[0] / config_->resolution) * config_->resolution;
      float loc_key_y = floor(collision_pt[1] / config_->resolution) * config_->resolution;


      // debug to check loc_key is in a valid range:


      float loc_value = kv.second;
      std::pair<float, float> loc_keys(loc_key_x, loc_key_y);
      filtered_loc_map.insert(std::make_pair(loc_keys,loc_value));
    }
 
    // Build the dense map, publish and save the results so the planner can access it
    //const auto dense_costmap = std::make_shared<DenseCostMap>(config_->resolution, config_->size_x, config_->size_y); // need to delcare this earlier

    dense_costmap->update(filtered_loc_map);
    
    
    // add transform to the localization vertex
    //dense_costmap->T_vertex_this() = tactic::EdgeTransform(true);
    //dense_costmap->vertex_id() = vid_loc;
    //dense_costmap->vertex_sid() = sid_loc;

    // debug, temporarily using my filtered costmap as the main costmap
    //auto costmap_msg = dense_costmap->toCostMapMsg();
    //costmap_msg.header.frame_id = "loc vertex frame";
    // costmap_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    //costmap_pub_->publish(costmap_msg);

    //CLOG(WARNING, "obstacle_detection.cbit") << "Successfully update the dense costmap"; 
  

    // publish the filtered occupancy grid
    auto filtered_costmap_msg = dense_costmap->toCostMapMsg();
    filtered_costmap_msg.header.frame_id = "loc vertex frame";
    // costmap_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    filtered_costmap_pub_->publish(filtered_costmap_msg);

    // Debug check that the filtered maps look okay
    vtr::lidar::BaseCostMap::XY2ValueMap dense_map = dense_costmap->filter(0.01);
    std::vector<std::pair<float, float>> keys2;
    keys2.reserve(dense_map.size());
    std::vector<float> vals2;
    vals2.reserve(dense_map.size());
    for(auto kv : dense_map) {   
      keys2.push_back(kv.first);
      vals2.push_back(kv.second);  
    } 
    //CLOG(ERROR, "obstacle_detection.cbit") << "The size of the dense map is" << dense_map.size();
    //CLOG(ERROR, "obstacle_detection.cbit") << "Displaying all Keys: " << keys2;
    //CLOG(ERROR, "obstacle_detection.cbit") << "Displaying all Values: " << vals2;
  }
  



  
  // End of Jordy's temporal filter changes





  /// publish the transformed pointcloud
  if (config_->visualize) {
    // publish the aligned points
    PointCloudMsg scan_msg;
    pcl::toROSMsg(aligned_points2, scan_msg);
    scan_msg.header.frame_id = "loc vertex frame";
    // scan_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    scan_pub_->publish(scan_msg);

    // publish the occupancy grid
    auto costmap_msg = costmap->toCostMapMsg();
    costmap_msg.header.frame_id = "loc vertex frame";
    // costmap_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    costmap_pub_->publish(costmap_msg);

    // publish the point cloud
    auto pointcloud_msg = costmap->toPointCloudMsg();
    pointcloud_msg.header.frame_id = "loc vertex frame";
    // pointcloud_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    costpcd_pub_->publish(pointcloud_msg);
  }

  /// output
  auto change_detection_costmap_ref = output.change_detection_costmap.locked();
  auto &change_detection_costmap = change_detection_costmap_ref.get();
  //change_detection_costmap = costmap;

  //Jordy debug, using experimental new costmaps instead
  if (costmap_history.size() < 10.0)
  {
    change_detection_costmap = costmap;
  }
  else
  {
    change_detection_costmap = dense_costmap; 
  }

  CLOG(INFO, "lidar.change_detection")
      << "Change detection for lidar scan at stamp: " << stamp << " - DONE";

}

}  // namespace lidar
}  // namespace vtr