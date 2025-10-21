// Copyright 2025, Autonomous Space Robotics Lab (ASRL)
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
 * \file preprocessing_module_curvature.cpp
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/preprocessing/preprocessing_module_curvature.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "vtr_lidar/features/normal.hpp"
#include "vtr_lidar/filters/voxel_downsample.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"

#include <random>

namespace vtr {
namespace lidar {

using namespace tactic;

auto PreprocessingCurvatureModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);

  config->crop_range = node->declare_parameter<float>(param_prefix + ".crop_range", config->crop_range);
  config->num_sample = node->declare_parameter<int>(param_prefix + ".num_sample", config->num_sample);
  config->k_neighbors = node->declare_parameter<int>(param_prefix + ".k_neighbors", config->k_neighbors);

  config->downsample_teach_map = node->declare_parameter<bool>(param_prefix + ".downsample_teach_map", config->downsample_teach_map);
  config->plane_voxel_size = node->declare_parameter<float>(param_prefix + ".plane_voxel_size", config->plane_voxel_size);
  config->feature_voxel_size = node->declare_parameter<float>(param_prefix + ".feature_voxel_size", config->feature_voxel_size);

  config->t = node->declare_parameter<float>(param_prefix + ".t", config->t);
  config->d_prime = node->declare_parameter<float>(param_prefix + ".d_prime", config->d_prime);
  config->ground_plane_threshold = node->declare_parameter<float>(param_prefix + ".ground_plane_threshold", config->ground_plane_threshold);
  config->min_points_threshold = node->declare_parameter<int>(param_prefix + ".min_points_threshold", config->min_points_threshold);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

struct UnionFind {
  std::vector<std::atomic<int>> parent;

  UnionFind(size_t n) : parent(n) {
    for (size_t i = 0; i < n; ++i) parent[i] = i;
  }

  int find(int x) {
    int p = parent[x];
    if (p != x) {
      int r = find(p);
      parent[x] = r;
      return r;
    }
    return x;
  }

  void unite(int x, int y) {
    while (true) {
      x = find(x);
      y = find(y);
      if (x == y) return;
      if (x < y) {
        if (parent[y].compare_exchange_strong(y, x)) return;
      } else {
        if (parent[x].compare_exchange_strong(x, y)) return;
      }
    }
  }
};
 
void PreprocessingCurvatureModule::compute_curvature(pcl::PointCloud<PointWithInfo>::Ptr& cloud) {
  if (!cloud || cloud->empty()) return;

  // Build KD-tree
  NanoFLANNAdapter<PointWithInfo> adapter(*cloud);
  KDTreeParams tree_params(10);
  auto kdtree = std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  std::vector<std::vector<size_t>> idx_buf(config_->num_threads, std::vector<size_t>(config_->k_neighbors));
  std::vector<std::vector<float>> dist_buf(config_->num_threads, std::vector<float>(config_->k_neighbors));

#pragma omp parallel for schedule(static, 64) num_threads(config_->num_threads)
  for (size_t i = 0; i < cloud->size(); ++i) {
    int tid = omp_get_thread_num();
    auto &idx = idx_buf[tid];
    auto &dists = dist_buf[tid];

    const auto& query_pt = (*cloud)[i];
    size_t found = kdtree->knnSearch(&query_pt.x, config_->k_neighbors, idx.data(), dists.data());
    if (found < 6) {
      (*cloud)[i].curvature = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    std::vector<int> idx_int(idx.begin(), idx.end());
    const pcl::PointCloud<PointWithInfo> points(*cloud, idx_int);

    // Compute centroid and covariance
    Eigen::Matrix3f covariance;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(points, centroid);
    pcl::computeCovarianceMatrix(points, centroid, covariance);

    // Eigen decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covariance);
    if (eig.info() != Eigen::Success) {
      (*cloud)[i].curvature = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    // Use smallest eigenvector
    Eigen::Vector3f normal = eig.eigenvectors().col(0);

    // Orient normal to face lidar origin
    Eigen::Vector3f p(query_pt.x, query_pt.y, query_pt.z);
    if (normal.dot(p) > 0) normal = -normal;
    normal.normalize();

    (*cloud)[i].getNormalVector3fMap() = normal;

    float lambda0 = eig.eigenvalues()(0);
    float lambda1 = eig.eigenvalues()(1);
    float lambda2 = eig.eigenvalues()(2);
    (*cloud)[i].normal_score = 1.0f - lambda0 / (lambda2 + 1e-9f);

    // Early planar skip
    if (lambda0 / (lambda1 + 1e-9f) < 1e-3f) {
      (*cloud)[i].curvature = 0.0f;
      continue;
    }

    ///  Quadratic surface fitting (following He 2005)
    // Build tangent basis (u,v)
    Eigen::Vector3f u_dir = (eig.eigenvectors().col(1)).normalized();
    Eigen::Vector3f v_dir = (eig.eigenvectors().col(2)).normalized();

    const size_t n = found;
    Eigen::VectorXf us(n), vs(n), zs(n);
    for (size_t j = 0; j < n; ++j) {
      Eigen::Vector3f q((*cloud)[idx[j]].x, (*cloud)[idx[j]].y, (*cloud)[idx[j]].z);
      Eigen::Vector3f diff = q - p;
      float z = diff.dot(normal);
      Eigen::Vector3f plane_proj = diff - z * normal;
      us(j) = plane_proj.dot(u_dir);
      vs(j) = plane_proj.dot(v_dir);
      zs(j) = z;
    }

    Eigen::MatrixXf A(n, 6);
    A.col(0) = us.array().square();
    A.col(1) = us.array() * vs.array();
    A.col(2) = vs.array().square();
    A.col(3) = us;
    A.col(4) = vs;
    A.col(5).setOnes();

    Eigen::VectorXf coeffs = A.colPivHouseholderQr().solve(zs);

    float A_ = coeffs(0), B_ = coeffs(1), C_ = coeffs(2);
    float D_ = coeffs(3), E_ = coeffs(4);
    float z_uu = 2.0f * A_, z_uv = B_, z_vv = 2.0f * C_;
    float z_u0 = D_, z_v0 = E_;

    float E_ff = 1.0f + z_u0 * z_u0;
    float F_ff = z_u0 * z_v0;
    float G_ff = 1.0f + z_v0 * z_v0;

    float L = z_uu, M = z_uv, N = z_vv;
    float denom = (E_ff * G_ff - F_ff * F_ff);
    if (std::abs(denom) < 1e-12f) {
      (*cloud)[i].curvature = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
    float K = (L * N - M * M) / denom;
    (*cloud)[i].curvature = K;
  }
}

void PreprocessingCurvatureModule::cluster_curvature(
    const pcl::PointCloud<PointWithInfo>::Ptr& cloud,
    std::vector<int>& cluster_ids) {

  if (!cloud || cloud->empty()) return;

  cluster_ids.assign(cloud->size(), -1);  // initialize

  const float flat_thresh = config_->ground_plane_threshold;
  std::vector<size_t> flat_inds, nonflat_inds;

  // Separate flat and non-flat points
  for (size_t i = 0; i < cloud->size(); ++i) {
    float curv = (*cloud)[i].curvature;
    if (!std::isfinite(curv)) continue;
    if (std::fabs(curv) < flat_thresh)
      flat_inds.push_back(i);
    else
      nonflat_inds.push_back(i);
  }

  // Early exit if no non-flat points
  if (nonflat_inds.empty()) {
    for (size_t i = 0; i < cloud->size(); ++i)
      cluster_ids[i] = 0;
    return;
  }

  // Build KD-tree for non-flat points
  pcl::PointCloud<PointWithInfo> nonflat_cloud;
  nonflat_cloud.resize(nonflat_inds.size());
  for (size_t k = 0; k < nonflat_inds.size(); ++k)
    nonflat_cloud[k] = (*cloud)[nonflat_inds[k]];

  NanoFLANNAdapter<PointWithInfo> adapter(nonflat_cloud);
  KDTreeParams tree_params(10);
  KDTree<PointWithInfo> kdtree(3, adapter, tree_params);
  kdtree.buildIndex();

  // Union-Find for clustering
  UnionFind uf(nonflat_inds.size());

  // Connect neighbors within distance d_prime
  const size_t K = 50; // max neighbors to search per point
#pragma omp parallel for schedule(dynamic)
  for (size_t i = 0; i < nonflat_cloud.size(); ++i) {
    std::vector<size_t> idx(K);
    std::vector<float> dist(K);
    size_t found = kdtree.knnSearch(&nonflat_cloud[i].x, K, idx.data(), dist.data());
    for (size_t j = 0; j < found; ++j) {
      if (idx[j] != i && dist[j] < config_->d_prime * config_->d_prime) { // squared distance
        uf.unite(i, idx[j]);
      }
    }
  }

  // Assign cluster labels
  std::unordered_map<int, int> root_to_label;
  int current_label = 1; // 0 = flat
  for (size_t k = 0; k < nonflat_inds.size(); ++k) {
    int root = uf.find(k);
    if (root_to_label.find(root) == root_to_label.end())
      root_to_label[root] = current_label++;
    cluster_ids[nonflat_inds[k]] = root_to_label[root];
  }

  // Assign flat points to cluster 0
  for (size_t i : flat_inds)
    cluster_ids[i] = 0;

  CLOG(INFO, "lidar.preprocessing.curvature")
    << "Clustered " << cloud->size() << " points into "
    << current_label << " clusters (including flat cluster 0).";
}

void PreprocessingCurvatureModule::remove_small_clusters(
  pcl::PointCloud<PointWithInfo>::Ptr& cloud,
  std::vector<int>& cluster_ids) {
  if (!cloud || cloud->empty()) return;

  // Count cluster sizes
  std::unordered_map<int, int> cluster_sizes;
  for (int id : cluster_ids)
    if (id >= 0) cluster_sizes[id]++;

  std::unordered_set<int> valid_labels;
  for (auto &[label, size] : cluster_sizes) 
    if (size >= config_->min_points_threshold)
      valid_labels.insert(label);

  for (auto &id : cluster_ids)
    if (valid_labels.find(id) == valid_labels.end())
      id = -1;
}

void PreprocessingCurvatureModule::run_(QueryCache &qdata0, OutputCache &,
                                        const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  using Stopwatch = common::timing::Stopwatch<>;
  std::vector<std::unique_ptr<Stopwatch>> timer;
  std::vector<std::string> clock_str;
  clock_str.push_back("initialization ......... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("curvature .............. ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("clustering ............. ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("remove outliers ........ ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("downsample ............. ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));

  // Create a publisher for visualization if needed
  if (config_->visualize && !publisher_initialized_) {
    filtered_pub_ = qdata.node->create_publisher<PointCloudMsg>("filtered_point_cloud", 5);
    publisher_initialized_ = true;
  }

  // Get input point cloud
  const auto point_cloud = qdata.raw_point_cloud.ptr();

  if (point_cloud->empty()) {
    std::string err{"Empty point cloud."};
    CLOG(ERROR, "lidar.preprocessing_curvature") << err;
    throw std::runtime_error{err};
  }

  timer[0]->start();
  CLOG(DEBUG, "lidar.preprocessing_curvature")
      << "raw point cloud size: " << point_cloud->size();

  auto filtered_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*point_cloud);

  // Range cropping
  {
    std::vector<int> indices;
    for (size_t i = 0; i < filtered_point_cloud->size(); ++i) {
      if ((*filtered_point_cloud)[i].rho < config_->crop_range)
        indices.push_back(static_cast<int>(i));
    }
    *filtered_point_cloud = pcl::PointCloud<PointWithInfo>(*filtered_point_cloud, indices);
  }

  CLOG(DEBUG, "lidar.preprocessing_curvature")
      << "range cropped point cloud size: " << filtered_point_cloud->size();

  // Downsample if too large for efficiency
  if (filtered_point_cloud->size() > config_->num_sample) {
    std::vector<int> indices(filtered_point_cloud->size());
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), std::mt19937{std::random_device{}()});
    indices.resize(config_->num_sample);
    std::sort(indices.begin(), indices.end());
    *filtered_point_cloud = pcl::PointCloud<PointWithInfo>(*filtered_point_cloud, indices);
  }
  timer[0]->stop();

  // estimate curvature
  timer[1]->start();
  compute_curvature(filtered_point_cloud);
  timer[1]->stop();

  // curvature-based clustering
  timer[2]->start();
  std::vector<int> cluster_ids;
  cluster_curvature(filtered_point_cloud, cluster_ids);
  timer[2]->stop();

  // remove small clusters
  timer[3]->start();
  remove_small_clusters(filtered_point_cloud, cluster_ids);
  timer[3]->stop();

  // extract labels
  std::vector<int> labels(filtered_point_cloud->size());
  for (size_t i = 0; i < filtered_point_cloud->size(); ++i)
    labels[i] =  cluster_ids[i];
  CLOG(DEBUG, "lidar.preprocessing_curvature")
      << "number of clusters after removing small ones: " << (*std::max_element(labels.begin(), labels.end())) + 1;

  // process each unique cluster label only once
  timer[4]->start();
  std::set<int> unique_labels(labels.begin(), labels.end());
  unique_labels.erase(-1); // Remove invalid label if present

  bool is_repeating = false;
  const auto &pipeline_mode = *qdata.pipeline_mode;
  if (pipeline_mode == PipelineMode::RepeatMetricLoc || pipeline_mode == PipelineMode::RepeatFollow) {
    is_repeating = true;
  }
  CLOG(DEBUG, "lidar.preprocessing_curvature")
      << "is repeating: " << (is_repeating ? "true" : "false")
      << ", config_->downsample_teach_map: " << (config_->downsample_teach_map ? "true" : "false");

  auto final_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>();
  if (is_repeating || config_->downsample_teach_map) {
    // If repeating or downsample_teach_map is true, use different voxel sizes for planar vs feature-rich clusters
    CLOG(DEBUG, "lidar.preprocessing_curvature")
        << "Downsampling each cluster separately using plane_voxel_size: " << config_->plane_voxel_size
        << " and feature_voxel_size: " << config_->feature_voxel_size;
    for (auto label : unique_labels) {
      std::vector<int> cluster_indices;
      for (size_t i = 0; i < labels.size(); ++i)
        if (labels[i] == label) cluster_indices.push_back(static_cast<int>(i));

      if (cluster_indices.empty()) continue;

      pcl::PointCloud<PointWithInfo>::Ptr cluster_cloud(new pcl::PointCloud<PointWithInfo>);
      pcl::copyPointCloud(*filtered_point_cloud, cluster_indices, *cluster_cloud);

      double mean_curvature = 0.0;
      for (const auto& pt : *cluster_cloud)
        mean_curvature += pt.curvature;
      mean_curvature /= static_cast<double>(cluster_cloud->size());

      CLOG(DEBUG, "lidar.preprocessing_curvature")
          << "cluster " << label << " size: " << cluster_cloud->size()
          << ", mean curvature: " << mean_curvature;

      float voxel_size = (std::abs(mean_curvature) < config_->ground_plane_threshold) 
                                                      ? config_->plane_voxel_size
                                                      : config_->feature_voxel_size;

      voxelDownsample(*cluster_cloud, voxel_size);
      *final_cloud += *cluster_cloud;
    }
  } else {
    // If mapping, downsample whole cloud using feature_voxel_size
    CLOG(DEBUG, "lidar.preprocessing_curvature")
        << "Are you building the map? Downsampling whole point cloud using feature_voxel_size: " << config_->feature_voxel_size;
    voxelDownsample(*filtered_point_cloud, config_->feature_voxel_size);
    final_cloud = filtered_point_cloud;
  }
  timer[4]->stop();

  // Dump timing info
  CLOG(DEBUG, "lidar.preprocessing_curvature") << "Dump timing info inside loop: ";
  for (size_t i = 0; i < clock_str.size(); i++) {
    CLOG(DEBUG, "lidar.preprocessing_curvature") << "  " << clock_str[i] << timer[i]->count();
  }

  filtered_point_cloud = final_cloud;

  CLOG(DEBUG, "lidar.preprocessing_curvature")
      << "final downsampled point cloud size: " << filtered_point_cloud->size();

  if (config_->visualize) {
    PointCloudMsg msg;
    pcl::toROSMsg(*filtered_point_cloud, msg);
    msg.header.frame_id = "lidar";
    filtered_pub_->publish(msg);
  }

  qdata.preprocessed_point_cloud = filtered_point_cloud;
}
}  // namespace lidar
}  // namespace vtr