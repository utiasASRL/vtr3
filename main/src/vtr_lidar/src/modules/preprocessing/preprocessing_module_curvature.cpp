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
  config->plane_voxel_size = node->declare_parameter<float>(param_prefix + ".plane_voxel_size", config->plane_voxel_size);
  config->feature_voxel_size = node->declare_parameter<float>(param_prefix + ".feature_voxel_size", config->feature_voxel_size);

  config->t = node->declare_parameter<float>(param_prefix + ".t", config->t);
  config->d_prime = node->declare_parameter<float>(param_prefix + ".d_prime", config->d_prime);
  config->ground_plane_threshold = node->declare_parameter<float>(param_prefix + ".ground_plane_threshold", config->ground_plane_threshold);

  config->min_points_threshold = node->declare_parameter<int>(param_prefix + ".min_points_threshold", config->min_points_threshold);
  config->k_neighbors = node->declare_parameter<int>(param_prefix + ".k_neighbors", config->k_neighbors);

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

  // Build nanoflann KD-tree for neighbor search
  NanoFLANNAdapter<PointWithInfo> adapter(*cloud);
  KDTreeParams tree_params(10); // max leaf
  auto kdtree = std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  std::vector<std::vector<size_t>> idx_buf(config_->num_threads, std::vector<size_t>(config_->k_neighbors));
  std::vector<std::vector<float>> dist_buf(config_->num_threads, std::vector<float>(config_->k_neighbors));
  std::vector<Eigen::VectorXf> coeffs_buf(config_->num_threads);

  // For each point, estimate curvature using quadratic surface fit (He 2005)
#pragma omp parallel for schedule(static, 64) num_threads(config_->num_threads)
  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto& query_pt = (*cloud)[i];
    int tid = omp_get_thread_num();
    auto &idx = idx_buf[tid];
    auto &dists = dist_buf[tid];

    size_t found = kdtree->knnSearch(&query_pt.x, config_->k_neighbors, idx.data(), dists.data());
    if (found < 6) {
      (*cloud)[i].curvature = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    // Gather neighbors
    Eigen::MatrixXf neighbors(found, 3);
    for (size_t j = 0; j < found; ++j) {
      neighbors.row(j) = Eigen::Vector3f(
          (*cloud)[idx[j]].x, (*cloud)[idx[j]].y, (*cloud)[idx[j]].z);
    }
    Eigen::Vector3f p(query_pt.x, query_pt.y, query_pt.z);

    // Center neighbors at query point
    Eigen::MatrixXf B_centered = neighbors.rowwise() - p.transpose();
  
    // Compute centroid
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (size_t j = 0; j < found; ++j) {
      centroid += Eigen::Vector3f(
          (*cloud)[idx[j]].x, (*cloud)[idx[j]].y, (*cloud)[idx[j]].z);
    }
    centroid /= static_cast<float>(found);

    // Compute covariance (fixed-size 3×3)
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (size_t j = 0; j < found; ++j) {
      Eigen::Vector3f diff = Eigen::Vector3f(
          (*cloud)[idx[j]].x, (*cloud)[idx[j]].y, (*cloud)[idx[j]].z) - centroid;
      cov += diff * diff.transpose();
    }
    cov /= static_cast<float>(found - 1);

    // Eigen decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
    if (eig.info() != Eigen::Success) {
      (*cloud)[i].curvature = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
    (*cloud)[i].getNormalVector3fMap() = 
        eig.eigenvectors().col(0).dot(p) > 0
            ? -Eigen::Vector3f(eig.eigenvectors().col(0))
            : Eigen::Vector3f(eig.eigenvectors().col(0));
    (*cloud)[i].normal_score = 1.0f - eig.eigenvalues()(0) / (eig.eigenvalues()(2) + 1e-9f);

    Eigen::Vector3f normal = (*cloud)[i].getNormalVector3fMap().normalized();

    // early planar skip
    float lambda0 = eig.eigenvalues()(0), lambda1 = eig.eigenvalues()(1);
    if (lambda0 / (lambda1 + 1e-12f) < 1e-3f) {
      (*cloud)[i].curvature = 0.0f; // or small value
      continue;
    }

    // --- Find farthest projected neighbor to define u ---
    float max_dist = 0.0f;
    Eigen::Vector3f u_dir(1.0f, 0.0f, 0.0f);  // fallback
    for (size_t j = 0; j < found; ++j) {
      Eigen::Vector3f vec = Eigen::Vector3f(
          (*cloud)[idx[j]].x, (*cloud)[idx[j]].y, (*cloud)[idx[j]].z) - p;

      // Remove normal component (projection onto tangent plane)
      float z = vec.dot(normal);
      Eigen::Vector3f plane_vec = vec - z * normal;
      float dist = plane_vec.squaredNorm();

      if (dist > max_dist) {
        max_dist = dist;
        u_dir = plane_vec;
      }
    }

    float u_norm = u_dir.norm();
    if (u_norm < 1e-12f) {
      (*cloud)[i].curvature = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
    Eigen::Vector3f u = u_dir / u_norm;
    Eigen::Vector3f v = normal.cross(u).normalized();
    if (v.norm() < 1e-12f) {
      (*cloud)[i].curvature = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    // --- Compute local (u, v, z) coordinates without matrix allocations ---
    Eigen::VectorXf us(found), vs(found), zs(found);
    for (size_t j = 0; j < found; ++j) {
      Eigen::Vector3f vec = Eigen::Vector3f(
          (*cloud)[idx[j]].x, (*cloud)[idx[j]].y, (*cloud)[idx[j]].z) - p;
      float z = vec.dot(normal);
      Eigen::Vector3f plane_vec = vec - z * normal;

      us(j) = plane_vec.dot(u);
      vs(j) = plane_vec.dot(v);
      zs(j) = z;
    }

    // Design matrix for quadratic fit: z = A u^2 + B u v + C v^2 + D u + E v + F
    Eigen::MatrixXf A_design(found, 6);
    A_design.col(0) = us.array().square();
    A_design.col(1) = us.array() * vs.array();
    A_design.col(2) = vs.array().square();
    A_design.col(3) = us;
    A_design.col(4) = vs;
    A_design.col(5).setOnes();

    // Least squares fit
    Eigen::VectorXf coeffs = A_design.colPivHouseholderQr().solve(zs);
    float A_coef = coeffs(0), B_coef = coeffs(1), C_coef = coeffs(2);
    float D_coef = coeffs(3), E_coef = coeffs(4);

    // Second derivatives at origin
    float z_uu = 2.0f * A_coef;
    float z_uv = B_coef;
    float z_vv = 2.0f * C_coef;

    // First derivatives at origin
    float z_u0 = D_coef;
    float z_v0 = E_coef;

    // First fundamental form
    float E_ff = 1.0f + z_u0 * z_u0;
    float F_ff = z_u0 * z_v0;
    float G_ff = 1.0f + z_v0 * z_v0;

    // Second fundamental form
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

void PreprocessingCurvatureModule::cluster_curvature(const pcl::PointCloud<PointWithInfo>::Ptr& cloud) {
  if (!cloud || cloud->empty()) return;

  const float flat_thresh = config_->ground_plane_threshold;

  // separate flat and non-flat points
  std::vector<size_t> flat_inds, nonflat_inds;
  flat_inds.reserve(cloud->size());
  nonflat_inds.reserve(cloud->size());

  for (size_t i = 0; i < cloud->size(); ++i) {
    float curv = (*cloud)[i].curvature;
    if (!std::isfinite(curv)) continue;
    if (std::fabs(curv) < flat_thresh)
      flat_inds.push_back(i);
    else
      nonflat_inds.push_back(i);
  }

  if (nonflat_inds.empty()) {
    // All flat
    for (size_t i = 0; i < cloud->size(); ++i)
      (*cloud)[i].cluster_id = 0;
    CLOG(DEBUG, "lidar.preprocessing.curvature") << "All points are flat; assigned to single cluster 0.";
    return;
  }

  // label as concave, flat, or convex
  std::vector<float> a(nonflat_inds.size(), 0.0f);
  for (size_t k = 0; k < nonflat_inds.size(); ++k) {
    size_t i = nonflat_inds[k];
    float curv = (*cloud)[i].curvature;
    if (curv < -config_->ground_plane_threshold)
      a[k] = -config_->t;
    else if (std::abs(curv) <= config_->ground_plane_threshold)
      a[k] = 0.0f;
    else if (curv > config_->ground_plane_threshold)
      a[k] = config_->t;
  }

  // embed non-flat into R^{n+1}
  std::vector<Eigen::Vector4f> X_embedded(nonflat_inds.size());
  for (size_t k = 0; k < nonflat_inds.size(); ++k) {
    size_t i = nonflat_inds[k];
    X_embedded[k] = Eigen::Vector4f(
      (*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z, a[k]);
  }

  // build adjacency
  std::vector<std::vector<size_t>> adjacency(nonflat_inds.size());
  for (size_t i = 0; i < nonflat_inds.size(); ++i) {
    for (size_t j = i + 1; j < nonflat_inds.size(); ++j) {
      float dist = (X_embedded[i] - X_embedded[j]).norm();
      if (dist < config_->d_prime) {
        adjacency[i].push_back(j);
        adjacency[j].push_back(i);
      }
    }
  }

  // connected components as clusters
  std::vector<int> labels(nonflat_inds.size(), -1);
  int current_label = 1;  // start from 1 since 0 = flat cluster

  for (size_t i = 0; i < nonflat_inds.size(); ++i) {
    if (labels[i] != -1) continue;
    std::queue<size_t> q;
    q.push(i);
    labels[i] = current_label;
    while (!q.empty()) {
      size_t idx = q.front(); q.pop();
      for (size_t nbr : adjacency[idx]) {
        if (labels[nbr] == -1) {
          labels[nbr] = current_label;
          q.push(nbr);
        }
      }
    }
    current_label++;
  }

  // assign cluster ids
  for (size_t i = 0; i < cloud->size(); ++i)
    (*cloud)[i].cluster_id = -1;

  // flat points → cluster 0
  for (size_t i : flat_inds)
    (*cloud)[i].cluster_id = 0;

  // non-flat points → cluster 1, 2, ...
  for (size_t k = 0; k < nonflat_inds.size(); ++k)
    (*cloud)[nonflat_inds[k]].cluster_id = labels[k];

  CLOG(INFO, "lidar.preprocessing.curvature")
    << "Clustered " << cloud->size() << " points into "
    << current_label << " clusters (including flat cluster 0).";
}

// void PreprocessingCurvatureModule::cluster_curvature(const pcl::PointCloud<PointWithInfo>::Ptr& cloud) {
//     if (!cloud || cloud->empty()) return;

//     const size_t N = cloud->size();
//     float radius = config_->d_prime;
//     float curv_thresh = config_->ground_plane_threshold;

//     NanoFLANNAdapter<PointWithInfo> adapter(*cloud);
//     KDTree<PointWithInfo> tree(3, adapter, KDTreeParams(10));
//     tree.buildIndex();

//     UnionFind uf(N);

// #pragma omp parallel for schedule(dynamic, 64) num_threads(config_->num_threads)
//     for (size_t i = 0; i < N; ++i) {
//         const float query_pt[3] = {(*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z};
//         std::vector<size_t> neighbors;
//         std::vector<float> dists;

//         NanoFLANNRadiusResultSet<float, size_t> resultSet(radius, dists, neighbors);
//         tree.findNeighbors(resultSet, query_pt, KDTreeSearchParams());

//         float curv_i = (*cloud)[i].curvature;
//         if (!std::isfinite(curv_i)) continue;

//         for (auto j : neighbors) {
//             if (j == i) continue;
//             float curv_j = (*cloud)[j].curvature;
//             if (!std::isfinite(curv_j)) continue;

//             if (std::fabs(curv_i - curv_j) < curv_thresh) {
//                 uf.unite(i, j);  // thread-safe union
//             }
//         }
//     }

//     // Assign cluster labels
//     std::vector<int> labels(N);
//     std::unordered_map<int, int> cluster_map;
//     int next_label = 0;

//     for (size_t i = 0; i < N; ++i) {
//         int root = uf.find(i);
//         int label;
//         auto it = cluster_map.find(root);
//         if (it == cluster_map.end()) {
//             label = next_label++;
//             cluster_map[root] = label;
//         } else {
//             label = it->second;
//         }
//         labels[i] = label;
//     }

//     for (size_t i = 0; i < N; ++i) {
//         (*cloud)[i].cluster_id = labels[i];
//     }

//     CLOG(INFO, "lidar.preprocessing.curvature")
//         << "Clustered " << N << " points into " << next_label << " clusters.";
// }


void PreprocessingCurvatureModule::remove_small_clusters(pcl::PointCloud<PointWithInfo>::Ptr& cloud) {
  if (!cloud || cloud->empty()) return;

  // Count cluster sizes
  std::unordered_map<int, int> cluster_sizes;
  for (const auto& pt : *cloud) cluster_sizes[pt.cluster_id]++;

  // Find valid cluster ids
  std::unordered_set<int> valid_labels;
  for (const auto& [label, size] : cluster_sizes) {
    if (size >= config_->min_points_threshold)
      valid_labels.insert(label);
  }

  // Mark invalid clusters
  for (auto& pt : *cloud) {
    if (valid_labels.find(pt.cluster_id) == valid_labels.end())
      pt.cluster_id = -1;
  }
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
  cluster_curvature(filtered_point_cloud);
  timer[2]->stop();

  // remove small clusters
  timer[3]->start();
  remove_small_clusters(filtered_point_cloud);
  timer[3]->stop();

  // extract labels
  std::vector<int> labels(filtered_point_cloud->size());
  for (size_t i = 0; i < filtered_point_cloud->size(); ++i)
    labels[i] = (*filtered_point_cloud)[i].cluster_id;
  CLOG(DEBUG, "lidar.preprocessing_curvature")
      << "number of clusters after removing small ones: " << (*std::max_element(labels.begin(), labels.end())) + 1;

  // process each unique cluster label only once
  timer[4]->start();
  std::set<int> unique_labels(labels.begin(), labels.end());
  unique_labels.erase(-1); // Remove invalid label if present

  // downsample each cluster based on its mean curvature
  auto final_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>();
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