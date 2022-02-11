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
 * \file normal.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */

#include <cmath>

#include "pcl/features/normal_3d.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"

#include "vtr_logging/logging.hpp"

namespace vtr {
namespace radar {

namespace normal {

template <typename PointT>
void compute2DCentroid(const pcl::PointCloud<PointT> &cloud,
                       Eigen::Vector2f &centroid) {
  centroid.setZero();
  int cp = 0;
  for (const auto &point : cloud) {
    centroid(0) += point.x;
    centroid(1) += point.y;
    ++cp;
  }
  centroid /= static_cast<float>(cp);
}

template <typename PointT>
void compute2DCovarianceMatrix(const pcl::PointCloud<PointT> &cloud,
                               const Eigen::Vector2f &centroid,
                               Eigen::Matrix2f &covariance_matrix) {
  covariance_matrix.setZero();

  // If the data is dense, we don't need to check for NaN
  // For each point in the cloud
  for (const auto &point : cloud) {
    Eigen::Vector2f pt;
    pt(0) = point.x - centroid(0);
    pt(1) = point.y - centroid(1);

    covariance_matrix(0, 0) += pt.x() * pt.x();
    covariance_matrix(1, 1) += pt.y() * pt.y();
    covariance_matrix(0, 1) += pt.x() * pt.y();
  }
  covariance_matrix(1, 0) = covariance_matrix(0, 1);
}

template <class PointT>
float computeNormalPCA(const pcl::PointCloud<PointT> &point_cloud,
                       const std::vector<int> indices, PointT &query) {
  // Safe check
  if (indices.size() < 3) {
    query.normal_variance = 0.0f;
    query.normal_score = -1.0f;
    return -1.0f;
  }

  // Get points for computation
  const pcl::PointCloud<PointT> points(point_cloud, indices);

  // compute centroid and covariance
  Eigen::Vector2f centroid;
  Eigen::Matrix2f covariance_mat;
  compute2DCentroid(points, centroid);
  compute2DCovarianceMatrix(points, centroid, covariance_mat);

  // Compute pca
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es;
  es.compute(covariance_mat);

  // Orient normal so that it always faces lidar origin
  Eigen::Vector3f normal_vec(es.eigenvectors()(0, 0), es.eigenvectors()(1, 0),
                             0);
  query.getNormalVector3fMap() =
      normal_vec.dot(query.getVector3fMap()) > 0 ? -normal_vec : normal_vec;

  // Variance is the smallest eigenvalue
  query.normal_variance = es.eigenvalues()(0);

  // Score is 1 - sphericity equivalent to planarity + linearity
  query.normal_score =
      1.0f - es.eigenvalues()(0) / (es.eigenvalues()(1) + 1e-9);

  return query.normal_score;
}

template <class PointT>
void scaleAndLogRadius(const pcl::PointCloud<PointT> &points,
                       const float &rho_scale,
                       pcl::PointCloud<pcl::PointXY> &scaled_points) {
  const float rho_factor = 1 / rho_scale;

  scaled_points.clear();
  scaled_points.reserve(points.size());

  for (const auto &point : points) {
    pcl::PointXY scaled_point;
    scaled_point.x = std::log(point.rho) * rho_factor;
    scaled_point.y = point.phi;
    scaled_points.emplace_back(scaled_point);
  }
}

}  // namespace normal

template <class PointT>
std::vector<float> extractNormal(
    const std::shared_ptr<pcl::PointCloud<PointT>> &points,
    const std::shared_ptr<pcl::PointCloud<PointT>> &queries,
    const float &radius, const float &rho_scale, const int parallel_threads) {
  /// distance
  auto scaled_points = std::make_shared<pcl::PointCloud<pcl::PointXY>>();
  auto scaled_queries = std::make_shared<pcl::PointCloud<pcl::PointXY>>();
  normal::scaleAndLogRadius(*points, rho_scale, *scaled_points);
  normal::scaleAndLogRadius(*queries, rho_scale, *scaled_queries);

  /// Create KD Tree to search for neighbors
  NanoFLANNAdapter<pcl::PointXY> adapter(*scaled_points);
  KDTreeParams tree_params(10 /* max leaf */);
  auto kdtree = std::make_unique<KDTree<pcl::PointXY>>(2, adapter, tree_params);
  kdtree->buildIndex();

  // Find neighbors and compute features
  // ***********************************

  const auto sq_radius = radius * radius;
  KDTreeSearchParams search_params;
  search_params.sorted = false;
  int max_neighbs = 10;

  std::vector<float> scores;
  scores.resize(scaled_queries->size());

// Get all features in a parallel loop
#pragma omp parallel for shared(max_neighbs) schedule(dynamic, 10) \
    num_threads(parallel_threads)
  for (size_t i = 0; i < scaled_queries->size(); i++) {
    std::vector<std::pair<size_t, float>> ind_dists;
    ind_dists.reserve(max_neighbs);

    // Find neighbors
    Eigen::Vector2f query_pt(scaled_queries->at(i).x, scaled_queries->at(i).y);
    auto num_neighbors = kdtree->radiusSearch(query_pt.data(), sq_radius,
                                              ind_dists, search_params);

    // extract only the indices
    std::vector<int> inds;
    inds.reserve(num_neighbors);
    for (size_t i = 0; i < num_neighbors; ++i)
      inds.emplace_back(static_cast<int>(ind_dists[i].first));

    // Compute PCA
    scores[i] = normal::computeNormalPCA(*points, inds, (*queries)[i]);
  }

  return scores;
}

}  // namespace radar
}  // namespace vtr