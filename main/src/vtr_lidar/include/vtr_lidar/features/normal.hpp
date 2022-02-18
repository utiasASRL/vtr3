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
 * \brief normal vector extraction functions
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */

#include <cmath>

#include "pcl/features/normal_3d.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"

#include "vtr_logging/logging.hpp"

namespace vtr {
namespace lidar {

template <class PointT>
float computeNormalPCA(const pcl::PointCloud<PointT> &point_cloud,
                       const std::vector<int> indices, PointT &query) {
  // Safe check
  if (indices.size() < 4) return -1.0f;

  // Get points for computation
  const pcl::PointCloud<PointT> points(point_cloud, indices);

  // Placeholder for the 3x3 covariance matrix at each surface patch
  Eigen::Matrix3f covariance_matrix;
  // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
  Eigen::Vector4f xyz_centroid;

  // Estimate the XYZ centroid
  pcl::compute3DCentroid(points, xyz_centroid);

  // Compute the 3x3 covariance matrix
  pcl::computeCovarianceMatrix(points, xyz_centroid, covariance_matrix);

  // Compute pca
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
  es.compute(covariance_matrix);

  // Orient normal so that it always faces lidar origin
  query.getNormalVector3fMap() =
      es.eigenvectors().col(0).dot(query.getVector3fMap()) > 0
          ? -Eigen::Vector3f(es.eigenvectors().col(0))
          : Eigen::Vector3f(es.eigenvectors().col(0));

  // Variance is the smallest eigenvalue
  query.normal_variance = es.eigenvalues()(0);

  // Score is 1 - sphericity equivalent to planarity + linearity
  query.normal_score = 1.f - es.eigenvalues()(0) / (es.eigenvalues()(2) + 1e-9);

  return query.normal_score;
}

template <class PointT>
void scaleAndLogRadius(const pcl::PointCloud<PointT> &points,
                       const float &r_scale, const float &h_scale,
                       pcl::PointCloud<pcl::PointXYZ> &scaled_points) {
  const float r_factor = 1 / r_scale;
  const float h_factor = 1 / h_scale;

  scaled_points.clear();
  scaled_points.reserve(points.size());

  for (const auto &point : points) {
    scaled_points.emplace_back(log(point.rho) * r_factor, point.theta,
                               point.phi * h_factor);
  }
}

template <class PointT>
std::vector<float> extractNormal(
    const std::shared_ptr<const pcl::PointCloud<PointT>> &points,
    const std::shared_ptr<pcl::PointCloud<PointT>> &queries,
    const float &radius, const float &r_scale, const float &h_scale,
    const int parallel_threads) {
  /// distance
  auto scaled_points = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto scaled_queries = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  scaleAndLogRadius(*points, r_scale, h_scale, *scaled_points);
  scaleAndLogRadius(*queries, r_scale, h_scale, *scaled_queries);

  /// Create KD Tree to search for neighbors
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree(false);
  kdtree.setInputCloud(scaled_points);

  // Find neighbors and compute features
  // ***********************************

  // Variable for reserving memory
  int max_neighbs = 10;

  std::vector<float> scores;
  scores.resize(scaled_queries->size());

// Get all features in a parallel loop
#pragma omp parallel for shared(max_neighbs) schedule(dynamic, 10) \
    num_threads(parallel_threads)
  for (size_t i = 0; i < scaled_queries->size(); i++) {
    std::vector<int> inds;
    std::vector<float> sqdists;
    inds.reserve(max_neighbs);
    sqdists.reserve(max_neighbs);

    if (!pcl::isFinite(scaled_queries->at(i))) {
      scores[i] = -1.0f;
      continue;
    }

    // Find neighbors
    auto n_neighbs =
        kdtree.radiusSearch(scaled_queries->at(i), radius, inds, sqdists);

    // Update max count
    if (n_neighbs > max_neighbs) {
#pragma omp atomic write
      max_neighbs = n_neighbs;
    }

    // Compute PCA
    scores[i] = computeNormalPCA(*points, inds, (*queries)[i]);
  }

  return scores;
}

template <class PointT>
std::vector<float> smartNormalScore(pcl::PointCloud<PointT> &point_cloud,
                                    const float &r0, const float &theta0) {
  // Parameters
  float S0 = 0.2;
  float S1 = 1.0 - S0;
  float a0 = M_PI / 2;       // Max possible angle for which score is zero
  float a1 = M_PI * theta0;  // if angle > a1, whatever radius, score is better
                             // if angle is smaller (up to S0)
  float factor = S0 / (a0 - a1);
  float inv_sigma2 = 0.01f;

  std::vector<float> scores;
  scores.reserve(point_cloud.size());

  std::stringstream ss;

  // loop over all
  for (auto &point : point_cloud) {
    float s2;
    const float r = point.rho;
    float angle = acos(std::min(
        abs(point.getVector3fMap().dot(point.getNormalVector3fMap()) / r),
        1.0f));

    if (angle > a1)
      s2 = factor * (a0 - angle);
    else
      // change to penalize points that are too close only
      s2 = S0 + S1 * exp(-(pow(std::min(r0, r) - r0, 2)) * inv_sigma2);

    point.normal_score = std::min(point.normal_score * s2, 1.0f);
    scores.emplace_back(point.normal_score);
  }

  return scores;
}

}  // namespace lidar
}  // namespace vtr