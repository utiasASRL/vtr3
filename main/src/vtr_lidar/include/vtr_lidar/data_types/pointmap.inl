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
 * \file pointmap.inl
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/data_types/pointmap.hpp"

#include "pcl/features/normal_3d.h"
#include "pcl_conversions/pcl_conversions.h"

#include "vtr_common/conversions/ros_lgmath.hpp"

#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

template <class PointT>
auto PointMap<PointT>::fromStorable(const PointMapMsg& storable) -> Ptr {
  // construct with dl and version
  auto data = std::make_shared<PointMap<PointT>>(storable.dl, storable.version);
  // load point cloud data
  pcl::fromROSMsg(storable.point_cloud, data->point_cloud_);
  // load vertex id
  data->vertex_id_ = tactic::VertexId(storable.vertex_id);
  // load transform
  using namespace vtr::common;
  conversions::fromROSMsg(storable.t_vertex_this, data->T_vertex_this_);
  // build voxel map
  data->samples_.clear();
  data->samples_.reserve(data->point_cloud_.size());
  size_t i = 0;
  for (const auto& p : data->point_cloud_) {
    auto result = data->samples_.emplace(data->getKey(p), i);
    if (!result.second)
      throw std::runtime_error{
          "PointMap fromStorable detects points with same key. This should "
          "never happen."};
    i++;
  }
  return data;
}

template <class PointT>
auto PointMap<PointT>::toStorable() const -> PointMapMsg {
  PointMapMsg storable;
  // save point cloud data
  pcl::toROSMsg(this->point_cloud_, storable.point_cloud);
  // save vertex id
  storable.vertex_id = this->vertex_id_;
  // save transform
  using namespace vtr::common;
  conversions::toROSMsg(this->T_vertex_this_, storable.t_vertex_this);
  // save version
  storable.version = this->version_;
  // save voxel size
  storable.dl = this->dl_;
  return storable;
}

template <class PointT>
void PointMap<PointT>::update(const PointCloudType& point_cloud) {
  // Reserve new space if needed
  updateCapacity(point_cloud.size());
  // Update the current map
  for (auto& p : point_cloud) {
    // Get the corresponding key
    auto k = getKey(p);
    // Update the point count
    if (samples_.count(k) < 1)
      initSample(k, p);
    else
      updateSample(samples_[k], p);
  }
}

template <class PointT>
void PointMap<PointT>::updateNormal(const PointCloudType& point_cloud) {
  //
  NanoFLANNAdapter<PointT> adapter(this->point_cloud_);
  KDTreeSearchParams search_params;
  KDTreeParams tree_params(10 /* max leaf */);
  auto kdtree = std::make_unique<KDTree<PointT>>(3, adapter, tree_params);
  kdtree->buildIndex();
  const auto search_radius = dl_ * 3.0;
  const auto sq_radius = search_radius * search_radius;
  for (auto& p : point_cloud) {
    std::vector<float> dists;
    std::vector<int> indices;
    NanoFLANNRadiusResultSet<float, int> result(sq_radius, dists, indices);
    kdtree->radiusSearchCustomCallback(p.data, result, search_params);

    if (indices.size() < 4) continue;

    // get points for computation
    const pcl::PointCloud<PointT> points(this->point_cloud_, indices);

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

    // normal direction
    auto& q = this->point_cloud_[samples_.at(getKey(p))];
    q.getNormalVector3fMap() = Eigen::Vector3f(es.eigenvectors().col(0));

    // normal score
    q.normal_score =
        (es.eigenvalues()(1) - es.eigenvalues()(0)) / es.eigenvalues()(2);
  }
}

template <class PointT>
void PointMap<PointT>::crop(const Eigen::Matrix4f& T_center_this, float range,
                            float ratio) {
  std::vector<int> indices;
  indices.reserve(this->point_cloud_.size());
  for (size_t i = 0; i < this->point_cloud_.size(); i++) {
    const auto& point = this->point_cloud_.at(i);
    //
    Eigen::Vector4f homo_point;
    homo_point << point.x, point.y, point.z, 1.0;
    Eigen::Vector4f transformed_point(T_center_this * homo_point);
    // crop box
    // if (transformed_point(0) < -range || transformed_point(0) > range ||
    //     transformed_point(1) < -range || transformed_point(1) > range ||
    //     transformed_point(2) < -range || transformed_point(2) > range)
    //   continue;
    // crop range
    const float rho = transformed_point.block(0, 0, 3, 1).norm();
    const float phi = std::atan2(transformed_point(1), transformed_point(0));
    /// \note assuming x is front, y is left, z is up
    const float ratio_given_phi =
        ratio + (1 - std::abs(phi) / M_PI) * (1 - ratio);
    if (rho > range * ratio_given_phi) continue;
    //
    indices.emplace_back(i);
  }
  filter(indices);
}

template <class PointT>
void PointMap<PointT>::subtractLifeTime(const float& life_time) {
  std::vector<int> indices;
  indices.reserve(this->point_cloud_.size());
  for (size_t i = 0; i < this->point_cloud_.size(); i++) {
    auto& point = this->point_cloud_.at(i);
    point.life_time -= life_time;
    if (point.life_time > 0.0) indices.emplace_back(i);
  }
  filter(indices);
}

template <class PointT>
void PointMap<PointT>::filter(const std::vector<int>& indices) {
  // create a copy of the point cloud and apply filter
  const auto point_cloud = this->point_cloud_;
  pcl::copyPointCloud(point_cloud, indices, this->point_cloud_);
  // rebuild the voxel map
  samples_.clear();
  samples_.reserve(this->point_cloud_.size());
  size_t i = 0;
  for (const auto& p : this->point_cloud_) {
    auto result = samples_.emplace(getKey(p), i);
    if (!result.second)
      throw std::runtime_error{
          "PointMap fromStorable detects points with same key. This should "
          "never happen."};
    i++;
  }
}

template <class PointT>
void PointMap<PointT>::updateCapacity(size_t num_pts) {
  // Reserve new space if needed
  if (samples_.empty()) samples_.reserve(10 * num_pts);
  this->point_cloud_.reserve(this->point_cloud_.size() + num_pts);
}

template <class PointT>
void PointMap<PointT>::initSample(const VoxKey& k, const PointT& p) {
  // We place a new key in the hashmap
  samples_.emplace(k, this->point_cloud_.size());
  // We add new voxel data
  this->point_cloud_.push_back(p);
}

template <class PointT>
void PointMap<PointT>::updateSample(const size_t idx, const PointT& p) {
  auto& p_ = this->point_cloud_[idx];

  // renew life time
  p_.life_time = std::max(p_.life_time, p.life_time);

  // Update normal if we have a clear view of it and closer distance (see
  // computation of score)
  if (p.normal_score <= this->point_cloud_[idx].normal_score) return;
  // copy point normal information
  std::copy(std::begin(p.data_n), std::end(p.data_n), std::begin(p_.data_n));
  // copy normal score
  p_.normal_variance = p.normal_variance;
  p_.normal_score = p.normal_score;
}

}  // namespace lidar
}  // namespace vtr