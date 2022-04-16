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
 * \file ray_tracing.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/data_types/point.hpp"

namespace vtr {
namespace lidar {
namespace ray_tracing {

struct PixKey {
  PixKey(int x0 = 0, int y0 = 0) : x(x0), y(y0) {}

  bool operator==(const PixKey& other) const {
    return (x == other.x && y == other.y);
  }

  int x, y;
};

inline PixKey operator+(const PixKey A, const PixKey B) {
  return PixKey(A.x + B.x, A.y + B.y);
}

inline PixKey operator-(const PixKey A, const PixKey B) {
  return PixKey(A.x - B.x, A.y - B.y);
}

template <class PointT>
void cart2pol(pcl::PointCloud<PointT>& point_cloud) {
  for (auto& p : point_cloud) {
    p.rho = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    p.theta = std::atan2(std::sqrt(p.x * p.x + p.y * p.y), p.z);
    p.phi = std::atan2(p.y, p.x);
  }
}

}  // namespace ray_tracing
}  // namespace lidar
}  // namespace vtr

// Specialization of std:hash function
namespace std {

template <>
struct hash<vtr::lidar::ray_tracing::PixKey> {
  std::size_t operator()(const vtr::lidar::ray_tracing::PixKey& k) const {
    std::size_t ret = 0;
    vtr::common::hash_combine(ret, k.x, k.y);
    return ret;
  }
};

}  // namespace std

namespace vtr {
namespace lidar {

template <class PointT>
inline ray_tracing::PixKey getKey(const PointT& p, const float& phi_res,
                                  const float& theta_res) {
  // Position of point in sample map
  return ray_tracing::PixKey((int)std::floor(p.theta / theta_res),
                             (int)std::floor(p.phi / phi_res));
}

template <class PointT>
void detectDynamicObjects(
    const pcl::PointCloud<PointT>& /* point scan */ reference,
    pcl::PointCloud<PointT>& /* point map */ query,
    const lgmath::se3::TransformationWithCovariance& T_ref_qry,
    const float& phi_res, const float& theta_res, const float& max_num_obs,
    const float& min_num_obs, const float& dynamic_threshold) {
  (void)max_num_obs;
  // Parameters
  const auto inner_ratio = 1 - std::max(phi_res, theta_res) / 2;
  (void)inner_ratio;
  const auto outer_ratio = 1 + std::max(phi_res, theta_res) / 2;
  // this takes into account surface orientation
  const auto tighter_inner_ratio =
      1 - (std::max(phi_res, theta_res) / 2) / tan(M_PI / 12);

  // Create and fill in the frustum grid
  std::unordered_map<ray_tracing::PixKey, float> frustum_grid;
  for (const auto& p : reference) {
    const auto k = getKey(p, phi_res, theta_res);
    if (frustum_grid.count(k) == 0)
      frustum_grid[k] = p.rho;
    else
      /// // always choose the further point
      // frustum_grid.at(k) = std::max(p.x, frustum_grid.at(k));
      /// always choose the closer point
      frustum_grid.at(k) = std::min(p.rho, frustum_grid.at(k));
  }

  // Perform a transformation in to the frame of the reference point cloud
  auto query_tmp = query;  // copy
  const auto& T_ref_qry_mat = T_ref_qry.matrix();
  // eigen mapping
  auto points_mat =
      query_tmp.getMatrixXfMap(3, PointT::size(), PointT::cartesian_offset());
  auto normal_mat =
      query_tmp.getMatrixXfMap(3, PointT::size(), PointT::normal_offset());
  // transform to the local frame of this vertex
  Eigen::Matrix3f R_tot = (T_ref_qry_mat.block<3, 3>(0, 0)).cast<float>();
  Eigen::Vector3f T_tot = (T_ref_qry_mat.block<3, 1>(0, 3)).cast<float>();
  points_mat = (R_tot * points_mat).colwise() + T_tot;
  normal_mat = R_tot * normal_mat;
  ray_tracing::cart2pol(query_tmp);

  //
  for (size_t i = 0; i < query.size(); i++) {
    auto& qp = query[i];           // point with dynamic obs to be updated
    const auto& p = query_tmp[i];  // expressed in the reference scan frame

    // compute polar coordinates and key
    const auto k = getKey(p, phi_res, theta_res);

    if (!frustum_grid.count(k)) continue;

    // the current point is occluded in the current observation
    if (p.rho > (frustum_grid.at(k) * outer_ratio)) continue;

    // update this point only when we have a good normal
    float angle = acos(std::min(
        abs(p.getVector3fMap().dot(p.getNormalVector3fMap()) / p.rho), 1.0f));
    if (angle > 5 * M_PI / 12) continue;

#if true
    qp.total_obs++;
    if (p.rho < (frustum_grid.at(k) * tighter_inner_ratio)) qp.dynamic_obs++;
#else
    if (qp.total_obs < max_num_obs) {
      qp.dynamic_obs += (p.rho < (frustum_grid.at(k) * inner_ratio));
      qp.total_obs++;
    } else {
      if (p.rho < (frustum_grid.at(k) * inner_ratio))
        qp.dynamic_obs = std::min(max_num_obs, qp.dynamic_obs + 1);
      else
        qp.dynamic_obs = std::max(0.f, qp.dynamic_obs - 1);
    }
#endif
  }

  for (size_t i = 0; i < query.size(); i++) {
    auto& qp = query[i];  // point with dynamic obs to be updated
    // update the scores
    if (qp.total_obs < min_num_obs)
      qp.static_score = 0;
    else
      qp.static_score =
          (qp.dynamic_obs / qp.total_obs) < dynamic_threshold ? 1 : 0;
  }
}

}  // namespace lidar
}  // namespace vtr