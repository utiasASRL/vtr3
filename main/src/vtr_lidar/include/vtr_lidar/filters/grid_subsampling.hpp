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
 * \file grid_subsampling.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "pcl/point_cloud.h"

#include "vtr_lidar/utils.hpp"

namespace vtr {
namespace lidar {

namespace grid_subsampling {

struct Point3D {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float data[3];
  };
  // clang-format off
  Point3D(const float& x0 = 0, const float& y0 = 0, const float& z0 = 0) : x(x0), y(y0), z(z0) {}

  template <class PointT>
  float dot(const PointT& P) const { return x * P.x + y * P.y + z * P.z; }
  template <class PointT>
  Point3D cross(const PointT& P) const { return Point3D(y * P.z - z * P.y, z * P.x - x * P.z, x * P.y - y * P.x); }

  float sq_norm() const { return x * x + y * y + z * z; }
  Point3D floor() const { return Point3D(std::floor(x), std::floor(y), std::floor(z)); }

  template <class PointT>
  friend Point3D operator+(const PointT& A, const Point3D& B) { return Point3D(A.x + B.x, A.y + B.y, A.z + B.z);}
  template <class PointT>
  friend Point3D operator+(const Point3D& A, const PointT& B) { return Point3D(A.x + B.x, A.y + B.y, A.z + B.z);}
  template <class PointT>
  friend Point3D operator-(const PointT& A, const Point3D& B) { return Point3D(A.x - B.x, A.y - B.y, A.z - B.z);}
  template <class PointT>
  friend Point3D operator-(const Point3D& A, const PointT& B) { return Point3D(A.x - B.x, A.y - B.y, A.z - B.z);}
  template <class ScalarT>
  friend Point3D operator*(const ScalarT& a, const Point3D& P) { return Point3D(P.x * a, P.y * a, P.z * a); }
  template <class ScalarT>
  friend Point3D operator*(const Point3D& P, const ScalarT& a) { return Point3D(P.x * a, P.y * a, P.z * a); }
  // clang-format on
};

template <class PointT>
Point3D getMaxPoint(const pcl::PointCloud<PointT>& points) {
  // Initialize limits
  Point3D max_pt(points[0].x, points[0].y, points[0].z);
  // Loop over all points
  for (const auto& p : points) {
    if (p.x > max_pt.x) max_pt.x = p.x;
    if (p.y > max_pt.y) max_pt.y = p.y;
    if (p.z > max_pt.z) max_pt.z = p.z;
  }
  return max_pt;
}

template <class PointT>
Point3D getMinPoint(const pcl::PointCloud<PointT>& points) {
  // Initialize limits
  Point3D min_pt(points[0].x, points[0].y, points[0].z);
  // Loop over all points
  for (const auto& p : points) {
    if (p.x < min_pt.x) min_pt.x = p.x;
    if (p.y < min_pt.y) min_pt.y = p.y;
    if (p.z < min_pt.z) min_pt.z = p.z;
  }
  return min_pt;
}

template <class PointT>
struct VoxelCenter {
  // Elements
  size_t idx = 0;
  Point3D center = Point3D();
  float d2 = 0;

  // Methods
  VoxelCenter(size_t idx0, const PointT& p0, const Point3D& center0)
      : idx(idx0), center(center0), d2((p0 - center0).sq_norm()) {}

  void update(size_t idx0, const PointT& p0) {
    const auto new_d2 = (p0 - center).sq_norm();
    if (new_d2 < d2) {
      d2 = new_d2;
      idx = idx0;
    }
  }
};

}  // namespace grid_subsampling

template <class PointT>
void gridSubsamplingCentersV2(pcl::PointCloud<PointT>& point_cloud,
                              const float& sample_dl) {
  using namespace grid_subsampling;
  // Initialize variables
  // ********************

  // Inverse of sample dl
  float inv_dl = 1 / sample_dl;

  // Limits of the map
  const auto minCorner = getMinPoint(point_cloud);
  const auto maxCorner = getMaxPoint(point_cloud);
  const auto originCorner = (minCorner * inv_dl).floor() * sample_dl;

  // Dimensions of the grid
  const auto sampleNX =
      (size_t)std::floor((maxCorner.x - originCorner.x) * inv_dl) + 1;
  const auto sampleNY =
      (size_t)std::floor((maxCorner.y - originCorner.y) * inv_dl) + 1;

  // Create the sampled map
  // **********************

  // Initialize variables
  std::unordered_map<size_t, VoxelCenter<PointT>> samples;
  samples.reserve(point_cloud.size());

  size_t i = 0;
  for (const auto& p : point_cloud) {
    // Position of point in sample map
    const auto iX = (size_t)std::floor((p.x - originCorner.x) * inv_dl);
    const auto iY = (size_t)std::floor((p.y - originCorner.y) * inv_dl);
    const auto iZ = (size_t)std::floor((p.z - originCorner.z) * inv_dl);
    const auto mapIdx = iX + sampleNX * iY + sampleNX * sampleNY * iZ;

    // Fill the sample map
    if (samples.count(mapIdx) < 1) {
      samples.emplace(mapIdx,
                      VoxelCenter<PointT>(
                          i, p,
                          Point3D(originCorner.x + (iX + 0.5) * sample_dl,
                                  originCorner.y + (iY + 0.5) * sample_dl,
                                  originCorner.z + (iZ + 0.5) * sample_dl)));
    } else {
      samples.at(mapIdx).update(i, p);
    }

    // Increment point index
    i++;
  }

  // Convert hmap to index vector
  std::vector<int> indices;
  indices.reserve(samples.size());
  for (const auto& v : samples) indices.push_back(v.second.idx);

  // Modify the point_cloud
  point_cloud = pcl::PointCloud<PointT>(point_cloud, indices);
}

}  // namespace lidar
}  // namespace vtr