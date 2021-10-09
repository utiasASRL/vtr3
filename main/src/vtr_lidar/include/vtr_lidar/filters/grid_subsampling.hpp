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
 * \brief Grid subsampling utilities
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "pcl/point_cloud.h"

#include "vtr_lidar/utils.hpp"

namespace vtr {
namespace lidar {

template <class PointT>
class VoxelCenter {
 public:
  // Elements
  // ********

  size_t idx = 0;
  Point3D center = Point3D();
  float d2 = 0;

  // Methods
  // *******

  // Constructor
  VoxelCenter() = default;

  VoxelCenter(size_t idx0, const PointT& p0, const Point3D& center0) {
    idx = idx0;
    center = center0;
    d2 =
        Point3D(p0.x - center0.x, p0.y - center0.y, p0.z - center0.z).sq_norm();
  }

  void update_points(size_t idx0, const PointT& p0) {
    float new_d2 =
        Point3D(p0.x - center.x, p0.y - center.y, p0.z - center.z).sq_norm();
    if (new_d2 < d2) {
      d2 = new_d2;
      idx = idx0;
    }
    return;
  }
};

template <class PointT>
void gridSubsamplingCentersV2(pcl::PointCloud<PointT>& point_cloud,
                              const float& sampleDl) {
  // Initialize variables
  // ********************

  // Inverse of sample dl
  float inv_dl = 1 / sampleDl;

  // Limits of the map
  Point3D minCorner = min_point(point_cloud);
  Point3D maxCorner = max_point(point_cloud);
  Point3D originCorner = floor(minCorner * inv_dl) * sampleDl;

  // Dimensions of the grid
  size_t sampleNX =
      (size_t)std::floor((maxCorner.x - originCorner.x) * inv_dl) + 1;
  size_t sampleNY =
      (size_t)std::floor((maxCorner.y - originCorner.y) * inv_dl) + 1;

  // Create the sampled map
  // **********************

  // Initialize variables
  size_t i, iX, iY, iZ, mapIdx;
  std::unordered_map<size_t, VoxelCenter<PointT>> samples;
  samples.reserve(point_cloud.size());

  i = 0;
  for (auto& p : point_cloud) {
    // Position of point in sample map
    iX = (size_t)std::floor((p.x - originCorner.x) * inv_dl);
    iY = (size_t)std::floor((p.y - originCorner.y) * inv_dl);
    iZ = (size_t)std::floor((p.z - originCorner.z) * inv_dl);
    mapIdx = iX + sampleNX * iY + sampleNX * sampleNY * iZ;

    // Fill the sample map
    if (samples.count(mapIdx) < 1) {
      samples.emplace(
          mapIdx,
          VoxelCenter<PointT>(i, p,
                              Point3D(originCorner.x + (iX + 0.5) * sampleDl,
                                      originCorner.y + (iY + 0.5) * sampleDl,
                                      originCorner.z + (iZ + 0.5) * sampleDl)));
    } else
      samples[mapIdx].update_points(i, p);

    // Increment point index
    i++;
  }

  // Convert hmap to index vector
  std::vector<int> indices;
  indices.reserve(samples.size());
  for (auto& v : samples) indices.push_back(v.second.idx);

  // Modify the point_cloud
  point_cloud = pcl::PointCloud<PointT>(point_cloud, indices);
}

}  // namespace lidar
}  // namespace vtr