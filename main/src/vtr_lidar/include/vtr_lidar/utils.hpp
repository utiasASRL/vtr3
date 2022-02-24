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
 * \file utils.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "lgmath.hpp"

#include "vtr_common/utils/hash.hpp"
#include "vtr_lidar/nanoflann/nanoflann.hpp"
#include "vtr_lidar/types.hpp"

namespace vtr {
namespace lidar {

template <class PointT>
struct NanoFLANNAdapter {
  NanoFLANNAdapter(const pcl::PointCloud<PointT>& points) : points_(points) {}

  const pcl::PointCloud<PointT>& points_;

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return points_.size(); }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate
  // value, the "if/else's" are actually solved at compile time.
  inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0)
      return points_[idx].x;
    else if (dim == 1)
      return points_[idx].y;
    else
      return points_[idx].z;
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in
  //   "bb" so it can be avoided to redo it again. Look at bb.size() to find out
  //   the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /* bb */) const {
    return false;
  }
};

// KDTree type definition
using KDTreeParams = nanoflann::KDTreeSingleIndexAdaptorParams;
using KDTreeSearchParams = nanoflann::SearchParams;
using KDTreeResultSet = nanoflann::KNNResultSet<float>;
template <class PointT>
using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, NanoFLANNAdapter<PointT>>,
    NanoFLANNAdapter<PointT>>;
template <class PointT>
using DynamicKDTree = nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<float, NanoFLANNAdapter<PointT>>,
    NanoFLANNAdapter<PointT>>;

}  // namespace lidar
}  // namespace vtr

namespace vtr {
namespace lidar {

template <class PointT>
void cart2pol(pcl::PointCloud<PointT>& point_cloud) {
  for (auto& p : point_cloud) {
    p.rho = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    p.theta = atan2(sqrt(p.x * p.x + p.y * p.y), p.z);
    p.phi = atan2(p.y, p.x) + M_PI / 2;
  }
}

}  // namespace lidar
}  // namespace vtr