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
 * \file nanoflann_utils.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/data_types/point.hpp"
#include "vtr_lidar/utils/nanoflann.hpp"

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

//Store all neighbours within a given radius
template <typename _DistanceType = float, typename _IndexType = size_t>
class NanoFLANNRadiusResultSet {
 public:
  using DistanceType = _DistanceType;
  using IndexType = _IndexType;

 public:
  const DistanceType radius;

  std::vector<DistanceType>& m_dists;
  std::vector<IndexType>& m_indices;

  inline NanoFLANNRadiusResultSet(DistanceType radius_,
                                  std::vector<DistanceType>& dists,
                                  std::vector<IndexType>& indices)
      : radius(radius_), m_dists(dists), m_indices(indices) {
    init();
  }

  inline void init() { clear(); }
  inline void clear() {
    m_dists.clear();
    m_indices.clear();
  }

  inline size_t size() const { return m_indices.size(); }

  inline bool full() const { return true; }

  /**
   * Called during search to add an element matching the criteria.
   * \return true if the search should be continued, false if the results are
   * sufficient
   */
  inline bool addPoint(DistanceType dist, IndexType index) {
    if (dist < radius) {
      m_dists.push_back(dist);
      m_indices.push_back(index);
    }
    return true;
  }

  inline DistanceType worstDist() const { return radius; }
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
