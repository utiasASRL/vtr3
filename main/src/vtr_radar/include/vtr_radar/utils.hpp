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
#include "vtr_radar/nanoflann/nanoflann.hpp"
#include "vtr_radar/types.hpp"

namespace vtr {
namespace radar {

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

template <>
struct NanoFLANNAdapter<pcl::PointXY> {
  NanoFLANNAdapter(const pcl::PointCloud<pcl::PointXY>& points)
      : points_(points) {}

  const pcl::PointCloud<pcl::PointXY>& points_;

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
      throw std::invalid_argument("Invalid dim");
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

}  // namespace radar
}  // namespace vtr

namespace vtr {
namespace radar {

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

}  // namespace radar
}  // namespace vtr

// Specialization of std:hash function
namespace std {
using namespace vtr::radar;
using namespace vtr::common;

template <>
struct hash<PixKey> {
  std::size_t operator()(const PixKey& k) const {
    std::size_t ret = 0;
    hash_combine(ret, k.x, k.y);
    return ret;
  }
};

}  // namespace std

namespace vtr {
namespace radar {

template <class PointT>
void cart2pol(pcl::PointCloud<PointT>& point_cloud) {
  for (auto& p : point_cloud) {
    p.rho = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    p.theta = atan2(sqrt(p.x * p.x + p.y * p.y), p.z);
    p.phi = atan2(p.y, p.x) + M_PI / 2;
  }
}

// vbar is 3x1 v_s_i_in_s where (s) is sensor, (i) is a static frame
// like an inertial frame or (pm) frame in this repository
// beta \approx f_transmission / (slope of modulation pattern dfdt)
template <class PointT>
void removeDoppler(pcl::PointCloud<PointT>& point_cloud,
  const Eigen::VectorXd &vbar, const double beta) {
  for (auto& p : point_cloud) {
    Eigen::Vector3d abar = {p.x, p.y, p.z};
    abar.normalize();
    // relative velocity (> 0: point/sensor moving towards each other)
    const double v = abar.transpose() * vbar;
    const double delta_rho = beta * v;
    // correct radial distance using Doppler factor
    const double rho = sqrt(p.x * p.x + p.y * p.y + p.z * p.z) + delta_rho;
    const double theta = atan2(sqrt(p.x * p.x + p.y * p.y), p.z);
    const double phi = atan2(p.y, p.x);
    // Correct point locations
    p.x = rho * std::cos(phi) * std::sin(theta);
    p.y = rho * std::sin(phi) * std::sin(theta);
    p.z = rho * std::cos(theta);
  }
}

}  // namespace radar
}  // namespace vtr