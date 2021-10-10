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
 * \author Hugues Thomas, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/nanoflann/nanoflann.hpp"
#include "vtr_lidar/types.hpp"

#include "lgmath.hpp"
#include "vtr_lidar_msgs/msg/se3_transform.hpp"

namespace vtr {

namespace common {

inline void toROSMsg(const lgmath::se3::TransformationWithCovariance& T,
                     vtr_lidar_msgs::msg::SE3Transform& T_msg) {
  // transform
  T_msg.xi.clear();
  T_msg.xi.reserve(6);
  auto vec = T.vec();
  for (int row = 0; row < 6; ++row) T_msg.xi.push_back(vec(row));

  // covariance
  T_msg.cov.clear();
  T_msg.cov.reserve(36);
  if (!T.covarianceSet()) {
    T_msg.cov_set = false;
  } else {
    auto cov = T.cov();
    for (int row = 0; row < 6; row++)
      for (int col = 0; col < 6; col++) T_msg.cov.push_back(cov(row, col));
    T_msg.cov_set = true;
  }
}

inline void fromROSMsg(const vtr_lidar_msgs::msg::SE3Transform& T_msg,
                       lgmath::se3::TransformationWithCovariance& T) {
  using TransformT = lgmath::se3::TransformationWithCovariance;
  using TransformVecT = Eigen::Matrix<double, 6, 1>;

  if (!T_msg.cov_set)
    T = TransformT(TransformVecT(T_msg.xi.data()));
  else {
    Eigen::Matrix<double, 6, 6> cov;
    for (int row = 0; row < 6; ++row)
      for (int col = 0; col < 6; ++col)
        cov(row, col) = T_msg.cov[row * 6 + col];
    T = TransformT(TransformVecT(T_msg.xi.data()), cov);
  }
}

}  // namespace common

namespace lidar {

template <class PointT>
struct NanoFLANNAdapter {
  NanoFLANNAdapter(const pcl::PointCloud<PointT>& points) : points_(points) {}

  const pcl::PointCloud<PointT>& points_;

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return points_.size(); }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate
  // value, the
  //  "if/else's" are actually solved at compile time.
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
    NanoFLANNAdapter<PointT>, 3>;
template <class PointT>
using DynamicKDTree = nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<float, NanoFLANNAdapter<PointT>>,
    NanoFLANNAdapter<PointT>, 3>;

class Point3D {
 public:
  // Elements
  // ********
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float data[3];
  };

  // Methods
  // *******

  // Constructor
  Point3D(float x0 = 0, float y0 = 0, float z0 = 0) : x(x0), y(y0), z(z0) {}

  // array type accessor
  float operator[](int i) const {
    if (i == 0)
      return x;
    else if (i == 1)
      return y;
    else
      return z;
  }

  // operations
  template <typename PointT>
  float dot(const PointT P) const {
    return x * P.x + y * P.y + z * P.z;
  }

  float sq_norm() const { return x * x + y * y + z * z; }

  template <typename PointT>
  Point3D cross(const PointT P) const {
    return Point3D(y * P.z - z * P.y, z * P.x - x * P.z, x * P.y - y * P.x);
  }

  Point3D& operator+=(const Point3D& P) {
    x += P.x;
    y += P.y;
    z += P.z;
    return *this;
  }

  Point3D& operator-=(const Point3D& P) {
    x -= P.x;
    y -= P.y;
    z -= P.z;
    return *this;
  }

  Point3D& operator*=(const float& a) {
    x *= a;
    y *= a;
    z *= a;
    return *this;
  }
};

// Point Operations
// *****************

inline Point3D operator+(const Point3D A, const Point3D B) {
  return Point3D(A.x + B.x, A.y + B.y, A.z + B.z);
}

inline Point3D operator-(const Point3D A, const Point3D B) {
  return Point3D(A.x - B.x, A.y - B.y, A.z - B.z);
}

inline Point3D operator*(const Point3D P, const float a) {
  return Point3D(P.x * a, P.y * a, P.z * a);
}

inline Point3D operator*(const float a, const Point3D P) {
  return Point3D(P.x * a, P.y * a, P.z * a);
}

inline Point3D operator/(const Point3D P, const float a) {
  return Point3D(P.x / a, P.y / a, P.z / a);
}

inline Point3D operator/(const float a, const Point3D P) {
  return Point3D(P.x / a, P.y / a, P.z / a);
}

inline std::ostream& operator<<(std::ostream& os, const Point3D P) {
  return os << "[" << P.x << ", " << P.y << ", " << P.z << "]";
}

inline bool operator==(const Point3D A, const Point3D B) {
  return A.x == B.x && A.y == B.y && A.z == B.z;
}

inline Point3D floor(const Point3D P) {
  return Point3D(std::floor(P.x), std::floor(P.y), std::floor(P.z));
}

template <class PointT>
Point3D max_point(const pcl::PointCloud<PointT>& point_cloud) {
  const auto& points = point_cloud.points;
  // Initialize limits
  Point3D maxP(points[0].x, points[0].y, points[0].z);
  // Loop over all points
  for (auto p : points) {
    if (p.x > maxP.x) maxP.x = p.x;
    if (p.y > maxP.y) maxP.y = p.y;
    if (p.z > maxP.z) maxP.z = p.z;
  }
  return maxP;
}

template <class PointT>
Point3D min_point(const pcl::PointCloud<PointT>& point_cloud) {
  const auto& points = point_cloud.points;
  // Initialize limits
  Point3D minP(points[0].x, points[0].y, points[0].z);
  // Loop over all points
  for (auto p : points) {
    if (p.x < minP.x) minP.x = p.x;
    if (p.y < minP.y) minP.y = p.y;
    if (p.z < minP.z) minP.z = p.z;
  }
  return minP;
}

}  // namespace lidar
}  // namespace vtr