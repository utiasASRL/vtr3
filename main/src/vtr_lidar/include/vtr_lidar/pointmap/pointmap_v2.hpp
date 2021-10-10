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
 * \file pointmap.hpp
 * \brief <Incremental,SingleExp,MultiExp>PointMap class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>
#include <unordered_set>

#include "lgmath.hpp"
#include "vtr_lidar/types.hpp"
#include "vtr_lidar/utils.hpp"
#include "vtr_logging/logging.hpp"
#include "vtr_tactic/types.hpp"

#if false
/// \todo re-enable this when deleting pointmap.hpp
namespace {
// Simple utility function to combine hashtables
template <typename T, typename... Rest>
inline void hash_combine(std::size_t& seed, const T& v, const Rest&... rest) {
  seed ^= std::hash<T>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  (hash_combine(seed, rest), ...);
}

}  // namespace

namespace vtr {
namespace lidar {

struct VoxKey {
  VoxKey(int x0 = 0, int y0 = 0, int z0 = 0) : x(x0), y(y0), z(z0) {}

  bool operator==(const VoxKey& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }

  int x, y, z;
};

inline VoxKey operator+(const VoxKey A, const VoxKey B) {
  return VoxKey(A.x + B.x, A.y + B.y, A.z + B.z);
}

inline VoxKey operator-(const VoxKey A, const VoxKey B) {
  return VoxKey(A.x - B.x, A.y - B.y, A.z - B.z);
}

}  // namespace lidar
}  // namespace vtr

// Specialization of std:hash function
namespace std {
using namespace vtr::lidar;

template <>
struct hash<VoxKey> {
  std::size_t operator()(const VoxKey& k) const {
    std::size_t ret = 0;
    hash_combine(ret, k.x, k.y, k.z);
    return ret;
  }
};

}  // namespace std
#endif

namespace vtr {
namespace lidar {

template <class PointT>
class PointScan {
 public:
  using PointCloudType = pcl::PointCloud<PointT>;
  using TransformType = lgmath::se3::TransformationWithCovariance;

  /** \brief Size of the map (number of point/voxel in the map) */
  size_t size() const { return point_cloud_.size(); }

  TransformType& T_vertex_map() { return T_vertex_this_; }
  const TransformType& T_vertex_map() const { return T_vertex_this_; }

  tactic::VertexId& vertex_id() { return vertex_id_; }
  const tactic::VertexId& vertex_id() const { return vertex_id_; }

  PointCloudType& point_map() { return point_cloud_; }
  const PointCloudType& point_map() const { return point_cloud_; }

 protected:
  PointCloudType point_cloud_;
  /** \brief the associated vertex id */
  tactic::VertexId vertex_id_;
  /** \brief the transform from this scan/map to its associated vertex */
  TransformType T_vertex_this_;
};

template <class PointT>
class PointMap : public PointScan<PointT> {
 public:
  using typename PointScan<PointT>::PointCloudType;

  PointMap(const float& dl) : dl_(dl) {}

  /** \brief Update map with a set of new points including movabilities. */
  void update(const PointCloudType& point_cloud) {
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

 private:
  void updateCapacity(size_t num_pts) {
    // Reserve new space if needed
    if (samples_.empty()) samples_.reserve(10 * num_pts);
    this->point_cloud_.reserve(this->point_cloud_.size() + num_pts);
  }

  /** \brief Initialize a voxel centroid */
  void initSample(const VoxKey& k, const PointT& p) {
    // We place a new key in the hashmap
    samples_.emplace(k, this->point_cloud_.size());
    // We add new voxel data but initiate only the centroid
    this->point_cloud_.push_back(p);
  }

  /** \brief Update of voxel centroid */
  void updateSample(const size_t idx, const PointT& p) {
    if (p.normal_score <= this->point_cloud_[idx].normal_score) return;

    // Update normal if we have a clear view of it and closer distance (see
    // computation of score)
    auto& p_ = this->point_cloud_[idx];
    // copy point normal information
    std::copy(std::begin(p.data_n), std::end(p.data_n), std::begin(p_.data_n));
    // copy normal score
    p_.normal_score = p.normal_score;
  }

 private:
  VoxKey getKey(const PointT& p) const {
    VoxKey k((int)std::floor(p.x / dl_), (int)std::floor(p.y / dl_),
             (int)std::floor(p.z / dl_));
    return k;
  }

 private:
  /** \brief Voxel grid size */
  const float dl_;

  /** \brief Sparse hashmap that contain voxels and map to point indices */
  std::unordered_map<VoxKey, size_t> samples_;
};

}  // namespace lidar
}  // namespace vtr