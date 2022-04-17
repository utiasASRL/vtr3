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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_common/utils/hash.hpp"
#include "vtr_lidar/data_types/pointscan.hpp"

#include "vtr_lidar_msgs/msg/point_map.hpp"

namespace vtr {
namespace lidar {

namespace pointmap {

struct VoxKey {
  VoxKey(int x0 = 0, int y0 = 0, int z0 = 0) : x(x0), y(y0), z(z0) {}
  // clang-format off
  bool operator==(const VoxKey& other) const { return (x == other.x && y == other.y && z == other.z); }
  friend VoxKey operator+(const VoxKey &A, const VoxKey &B) { return VoxKey(A.x + B.x, A.y + B.y, A.z + B.z); }
  friend VoxKey operator-(const VoxKey &A, const VoxKey &B) { return VoxKey(A.x - B.x, A.y - B.y, A.z - B.z); }
  // clang-format on
  friend class std::hash<VoxKey>;
  int x, y, z;
};

}  // namespace pointmap
}  // namespace lidar
}  // namespace vtr

// Specialization of std:hash function
namespace std {

template <>
struct hash<vtr::lidar::pointmap::VoxKey> {
  std::size_t operator()(const vtr::lidar::pointmap::VoxKey& k) const {
    std::size_t ret = 0;
    vtr::common::hash_combine(ret, k.x, k.y, k.z);
    return ret;
  }
};

}  // namespace std

namespace vtr {
namespace lidar {

template <class PointT>
class PointMap : public PointScan<PointT> {
 public:
  using typename PointScan<PointT>::PointCloudType;
  PTR_TYPEDEFS(PointMap<PointT>);

  using PointMapMsg = vtr_lidar_msgs::msg::PointMap;
  /// constexpr of map version enum (keep in sync with the msg)
  static constexpr unsigned INITIAL = PointMapMsg::INITIAL;
  static constexpr unsigned INTRA_EXP_MERGED = PointMapMsg::INTRA_EXP_MERGED;
  static constexpr unsigned DYNAMIC_REMOVED = PointMapMsg::DYNAMIC_REMOVED;
  static constexpr unsigned INTER_EXP_MERGED = PointMapMsg::INTER_EXP_MERGED;
  /** \brief Static function that constructs this class from ROS2 message */
  static Ptr fromStorable(const PointMapMsg& storable);
  /** \brief Returns the ROS2 message to be stored */
  PointMapMsg toStorable() const;

  PointMap(const float& dl, const unsigned& version = INITIAL)
      : dl_(dl), version_(version) {}

  float dl() const { return dl_; }

  unsigned& version() { return version_; }
  const unsigned& version() const { return version_; }

  /** \brief Update map with a set of new points. */
  void update(const PointCloudType& point_cloud);

  void updateNormal(const PointCloudType& point_cloud);

  void crop(const Eigen::Matrix4f& T_center_this, float range, float ratio);

  void subtractLifeTime(const float& life_time = 1.0);

 protected:
  using VoxKey = pointmap::VoxKey;
  VoxKey getKey(const PointT& p) const {
    return VoxKey((int)std::floor(p.x / dl_), (int)std::floor(p.y / dl_),
                  (int)std::floor(p.z / dl_));
  }

  void filter(const std::vector<int>& indices);

 private:
  void updateCapacity(size_t num_pts);
  /** \brief Initialize a voxel centroid */
  void initSample(const VoxKey& k, const PointT& p);
  /** \brief Update of voxel centroid */
  void updateSample(const size_t idx, const PointT& p);

 protected:
  /** \brief Voxel grid size */
  float dl_;
  /** \brief Version of the map */
  unsigned version_;
  /** \brief Sparse hashmap that contain voxels and map to point indices */
  std::unordered_map<VoxKey, size_t> samples_;
};

}  // namespace lidar
}  // namespace vtr

#include "vtr_lidar/data_types/pointmap.inl"