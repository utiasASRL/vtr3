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

#include "pcl_conversions/pcl_conversions.h"

#include "lgmath.hpp"
#include "vtr_lidar/types.hpp"
#include "vtr_lidar/utils.hpp"
#include "vtr_logging/logging.hpp"
#include "vtr_tactic/types.hpp"

#include "vtr_lidar_msgs/msg/multi_exp_point_map.hpp"
#include "vtr_lidar_msgs/msg/point_map.hpp"
#include "vtr_lidar_msgs/msg/point_scan.hpp"

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

namespace vtr {
namespace lidar {

template <class PointT>
class PointScan {
 public:
  using PointScanMsg = vtr_lidar_msgs::msg::PointScan;

  using PointCloudType = pcl::PointCloud<PointT>;
  using TransformType = lgmath::se3::TransformationWithCovariance;

  /** \brief Static function that constructs this class from ROS2 message */
  static std::shared_ptr<PointScan<PointT>> fromStorable(
      const PointScanMsg& storable) {
    // construct with dl
    auto data = std::make_shared<PointScan<PointT>>();
    // load point cloud data
    pcl::fromROSMsg(storable.point_cloud, data->point_cloud_);
    // load vertex id
    data->vertex_id_ = tactic::VertexId(storable.vertex_id);
    // load transform
    common::fromROSMsg(storable.t_vertex_this, data->T_vertex_this_);
    return data;
  }

  /** \brief Returns the ROS2 message to be stored */
  PointScanMsg toStorable() const {
    PointScanMsg storable;
    // save point cloud data
    pcl::toROSMsg(this->point_cloud_, storable.point_cloud);
    // save vertex id
    storable.vertex_id = this->vertex_id_;
    // save transform
    common::toROSMsg(this->T_vertex_this_, storable.t_vertex_this);
    return storable;
  }

  virtual ~PointScan() = default;

  /** \brief Size of the map (number of point/voxel in the map) */
  size_t size() const { return point_cloud_.size(); }

  virtual TransformType& T_vertex_map() { return T_vertex_this_; }
  const TransformType& T_vertex_map() const { return T_vertex_this_; }

  virtual tactic::VertexId& vertex_id() { return vertex_id_; }
  const tactic::VertexId& vertex_id() const { return vertex_id_; }

  virtual PointCloudType& point_map() { return point_cloud_; }
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
  using PointMapMsg = vtr_lidar_msgs::msg::PointMap;

  /// constexpr of map version enum (keep in sync with the msg)
  static constexpr unsigned INITIAL = PointMapMsg::INITIAL;
  static constexpr unsigned INTRA_EXP_MERGED = PointMapMsg::INTRA_EXP_MERGED;
  static constexpr unsigned DYNAMIC_REMOVED = PointMapMsg::DYNAMIC_REMOVED;
  static constexpr unsigned INTER_EXP_MERGED = PointMapMsg::INTER_EXP_MERGED;

  /** \brief Static function that constructs this class from ROS2 message */
  static std::shared_ptr<PointMap<PointT>> fromStorable(
      const PointMapMsg& storable) {
    // construct with dl and version
    auto data =
        std::make_shared<PointMap<PointT>>(storable.dl, storable.version);
    // load point cloud data
    pcl::fromROSMsg(storable.point_cloud, data->point_cloud_);
    // load vertex id
    data->vertex_id_ = tactic::VertexId(storable.vertex_id);
    // load transform
    common::fromROSMsg(storable.t_vertex_this, data->T_vertex_this_);
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

  /** \brief Returns the ROS2 message to be stored */
  PointMapMsg toStorable() const {
    PointMapMsg storable;
    // save point cloud data
    pcl::toROSMsg(this->point_cloud_, storable.point_cloud);
    // save vertex id
    storable.vertex_id = this->vertex_id_;
    // save transform
    common::toROSMsg(this->T_vertex_this_, storable.t_vertex_this);
    // save version
    storable.version = this->version_;
    // save voxel size
    storable.dl = this->dl_;
    return storable;
  }

  PointMap(const float& dl, const unsigned& version = INITIAL)
      : dl_(dl), version_(version) {}

  virtual unsigned& version() { return version_; }
  const unsigned& version() const { return version_; }

  float dl() const { return dl_; }

  /** \brief Update map with a set of new points including movabilities. */
  virtual void update(const PointCloudType& point_cloud, bool filter = false) {
    // Reserve new space if needed
    updateCapacity(point_cloud.size());
    // Update the current map
    for (auto& p : point_cloud) {
      // filter based on icp score, optional
      /// \todo hardcoded 0.5
      if (filter && p.icp_score > 0.5) continue;
      // Get the corresponding key
      auto k = getKey(p);
      // Update the point count
      if (samples_.count(k) < 1)
        initSample(k, p);
      else
        updateSample(samples_[k], p);
    }
  }

 protected:
  VoxKey getKey(const PointT& p) const {
    VoxKey k((int)std::floor(p.x / dl_), (int)std::floor(p.y / dl_),
             (int)std::floor(p.z / dl_));
    return k;
  }

 private:
  virtual void updateCapacity(size_t num_pts) {
    // Reserve new space if needed
    if (samples_.empty()) samples_.reserve(10 * num_pts);
    this->point_cloud_.reserve(this->point_cloud_.size() + num_pts);
  }

  /** \brief Initialize a voxel centroid */
  virtual void initSample(const VoxKey& k, const PointT& p) {
    // We place a new key in the hashmap
    samples_.emplace(k, this->point_cloud_.size());
    // We add new voxel data
    this->point_cloud_.push_back(p);
    // Initializes point weights (will be updated in dynamic detection)
    this->point_cloud_.back().icp_score = 1;
  }

  /** \brief Update of voxel centroid */
  virtual void updateSample(const size_t idx, const PointT& p) {
    if (p.normal_score <= this->point_cloud_[idx].normal_score) return;

    // Update normal if we have a clear view of it and closer distance (see
    // computation of score)
    auto& p_ = this->point_cloud_[idx];
    // copy point normal information
    std::copy(std::begin(p.data_n), std::end(p.data_n), std::begin(p_.data_n));
    // copy normal score
    p_.normal_score = p.normal_score;
  }

 protected:
  /** \brief Voxel grid size */
  float dl_;

  /** \brief Version of the map */
  unsigned version_;

  /** \brief Sparse hashmap that contain voxels and map to point indices */
  std::unordered_map<VoxKey, size_t> samples_;

  template <class PointT2>
  friend class MultiExpPointMap;
};

template <class PointT>
class MultiExpPointMap : public PointMap<PointT> {
 public:
  using typename PointScan<PointT>::PointCloudType;
  using typename PointScan<PointT>::TransformType;
  using MultiExpPointMapMsg = vtr_lidar_msgs::msg::MultiExpPointMap;

  /** \brief Static function that constructs this class from ROS2 message */
  static std::shared_ptr<MultiExpPointMap<PointT>> fromStorable(
      const MultiExpPointMapMsg& storable) {
    // construct with dl and version
    auto data = std::make_shared<MultiExpPointMap<PointT>>(
        storable.dl, storable.max_num_exps);
    // load point cloud data
    pcl::fromROSMsg(storable.point_cloud, data->point_cloud_);
    // load vertex id
    data->vertex_id_ = tactic::VertexId(storable.vertex_id);
    // load transform
    common::fromROSMsg(storable.t_vertex_this, data->T_vertex_this_);
    // build voxel map
    data->samples_.clear();
    data->samples_.reserve(data->point_cloud_.size());
    size_t i = 0;
    for (const auto& p : data->point_cloud_) {
      auto result = data->samples_.emplace(data->getKey(p), i);
      if (!result.second)
        throw std::runtime_error{
            "MultiExpPointMap fromStorable detects points with same key. This "
            "should never happen."};
      i++;
    }
    // build the experience queue
    for (const auto& vid : storable.experiences)
      data->experiences_.emplace_back(vid);

    return data;
  }

  /** \brief Returns the ROS2 message to be stored */
  MultiExpPointMapMsg toStorable() const {
    MultiExpPointMapMsg storable;
    // save point cloud data
    pcl::toROSMsg(this->point_cloud_, storable.point_cloud);
    // save vertex id
    storable.vertex_id = this->vertex_id_;
    // save transform
    common::toROSMsg(this->T_vertex_this_, storable.t_vertex_this);
    // save voxel size
    storable.dl = this->dl_;
    // save max number of experiences
    storable.max_num_exps = max_num_exps_;
    // save experiences
    storable.experiences =
        std::vector<uint64_t>(experiences_.begin(), experiences_.end());
    return storable;
  }

  MultiExpPointMap(const float& dl, const size_t& max_num_exps)
      : PointMap<PointT>(dl, PointMap<PointT>::INTER_EXP_MERGED),
        max_num_exps_(max_num_exps) {
    if (max_num_exps_ < 1 || max_num_exps_ > 8 * sizeof((PointT*)0)->bits) {
      std::string err{
          "Invalid maximum number of experience ( <1 or exceeds point-wise bit "
          "vector length " +
          std::to_string(8 * sizeof((PointT*)0)->bits) +
          "): " + std::to_string(max_num_exps_)};
      LOG(ERROR) << err;
      throw std::runtime_error{err};
    }
  }

  void update(const PointMap<PointT>& point_map) {
    if (point_map.version_ < PointMap<PointT>::DYNAMIC_REMOVED) {
      std::string err{
          "Initializing/updating a multi-experience point map with non-dynamic "
          "removed point map is disallowed."};
      LOG(ERROR) << err;
      throw std::runtime_error{err};
    }
    // first experience (this must be a privileged experience)
    if (experiences_.empty()) {
      this->vertex_id_ = point_map.vertex_id_;
      this->T_vertex_this_ = point_map.T_vertex_this_;
      // update points
      for (auto& p : point_map.point_cloud_) {
        /// \todo hard-coded 0.5 for short-term removal
        if (p.icp_score > 0.5) continue;
        // Get the corresponding key
        auto k = this->getKey(p);
        // Update the point count
        if (this->samples_.count(k) < 1)
          initSample(k, p);
        else {
          std::string err{"Found grid collision during initial map update."};
          LOG(ERROR) << err;
          throw std::runtime_error{err};
        }
      }
    }
    // update with another experience
    else {
      // shift the bit vector by 1 for the new experience and update score
      std::for_each(this->point_cloud_.begin(), this->point_cloud_.end(),
                    [&](PointT& p) {
                      p.icp_score -= (p.bits >> (max_num_exps_ - 1));
                      p.bits <<= 1;
                    });
      // update points
      for (auto& p : point_map.point_cloud_) {
        /// \todo hard-coded 0.5 for short-term removal
        if (p.icp_score > 0.5) continue;
        // Get the corresponding key
        auto k = this->getKey(p);
        // Update the point count
        if (this->samples_.count(k) < 1)
          initSample(k, p);
        else
          updateSample(this->samples_[k], p);
      }
    }
    // remove points with bit vector zero
    /// \todo this currently does not update samples_ map, leaving point map in
    /// an inconsistent state
    for (auto it = this->point_cloud_.begin();
         it != this->point_cloud_.end();) {
      if (it->bits << (8 * sizeof((PointT*)0)->bits - max_num_exps_) == 0)
        it = this->point_cloud_.erase(it);
      else
        ++it;
    }

    // update the experience vector
    experiences_.push_back(point_map.vertex_id_);
    if (experiences_.size() > max_num_exps_) experiences_.pop_front();
  }

  const std::deque<tactic::VertexId>& experiences() const {
    return experiences_;
  }

 protected:
  /** \brief Initialize a voxel centroid */
  void initSample(const VoxKey& k, const PointT& p) override {
    // We place a new key in the hashmap
    this->samples_.emplace(k, this->point_cloud_.size());
    // We add new voxel data
    this->point_cloud_.push_back(p);
    // initialize the bit vector
    this->point_cloud_.back().bits = 1;
    this->point_cloud_.back().icp_score = 1;
  }

  /** \brief Update of voxel centroid */
  void updateSample(const size_t idx, const PointT& p) override {
    // Update normal if we have a clear view of it and closer distance (see
    // computation of score)
    auto& p_ = this->point_cloud_[idx];
    // copy point normal information
    std::copy(std::begin(p.data_n), std::end(p.data_n), std::begin(p_.data_n));
    // copy time info
    p_.time = p.time;
    // copy normal score
    p_.normal_score = p.normal_score;
    // update bit vector (only if it has not been updated yet)
    if ((p_.bits & 1) == 0) {
      p_.bits++;
      p_.icp_score++;
    }
  }

 protected:
  void update(const PointCloudType&, bool) override {}

 private:
  size_t max_num_exps_;

  std::deque<tactic::VertexId> experiences_;
};

}  // namespace lidar
}  // namespace vtr