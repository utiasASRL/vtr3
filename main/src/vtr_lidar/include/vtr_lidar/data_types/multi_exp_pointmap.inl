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
 * \file multi_exp_pointmap.inl
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/data_types/multi_exp_pointmap.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "vtr_common/conversions/ros_lgmath.hpp"
#include "vtr_logging/logging.hpp"

namespace vtr {
namespace lidar {

template <class PointT>
auto MultiExpPointMap<PointT>::fromStorable(const MultiExpPointMapMsg& storable)
    -> Ptr {
  // construct with dl and version
  auto data = std::make_shared<MultiExpPointMap<PointT>>(storable.dl,
                                                         storable.max_num_exps);
  // load point cloud data
  pcl::fromROSMsg(storable.point_cloud, data->point_cloud_);
  // load vertex id
  data->vertex_id_ = tactic::VertexId(storable.vertex_id);
  // load transform
  using namespace vtr::common;
  conversions::fromROSMsg(storable.t_vertex_this, data->T_vertex_this_);
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
  for (const auto& vid : storable.experiences) data->exps_.emplace_back(vid);

  return data;
}

template <class PointT>
auto MultiExpPointMap<PointT>::toStorable() const -> MultiExpPointMapMsg {
  MultiExpPointMapMsg storable;
  // save point cloud data
  pcl::toROSMsg(this->point_cloud_, storable.point_cloud);
  // save vertex id
  storable.vertex_id = this->vertex_id_;
  // save transform
  using namespace vtr::common;
  conversions::toROSMsg(this->T_vertex_this_, storable.t_vertex_this);
  // save voxel size
  storable.dl = this->dl_;
  // save max number of experiences
  storable.max_num_exps = max_num_exps_;
  // save experiences
  storable.experiences = std::vector<uint64_t>(exps_.begin(), exps_.end());
  return storable;
}

template <class PointT>
MultiExpPointMap<PointT>::MultiExpPointMap(const float& dl,
                                           const size_t& max_num_exps)
    : PointMap<PointT>(dl, PointMap<PointT>::INTER_EXP_MERGED),
      max_num_exps_(max_num_exps) {
  if (max_num_exps_ < 1 || max_num_exps_ > 8 * sizeof((PointT*)0)->bits) {
    std::string err{
        "Invalid maximum number of experience ( <1 or exceeds point-wise bit "
        "vector length " +
        std::to_string(8 * sizeof((PointT*)0)->bits) +
        "): " + std::to_string(max_num_exps_)};
    CLOG(ERROR, "multi_exp_pointmap") << err;
    throw std::runtime_error{err};
  }
}

template <class PointT>
void MultiExpPointMap<PointT>::update(const PointMap<PointT>& point_map) {
  if (point_map.version() < PointMap<PointT>::DYNAMIC_REMOVED) {
    std::string err{
        "Initializing/updating a multi-experience point map with non-dynamic "
        "removed point map is disallowed."};
    CLOG(ERROR, "multi_exp_pointmap") << err;
    throw std::runtime_error{err};
  }
  // first experience (this must be a privileged experience)
  if (exps_.empty()) {
    this->vertex_id_ = point_map.vertex_id();
    this->T_vertex_this_ = point_map.T_vertex_this();
    // update points
    for (auto& p : point_map.point_cloud()) {
      /// \todo hard-coded 0.5 for short-term removal
      if (p.static_score < 0.5) continue;
      // Get the corresponding key
      auto k = this->getKey(p);
      // Update the point count
      if (this->samples_.count(k) < 1)
        initSample(k, p);
      else {
        std::string err{"Found grid collision during initial map update."};
        CLOG(ERROR, "lidar.pointmap") << err;
        throw std::runtime_error{err};
      }
    }
  }
  // update with another experience
  else {
    // shift the bit vector by 1 for the new experience and update score
    std::for_each(this->point_cloud_.begin(), this->point_cloud_.end(),
                  [&](PointT& p) { p.bits <<= 1; });
    // update points
    for (auto& p : point_map.point_cloud()) {
      if (p.static_score < 0.5) continue;  /// \todo hard-coded 0.5
      // Get the corresponding key
      auto k = this->getKey(p);
      // Update the point count
      if (this->samples_.count(k) < 1)
        initSample(k, p);
      else
        updateSample(this->samples_[k], p);
      /// \todo point cloud maybe sparse, so probably also need to update its
      /// spatial neighbors (based on normal agreement)
    }
  }
  // remove points with bit vector zero
  /// \todo this currently does not update samples_ map, leaving point map in
  /// an inconsistent state
  for (auto it = this->point_cloud_.begin(); it != this->point_cloud_.end();) {
    if (it->bits << (8 * sizeof((PointT*)0)->bits - max_num_exps_) == 0)
      it = this->point_cloud_.erase(it);
    else
      ++it;
  }

  // update the experience vector
  exps_.push_back(point_map.vertex_id());
  if (exps_.size() > max_num_exps_) exps_.pop_front();
}

template <class PointT>
void MultiExpPointMap<PointT>::initSample(const VoxKey& k, const PointT& p) {
  // We place a new key in the hashmap
  this->samples_.emplace(k, this->point_cloud_.size());
  // We add new voxel data
  this->point_cloud_.push_back(p);
  // initialize the bit vector
  this->point_cloud_.back().bits = 1;
}

template <class PointT>
void MultiExpPointMap<PointT>::updateSample(const size_t idx, const PointT& p) {
  auto& p_ = this->point_cloud_[idx];
  // copy point normal information
  std::copy(std::begin(p.data_n), std::end(p.data_n), std::begin(p_.data_n));
  // copy time info
  p_.time = p.time;
  // copy normal variance and score
  p_.normal_score = p.normal_score;
  // update bit vector (only if it has not been updated yet)
  if ((p_.bits & 1) == 0) p_.bits++;
}

}  // namespace lidar
}  // namespace vtr