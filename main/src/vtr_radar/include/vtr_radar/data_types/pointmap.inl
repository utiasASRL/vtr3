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
 * \file pointmap.inl
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_radar/data_types/pointmap.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "vtr_common/conversions/ros_lgmath.hpp"

namespace vtr {
namespace radar {

template <class PointT>
auto PointMap<PointT>::fromStorable(const PointMapMsg& storable) -> Ptr {
  // construct with dl and version
  auto data = std::make_shared<PointMap<PointT>>(storable.dl, storable.version);
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
          "PointMap fromStorable detects points with same key. This should "
          "never happen."};
    i++;
  }
  return data;
}

template <class PointT>
auto PointMap<PointT>::toStorable() const -> PointMapMsg {
  PointMapMsg storable;
  // save point cloud data
  pcl::toROSMsg(this->point_cloud_, storable.point_cloud);
  // save vertex id
  storable.vertex_id = this->vertex_id_;
  // save transform
  using namespace vtr::common;
  conversions::toROSMsg(this->T_vertex_this_, storable.t_vertex_this);
  // save version
  storable.version = this->version_;
  // save voxel size
  storable.dl = this->dl_;
  return storable;
}

template <class PointT>
template <class Callback>
void PointMap<PointT>::update(const PointCloudType& point_cloud,
                              const Callback& callback) {
  // reserve new space if needed
  if (samples_.empty()) samples_.reserve(10 * point_cloud.size());
  this->point_cloud_.reserve(this->point_cloud_.size() + point_cloud.size());

  // Update the current map
  for (auto& p : point_cloud) {
    const auto res = samples_.try_emplace(getKey(p), this->point_cloud_.size());
    if (res.second) this->point_cloud_.emplace_back(p);
    callback(/* success */ res.second,
             /* curr_pt */ this->point_cloud_[res.first->second],
             /* new_pt */ p);
  }
}

template <class PointT>
template <class Callback>
void PointMap<PointT>::filter(const Callback& callback) {
  //
  std::vector<int> indices;
  indices.reserve(this->point_cloud_.size());
  for (size_t i = 0; i < this->point_cloud_.size(); ++i) {
    if (callback(this->point_cloud_[i])) indices.emplace_back(i);
  }
  // create a copy of the point cloud and apply filter
  const auto point_cloud = this->point_cloud_;
  pcl::copyPointCloud(point_cloud, indices, this->point_cloud_);
  // rebuild the voxel map
  samples_.clear();
  samples_.reserve(this->point_cloud_.size());
  for (size_t i = 0; i < this->point_cloud_.size(); ++i) {
    auto result = samples_.emplace(getKey(this->point_cloud_[i]), i);
    if (!result.second)
      throw std::runtime_error{
          "PointMap fromStorable detects points with same key. This should "
          "never happen."};
  }
}

}  // namespace radar
}  // namespace vtr