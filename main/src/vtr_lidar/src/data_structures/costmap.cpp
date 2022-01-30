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
 * \file costmap.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/data_structures/costmap.hpp"

#include "pcl_conversions/pcl_conversions.h"

namespace vtr {
namespace lidar {

BaseCostMap::BaseCostMap(const float& dl, const float& size_x,
                         const float& size_y, const float& default_value)
    : dl_(dl),
      size_x_(abs(size_x)),
      size_y_(abs(size_y)),
      default_value_(default_value),
      width_(2 * std::round(size_x_ / 2.0f / dl_) + 1),
      height_(2 * std::round(size_y_ / 2.0f / dl_) + 1),
      origin_(-std::round(size_x_ / 2.0f / dl_),
              -std::round(size_y_ / 2.0f / dl_)) {
#if false
    LOG(DEBUG) << "BaseCostMap: dl: " << dl_ << ", size_x_: " << size_x_
               << ", size_y_: " << size_y_ << ", width_: " << width_
               << ", height_: " << height_ << ", origin_: " << origin_.x << ", "
               << origin_.y;
#endif
}

void BaseCostMap::getValue(const teb_local_planner::PoseSE2& pose,
                           float& value) const {
  /// transform into costmap's frame
  Eigen::Vector4d p(pose.x(), pose.y(), 0.0, 1.0);
  Eigen::Vector4d p_this = T_this_plan_ * p;
  const auto x = p_this(0);
  const auto y = p_this(1);
  /// bilinear interpolation
  const double xt = x / dl_;
  const double yt = y / dl_;
  const int xf = std::floor(xt), xc = std::ceil(xt);
  const int yf = std::floor(yt), yc = std::ceil(yt);
  const double dx = xt - (double)xf;
  const double dy = yt - (double)yf;
  //
  const auto v00 = at(PixKey(xf, yf));
  const auto v10 = at(PixKey(xc, yf));
  const auto v01 = at(PixKey(xf, yc));
  const auto v11 = at(PixKey(xc, yc));
  //
  const auto vt0 = v00 + dx * (v10 - v00);
  const auto vt1 = v01 + dx * (v11 - v01);
  const auto vtt = vt0 + dy * (vt1 - vt0);
  //
  value = vtt;
}

void BaseCostMap::getValue(const teb_local_planner::VertexPose& pose,
                           float& value) const {
  getValue(teb_local_planner::PoseSE2(pose.x(), pose.y(), pose.theta()), value);
}

bool BaseCostMap::contains(const PixKey& k) const {
  const auto shifted_k = k - origin_;
  return (shifted_k.x >= 0 && shifted_k.x < width_ && shifted_k.y >= 0 &&
          shifted_k.y < height_);
}

DenseCostMap::DenseCostMap(const float& dl, const float& size_x,
                           const float& size_y, const float& default_value)
    : BaseCostMap(dl, size_x, size_y, default_value),
      values_(Eigen::MatrixXf::Constant(width_, height_, default_value_)) {}

auto DenseCostMap::toCostMapMsg() const -> CostMapMsg {
  CostMapMsg costmap_msg;

  // transform from (0, 0) of the ros msg to (0, 0) of this class
  Eigen::Matrix4d T_this_ros_mat = Eigen::Matrix4d::Identity();
  T_this_ros_mat(0, 3) = origin_.x * dl_ - dl_ / 2.0;
  T_this_ros_mat(1, 3) = origin_.y * dl_ - dl_ / 2.0;
  tactic::EdgeTransform T_this_ros(T_this_ros_mat);

  // clamp and fill in data
  std::vector<int8_t> data(width_ * height_, default_value_);
  for (int x = 0; x < width_; ++x)
    for (int y = 0; y < height_; ++y)
      data[x + y * width_] =
          (int8_t)(std::clamp(values_(x, y), 0.f, 1.f) * 100);

  costmap_msg.info.resolution = dl_;
  costmap_msg.info.width = width_;
  costmap_msg.info.height = height_;
  costmap_msg.info.origin = common::conversions::toPoseMessage(T_this_ros);
  costmap_msg.data = data;

  return costmap_msg;
}

auto DenseCostMap::toPointCloudMsg() const -> PointCloudMsg {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  for (int x = 0; x < width_; ++x)
    for (int y = 0; y < height_; ++y) {
      pcl::PointXYZI point;
      point.x = (x + origin_.x) * dl_;
      point.y = (y + origin_.y) * dl_;
      point.z = 0.0f;
      point.intensity = values_(x, y);
      pointcloud.emplace_back(point);
    }

  PointCloudMsg pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  return pointcloud_msg;
}

void DenseCostMap::update(const std::unordered_map<PixKey, float>& values) {
  for (const auto& val : values) {
    const auto shifted_k = val.first - origin_;
    values_(shifted_k.x, shifted_k.y) = val.second;
  }
}

auto DenseCostMap::filter(const float& threshold) const -> XY2ValueMap {
  XY2ValueMap filtered;
  filtered.reserve(values_.size());
  for (int x = 0; x < width_; ++x)
    for (int y = 0; y < height_; ++y) {
      if (values_(x, y) < threshold) continue;
      const auto key = PixKey(x, y) + origin_;
      filtered.emplace(
          std::make_pair((float)(key.x * dl_), (float)(key.y * dl_)),
          values_(x, y));
    }
  return filtered;
}

float DenseCostMap::at(const PixKey& k) const {
  if (!contains(k)) return default_value_;
  const auto shifted_k = k - origin_;
  return values_(shifted_k.x, shifted_k.y);
}

SparseCostMap::SparseCostMap(const float& dl, const float& size_x,
                             const float& size_y, const float& default_value)
    : BaseCostMap(dl, size_x, size_y, default_value) {}

DenseCostMap SparseCostMap::toDense() const {
  DenseCostMap dense_cost_map(dl_, size_x_, size_y_, default_value_);
  dense_cost_map.update(values_);
  dense_cost_map.vertex_sid() = vertex_sid_;
  dense_cost_map.vertex_id() = vertex_id_;
  dense_cost_map.T_vertex_this() = T_vertex_this_;
  return dense_cost_map;
}

auto SparseCostMap::filter(const float& threshold) const -> XY2ValueMap {
  XY2ValueMap filtered;
  filtered.reserve(values_.size());
  for (const auto& val : values_) {
    const auto key = val.first;
    if (val.second < threshold) continue;
    filtered.emplace(std::make_pair((float)(key.x * dl_), (float)(key.y * dl_)),
                     val.second);
  }
  return filtered;
}

auto SparseCostMap::toCostMapMsg() const -> CostMapMsg {
  CostMapMsg costmap_msg;

  // transform from (0, 0) of the ros msg to (0, 0) of this class
  Eigen::Matrix4d T_this_ros_mat = Eigen::Matrix4d::Identity();
  T_this_ros_mat(0, 3) = origin_.x * dl_ - dl_ / 2.0;
  T_this_ros_mat(1, 3) = origin_.y * dl_ - dl_ / 2.0;
  tactic::EdgeTransform T_this_ros(T_this_ros_mat);
  const auto T_vertex_ros = T_vertex_this_ * T_this_ros;

  // clamp and fill in data
  std::vector<int8_t> data(width_ * height_, default_value_);
  for (const auto& val : values_) {
    const auto shifted_k = val.first - origin_;
    data[shifted_k.x + shifted_k.y * width_] =
        (int8_t)(std::clamp(val.second, 0.f, 1.f) * 100);
  }

  costmap_msg.info.resolution = dl_;
  costmap_msg.info.width = width_;
  costmap_msg.info.height = height_;
  costmap_msg.info.origin = common::conversions::toPoseMessage(T_vertex_ros);
  costmap_msg.data = data;

  return costmap_msg;
}

float SparseCostMap::at(const PixKey& k) const {
  if (!values_.count(k)) return default_value_;
  return values_.at(k);
}

}  // namespace lidar
}  // namespace vtr