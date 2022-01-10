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
 * \file occupancy_grid.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 * \brief OccupancyGrid detection functions.
 */
#pragma once

#include <algorithm>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "vtr_common/conversions/ros_lgmath.hpp"
#include "vtr_lidar/utils.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace lidar {

class OccupancyGrid {
 public:
  using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;

  /**
   * \brief Returns the ROS2 message to be stored
   * \todo fromStorable and make this function return a customized message
   */
  OccupancyGridMsg toStorable() const {
    OccupancyGridMsg storable;

    // transform from (0, 0) of the ros msg to (0, 0) of this class
    Eigen::Matrix4d T_this_ros_mat = Eigen::Matrix4d::Identity();
    T_this_ros_mat(0, 3) = origin_.x * dl_ - dl_ / 2.0;
    T_this_ros_mat(1, 3) = origin_.y * dl_ - dl_ / 2.0;
    tactic::EdgeTransform T_this_ros(T_this_ros_mat);
    const auto T_vertex_ros = T_vertex_this_ * T_this_ros;

    // clamp and fill in data
    std::vector<int8_t> data(width_ * height_, 0);
    for (const auto& val : values_) {
      const auto shifted_k = val.first - origin_;
      data[shifted_k.x + shifted_k.y * width_] =
          (int8_t)(std::clamp(val.second, 0.f, 1.f) * 100);
    }

    storable.info.resolution = dl_;
    storable.info.width = width_;
    storable.info.height = height_;
    storable.info.origin = common::conversions::toPoseMessage(T_vertex_ros);
    storable.data = data;

    return storable;
  }

  /**
   * \param[in] dl resolution of the grid
   * \param[in] size_x total size of the grid in x direction [meter]
   * \param[in] size_y total size of the grid in y direction [meter]
   */
  OccupancyGrid(const float& dl, const float& size_x, const float& size_y)
      : dl_(dl),
        max_x_(abs(size_x)),
        max_y_(abs(size_y)),
        width_(2 * std::round(max_x_ / 2.0f / dl) + 1),
        height_(2 * std::round(max_y_ / 2.0f / dl) + 1),
        origin_(-std::round(max_x_ / 2.0f / dl_),
                -std::round(max_y_ / 2.0f / dl_)) {
#if false
    LOG(DEBUG) << "OccupancyGrid: dl: " << dl_ << ", max_x_: " << max_x_
               << ", max_y_: " << max_y_ << ", width_: " << width_
               << ", height_: " << height_ << ", origin_: " << origin_.x << ", "
               << origin_.y;
#endif
  }

  float dl() const { return dl_; }

  virtual tactic::EdgeTransform& T_vertex_this() { return T_vertex_this_; }
  const tactic::EdgeTransform& T_vertex_this() const { return T_vertex_this_; }
  virtual tactic::VertexId& vertex_id() { return vertex_id_; }
  const tactic::VertexId& vertex_id() const { return vertex_id_; }
  virtual unsigned& vertex_sid() { return vertex_sid_; }
  const unsigned& vertex_sid() const { return vertex_sid_; }

  using XY2ValueMap = std::unordered_map<std::pair<float, float>, float>;
  XY2ValueMap getOccupied() const {
    XY2ValueMap occupied;
    occupied.reserve(values_.size());
    for (const auto& val : values_) {
      const auto key = val.first;
      occupied.emplace(
          std::make_pair((float)(key.x * dl_), (float)(key.y * dl_)),
          val.second);
    }
    return occupied;
  }

  struct AvgOp {
    using InputIt = std::vector<float>::const_iterator;
    float operator()(InputIt first, InputIt last) const {
      float count = 0.;
      float sum = 0.;
      for (; first != last; ++first) {
        sum += *first;
        ++count;
      }
      return count == 0 ? 0.0 : sum / count;
    }
  };

  template <typename PointCloud, typename ReductionOp = AvgOp>
  void update(const PointCloud& points, const std::vector<float>& values,
              const ReductionOp& op = ReductionOp()) {
    for (size_t i = 0; i < points.size(); i++) {
      const auto k = getKey(points[i]);
      if (!contains(k)) continue;
      auto res = raw_values_.try_emplace(k, std::vector<float>());
      res.first->second.emplace_back(values[i]);
    }
    // re-compute ogm values
    values_.clear();
    for (const auto& [key, values] : raw_values_) {
      const auto value = op(values.begin(), values.end());
      values_.try_emplace(key, value);
    }
  }

 private:
  template <typename PointT>
  PixKey getKey(const PointT& p) const {
    return PixKey((int)std::round(p.x / dl_), (int)std::round(p.y / dl_));
  }

  bool contains(const PixKey& k) const {
    const auto shifted_k = k - origin_;
    return (shifted_k.x >= 0 && shifted_k.x < width_ && shifted_k.y >= 0 &&
            shifted_k.y < height_);
  }

 private:
  const float dl_;
  const float max_x_, max_y_;
  const int width_, height_;
  const PixKey origin_;

  /** \brief the associated vertex sequence id */
  unsigned vertex_sid_ = -1;
  /** \brief the associated vertex id */
  tactic::VertexId vertex_id_ = tactic::VertexId::Invalid();
  /** \brief the transform from this scan/map to its associated vertex */
  tactic::EdgeTransform T_vertex_this_ = tactic::EdgeTransform(true);

  std::unordered_map<PixKey, std::vector<float>> raw_values_;
  std::unordered_map<PixKey, float> values_;
};

}  // namespace lidar
}  // namespace vtr