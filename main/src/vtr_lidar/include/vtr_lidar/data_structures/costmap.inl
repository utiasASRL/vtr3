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
 * \file costmap.inl
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/data_structures/costmap.hpp"

namespace vtr {
namespace lidar {

template <typename PointT>
PixKey BaseCostMap::getKey(const PointT& p) const {
  return PixKey((int)std::round(p.x / dl_), (int)std::round(p.y / dl_));
}

template <typename PointT>
bool BaseCostMap::contains(const PointT& p) const {
  return (p.x >= -size_x_ / 2.0f) && (p.x <= size_x_ / 2.0f) &&
         (p.y >= -size_y_ / 2.0f) && (p.y <= size_y_ / 2.0f);
}

template <typename PointCloud, typename ReductionOp = SparseCostMap::AvgOp>
void SparseCostMap::update(const PointCloud& points,
                           const std::vector<float>& values,
                           const ReductionOp& op) {
  for (size_t i = 0; i < points.size(); i++) {
    const auto k = getKey(points[i]);
    if (!contains(points[i])) continue;
    auto res = raw_values_.try_emplace(k, std::vector<float>());
    res.first->second.emplace_back(values[i]);
  }
  // re-compute cost map values
  values_.clear();
  for (const auto& [key, values] : raw_values_) {
    const auto value = op(values.begin(), values.end());
    values_.try_emplace(key, value);
  }
}

}  // namespace lidar
}  // namespace vtr