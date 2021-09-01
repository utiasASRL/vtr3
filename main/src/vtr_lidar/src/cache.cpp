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
 * \file cache.cpp
 * \brief Template instantiation of Lidar pipeline specific caches
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/cache.hpp>
#include <vtr_tactic/utils/cache_container.inl>

namespace vtr {
namespace common {
template class cache_ptr<sensor_msgs::msg::PointCloud2>;
template class cache_ptr<std::vector<lidar::PointXYZ>>;
template class cache_ptr<Eigen::Matrix4d>;
template class cache_ptr<lidar::IncrementalPointMap>;
template class cache_ptr<lidar::SingleExpPointMap>;
template class cache_ptr<lidar::MultiExpPointMap>;
}  // namespace common
}  // namespace vtr