// Copyright 2024, Autonomous Space Robotics Lab (ASRL)
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
 * \file corridor_filter.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/data_types/point.hpp"
#include "math.h"
#include "vtr_tactic/types.hpp"


namespace vtr {
namespace lidar{

pcl::PointCloud<PointWithInfo> filter_by_corridor(const pcl::PointCloud<PointWithInfo>& point_cloud, long curr_sid, double path_length, const tactic::LocalizationChain& chain, double corridor_width, 
        const tactic::EdgeTransform &T_cam_w);
} // namespace lidar
} // vtr