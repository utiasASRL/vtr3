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
 * \file cache.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/cache.hpp"
#include "vtr_radar/cache.hpp"

namespace vtr {
namespace radar_lidar {

struct RadarLidarQueryCache : public lidar::LidarQueryCache,
                              public radar::RadarQueryCache {
  using Ptr = std::shared_ptr<RadarLidarQueryCache>;
};

struct RadarLidarOutputCache : public lidar::LidarOutputCache,
                               public radar::RadarOutputCache {
  using Ptr = std::shared_ptr<RadarLidarOutputCache>;
};

}  // namespace radar_lidar
}  // namespace vtr