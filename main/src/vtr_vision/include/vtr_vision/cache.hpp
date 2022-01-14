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
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_tactic/cache.hpp"
#include "vtr_vision/types.hpp"

namespace vtr {
namespace vision {

struct VisionQueryCache : public tactic::QueryCache {
  using Ptr = std::shared_ptr<VisionQueryCache>;

  // clang-format off
  tactic::Cache<std::list<vision::RigCalibration>> rig_calibrations;
  tactic::Cache<std::list<vision::RigImages>> rig_images;
  tactic::Cache<std::vector<vision::RigFeatures>> rig_features;
  // clang-format on
};

struct VisionOutputCache : public tactic::OutputCache {
  using Ptr = std::shared_ptr<VisionOutputCache>;
};

}  // namespace vision
}  // namespace vtr