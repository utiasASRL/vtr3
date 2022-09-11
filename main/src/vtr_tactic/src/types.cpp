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
 * \file types.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace tactic {

std::ostream& operator<<(std::ostream& os, const PipelineMode& mode) {
  switch (mode) {
    case PipelineMode::Idle:
      os << "Idle";
      return os;
    case PipelineMode::TeachMetricLoc:
      os << "TeachMetricLoc";
      return os;
    case PipelineMode::TeachBranch:
      os << "TeachBranch";
      return os;
    case PipelineMode::TeachMerge:
      os << "TeachMerge";
      return os;
    case PipelineMode::RepeatMetricLoc:
      os << "RepeatMetricLoc";
      return os;
    case PipelineMode::RepeatFollow:
      os << "RepeatFollow";
      return os;
    default:
      os << "Unknown";
      return os;
  }
}

}  // namespace tactic
}  // namespace vtr