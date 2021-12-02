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
 * \file state_machine_interface.cpp
 * \brief Interface for state machines
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_tactic/state_machine_interface.hpp"

namespace vtr {
namespace tactic {

std::ostream& operator<<(std::ostream& os, const PipelineMode& mode) {
  switch (mode) {
    case PipelineMode::Idle:
      os << "Idle";
      return os;
    case PipelineMode::Branching:
      os << "Branching";
      return os;
    case PipelineMode::Merging:
      os << "Merging";
      return os;
    case PipelineMode::Following:
      os << "Following";
      return os;
    case PipelineMode::Searching:
      os << "Searching";
      return os;
    default:
      os << "Unknown";
      return os;
  }
}

}  // namespace tactic
}  // namespace vtr