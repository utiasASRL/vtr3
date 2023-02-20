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
 * \file base_path_planner.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_common/utils/macros.hpp"

namespace vtr {
namespace path_planning {

/** \brief Path planner interface for state machine use */
class PathPlannerInterface {
 public:
  PTR_TYPEDEFS(PathPlannerInterface);

  virtual ~PathPlannerInterface() = default;

  /** \brief Sets whether control command should be computed */
  virtual void setRunning(const bool) = 0;
};

}  // namespace path_planning
}  // namespace vtr
