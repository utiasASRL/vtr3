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
 * \file speed_scheduler.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

#pragma once
#include <vector>
#include <vtr_tactic/types.hpp>
#include <vtr_path_planning/mpc/mpc_common.hpp>



namespace vtr::path_planning
{
struct SpeedSchedConfig {
    double target_vel = 1;
    double min_vel = 0;
    
    double planar_curv_weight = 0;
    double profile_curv_weight = 0;
    double eop_weight = 0;
    unsigned horizon_steps = 1;
};

double ScheduleSpeed(tactic::LocalizationChain::Ptr chain, const SpeedSchedConfig& params);
  
} // namespace vtr::path_planning