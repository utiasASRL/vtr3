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
#include "vtr_logging/logging.hpp"
#include "vtr_path_planning/cbit/generate_pq.hpp"


double ScheduleSpeed(const std::vector<double>& disc_path_curvature_xy, const std::vector<double>& disc_path_curvature_xz_yz, double VF, unsigned curr_sid, double planar_curv_weight, double profile_curv_weight, double eop_weight, double horizon_step_size, double min_vel);
