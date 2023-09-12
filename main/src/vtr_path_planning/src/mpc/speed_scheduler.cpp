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
 * \file speed_scheduler.cpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/mpc/speed_scheduler.hpp"
#include <algorithm>
#include <iostream>


double ScheduleSpeed(const std::vector<double>& disc_path_curvature_xy, const std::vector<double>& disc_path_curvature_xz_yz, double VF, int curr_sid, double planar_curv_weight, double profile_curv_weight, double eop_weight, double horizon_step_size, double min_vel)
{

    // Experimental Speed Scheduler:
    // Takes in the desired forward_velocity and the pre-processed global path and reduces the set speed based on a range of tunable factors:
    // 1. Planar Curvature
    // 2. Profile Curvature
    // 3. End of Path

    // Pseudocode:
    // - Estimate the current p value of the vehicle (doesnt need to be super precise so here we can imply opt to use the sid value)
    // - Avergage the radius of curvature in the upcoming segments of the path

    // Basic implementation - weights hardcoded for now
    CLOG(INFO, "mpc.speed_scheduler") << "TRYING TO SCHEDULE SPEED:";
    CLOG(INFO, "mpc.speed_scheduler") << "CURRENT SID IS:" << curr_sid;
    double VF_EOP;
    double VF_XY;
    double VF_XZ_YZ;
    double avg_curvature_xy = 0.0;
    double avg_curvature_xz_yz = 0.0;
    double end_of_path = 0.0;
    int horizon_steps = 5.0 / horizon_step_size; // Set lookahead horizon to 5m (default params tuned for this value)
    for (int i = curr_sid; i < curr_sid + horizon_steps; i++) 
    {
      // Handle end of path case
      if (i == (disc_path_curvature_xy.size()-1))
      {
        end_of_path = 1.0;
        break;
      }
      avg_curvature_xy = avg_curvature_xy + disc_path_curvature_xy[i];
      avg_curvature_xz_yz = avg_curvature_xz_yz + disc_path_curvature_xz_yz[i];

    }
    avg_curvature_xy = avg_curvature_xy / horizon_steps;
    avg_curvature_xz_yz = avg_curvature_xz_yz / horizon_steps;
    CLOG(INFO, "mpc.speed_scheduler") << "THE AVERAGE PLANAR CURVATURE IS:  " << avg_curvature_xy;
    CLOG(INFO, "mpc.speed_scheduler") << "THE AVERAGE PROFILE CURVATURE IS:  " << avg_curvature_xz_yz;

    // handle forward/referse case and calculate a candidate VF speed for each of our scheduler modules (XY curvature, XZ curvature, End of Path etc)

    VF_EOP = std::max(0.5, VF / (1 + (end_of_path * end_of_path * eop_weight)));
    VF_XY = std::max(0.5, VF / (1 + (avg_curvature_xy * avg_curvature_xy * planar_curv_weight)));
    VF_XZ_YZ = std::max(0.5, VF / (1 + (avg_curvature_xz_yz * avg_curvature_xz_yz * profile_curv_weight)));
    
    // Take the minimum of all candidate (positive) scheduled speeds (Lowest allowed scheduled velocity is 0.5m/s, should be left this way)
    VF = std::min({VF_EOP, VF_XY, VF_XZ_YZ});
    CLOG(INFO, "mpc.speed_scheduler") << "THE VF_EOP SPEED IS:  " << VF_EOP;
    CLOG(INFO, "mpc.speed_scheduler") << "THE VF_XY SPEED IS:  " << VF_XY;
    CLOG(INFO, "mpc.speed_scheduler") << "THE VF_XZ SPEED IS:  " << VF_XZ_YZ;

    // Take the minimum of all candidate scheduled speeds
    CLOG(INFO, "mpc.speed_scheduler") << "THE SPEED SCHEDULED SPEED IS:  " << VF;
    // End of speed scheduler code

    // Return the scheduled speed
    return VF;
}