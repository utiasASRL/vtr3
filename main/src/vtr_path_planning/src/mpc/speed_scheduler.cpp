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

namespace vtr::path_planning
{
  double ScheduleSpeed(tactic::LocalizationChain::Ptr chain, const SpeedSchedConfig& params) {
    
    unsigned curr_sid = chain->trunkSequenceId();

    CLOG(DEBUG, "mpc.speed_scheduler") << "TRYING TO SCHEDULE SPEED:";
    CLOG(DEBUG, "mpc.speed_scheduler") << "CURRENT SID IS:" << curr_sid;

    double VF_EOP = std::max(params.min_vel, params.target_vel * (params.eop_weight * (chain->size()- 1 - curr_sid) / 20.0));
    double VF_SOP = std::max(params.min_vel, params.target_vel * (params.eop_weight * curr_sid / 10));


    double VF_XY;
    double avg_curvature = 0.0;
    unsigned window_steps = 0;
    for (auto itr = chain->begin(curr_sid); itr != chain->end() && unsigned(itr) < curr_sid + params.horizon_steps; itr++) {
      const auto curvature = CurvatureInfo::fromTransform(itr->T());
      avg_curvature += curvature.curvature();
      ++window_steps;
    }
    avg_curvature /= window_steps;
    CLOG(DEBUG, "mpc.speed_scheduler") << "THE AVERAGE CURVATURE IS:  " << avg_curvature;

    // handle forward/reverse case and calculate a candidate VF speed for each of our scheduler modules (XY curvature, XZ curvature, End of Path etc)

    VF_XY = std::max(params.min_vel, params.target_vel / (1 + (avg_curvature * avg_curvature * params.planar_curv_weight)));
    
    // Take the minimum of all candidate (positive) scheduled speeds (Lowest allowed scheduled velocity is 0.5m/s, should be left this way)
    double VF = std::min({VF_EOP, VF_SOP, VF_XY});
    CLOG(DEBUG, "mpc.speed_scheduler") << "THE VF_EOP SPEED IS:  " << VF_EOP;
    CLOG(DEBUG, "mpc.speed_scheduler") << "THE VF_SOP SPEED IS:  " << VF_SOP;
    CLOG(DEBUG, "mpc.speed_scheduler") << "THE VF_XY SPEED IS:  " << VF_XY;

    // Take the minimum of all candidate scheduled speeds
    CLOG(DEBUG, "mpc.speed_scheduler") << "THE SPEED SCHEDULED SPEED IS:  " << VF;
    // End of speed scheduler code

    // Return the scheduled speed
    return VF;
  }

  
} // namespace vtr::path_planning


double ScheduleSpeed(const std::vector<double>& disc_path_curvature_xy, const std::vector<double>& disc_path_curvature_xz_yz, double VF, unsigned curr_sid, double planar_curv_weight, double profile_curv_weight, double eop_weight, unsigned horizon_steps, double min_vel) {

    // Experimental Speed Scheduler:
    // Takes in the desired forward_velocity and the pre-processed global path and reduces the set speed based on a range of tunable factors:
    // 1. Planar Curvature
    // 2. Profile Curvature
    // 3. End of Path

    // Pseudocode:
    // - Estimate the current p value of the vehicle (doesnt need to be super precise so here we can imply opt to use the sid value)
    // - Average the radius of curvature in the upcoming segments of the path

    // Basic implementation - weights hardcoded for now
    CLOG(DEBUG, "mpc.speed_scheduler") << "TRYING TO SCHEDULE SPEED:";
    CLOG(DEBUG, "mpc.speed_scheduler") << "CURRENT SID IS:" << curr_sid;

    double VF_EOP = std::max(min_vel, VF * (eop_weight * (disc_path_curvature_xy.size()- 1 - curr_sid) / 20.0));
    double VF_SOP = std::max(min_vel, VF * (eop_weight * curr_sid / 10));


    double VF_XY;
    double VF_XZ_YZ;
    double avg_curvature_xy = 0.0;
    double avg_curvature_xz_yz = 0.0;
    unsigned window_steps = 0;
    for (unsigned i = curr_sid; i < curr_sid + horizon_steps; i++) {
      // Handle end of path case
      if (i < disc_path_curvature_xy.size()-1) {
        avg_curvature_xy += disc_path_curvature_xy[i];
        avg_curvature_xz_yz += disc_path_curvature_xz_yz[i];
        ++window_steps;
      }
    }
    avg_curvature_xy /= window_steps;
    avg_curvature_xz_yz /= window_steps;
    CLOG(DEBUG, "mpc.speed_scheduler") << "THE AVERAGE PLANAR CURVATURE IS:  " << avg_curvature_xy;
    CLOG(DEBUG, "mpc.speed_scheduler") << "THE AVERAGE PROFILE CURVATURE IS:  " << avg_curvature_xz_yz;

    // handle forward/reverse case and calculate a candidate VF speed for each of our scheduler modules (XY curvature, XZ curvature, End of Path etc)

    VF_XY = std::max(min_vel, VF / (1 + (avg_curvature_xy * avg_curvature_xy * planar_curv_weight)));
    VF_XZ_YZ = std::max(min_vel, VF / (1 + (avg_curvature_xz_yz * avg_curvature_xz_yz * profile_curv_weight)));
    
    // Take the minimum of all candidate (positive) scheduled speeds (Lowest allowed scheduled velocity is 0.5m/s, should be left this way)
    VF = std::min({VF_EOP, VF_SOP, VF_XY, VF_XZ_YZ});
    CLOG(DEBUG, "mpc.speed_scheduler") << "THE VF_EOP SPEED IS:  " << VF_EOP;
    CLOG(DEBUG, "mpc.speed_scheduler") << "THE VF_SOP SPEED IS:  " << VF_SOP;
    CLOG(DEBUG, "mpc.speed_scheduler") << "THE VF_XY SPEED IS:  " << VF_XY;
    CLOG(DEBUG, "mpc.speed_scheduler") << "THE VF_XZ SPEED IS:  " << VF_XZ_YZ;

    // Take the minimum of all candidate scheduled speeds
    CLOG(DEBUG, "mpc.speed_scheduler") << "THE SPEED SCHEDULED SPEED IS:  " << VF;
    // End of speed scheduler code

    // Return the scheduled speed
    return VF;
}