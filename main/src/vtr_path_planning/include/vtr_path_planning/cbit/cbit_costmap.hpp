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
 * \file cbit_costmap.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

// This header defines a temporal costmap class which will be updated by the current ros occupancy grid in the vtr_lidar package.

#include <unordered_map>
#include <memory>

#include "vtr_tactic/tactic.hpp"
#include "vtr_common/utils/hash.hpp"  // for std::pair hash

#pragma once

class CBITCostmap {
    public:
        // The actual costmap is an unordered map of (x,y) coordinate keys corresponding to the occupancy grid value of that cell
        //std::unique_ptr<std::unordered_map<std::pair<float, float>, float>> obs_map_ptr;
        // Costmap in the costmap frame
        std::unordered_map<std::pair<float, float>, float> obs_map;

        // For storing a history of the costmaps for temporal filtering
        std::vector<std::unordered_map<std::pair<float, float>, float>> obs_map_vect;




        //std::unique_ptr<vtr::tactic::EdgeTransform> T_r_costmap_ptr;
        // We also want to store a pointer to the current transform from the robot to the costmap
        vtr::tactic::EdgeTransform T_c_w;

        // For storing a history of the transforms for temporal filtering
        std::vector<vtr::tactic::EdgeTransform> T_c_w_vect;



        // Todo, I think we will also need to save the grid resolution here too, which I think is accessed in the change_detection_costmap
        float grid_resolution;

        // todo, may add a constructor and associated cpp file which does some more sophisticated temporal processing of the occupancy grid
};