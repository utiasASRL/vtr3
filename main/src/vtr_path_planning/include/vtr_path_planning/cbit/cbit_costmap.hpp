// This header will define a temporal costmap class which will be updated by the current ros occupancy grid in the vtr_lidar package.

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