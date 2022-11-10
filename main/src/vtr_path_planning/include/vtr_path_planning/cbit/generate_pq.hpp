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
 * \file generate_pq.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

// Header for generating the curvilinear pq space by pre-processing the taught path

#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstdio>

#include "vtr_logging/logging.hpp"
#include "vtr_path_planning/cbit/utils.hpp"
#include "vtr_path_planning/cbit/cbit_config.hpp"

#pragma once 

// TODO: ROC singularity region handling

class CBITPath {
    public:
        std::vector<Pose> disc_path; // Stores the se3 discrete path as a vector of Euclidean vectors (The original disc path) // TODO, change to se(3) class
        std::vector<double> p; //associated p values for each pose in disc_path
        std::vector<Pose> path; // Stores the se3 splined discrete path as a vector of Euclidean vectors; //TODO, change to se(3) class
        CBITPath(CBITConfig config, std::vector<Pose> initial_path); // constructor; need to feed this the path 
        CBITPath() = default;

        double delta_p_calc(Pose start_pose, Pose end_pose, double alpha); // Function for computing delta p intervals in p,q space

    // Internal function declarations
    private:
        void spline_curvature();
       
        // Actually I think ill just do this in the constructor for now
        //std::vector<double> ProcessPath(std::vector<Pose> disc_path); // Function for assigning p distance values for each euclidean point in pre-processing
};