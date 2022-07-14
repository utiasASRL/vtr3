#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstdio>

//#include "vtr_path_planning/base_path_planner.hpp"
#include "vtr_path_planning/cbit/utils.hpp"
#include "vtr_path_planning/cbit/cbit_config.hpp"

#pragma once 

// For now not worrying about any of the ROC stuff

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