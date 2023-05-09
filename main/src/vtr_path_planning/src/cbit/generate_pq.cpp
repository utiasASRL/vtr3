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
 * \file generate_pq.cpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

// Code for pre-processing the taught path into its curvilinear representation. Need to update this file to include spline smoothing of the path
// In order to generate a smooth instantaneous radious of curvature plot for defining the singularity regions

#include "vtr_path_planning/cbit/generate_pq.hpp"

// Path Constructor:
CBITPath::CBITPath(CBITConfig config, std::vector<Pose> initial_path)
{   
    double alpha = config.alpha;

    // Process the path
    disc_path = initial_path;

    // TODO: Cubic spline the discrete path to make it have smooth derivatives
    // TEMP DEBUG, set the path equal to discrete path (no interpolations yet)
    path = disc_path;

    // Iterate through all poses in disc_path, assign a p coordinate value to the pose to use for the curvilinear space reference
    int vect_size = disc_path.size();
    p.reserve(vect_size);
    p.push_back(0);
    sid.reserve(vect_size);
    sid.push_back(0);
    for (int i=1; i<vect_size; i++)
    {
        p.push_back(p[i-1] + delta_p_calc(disc_path[i-1], disc_path[i], alpha));
        sid.push_back(i);
    }

    CLOG(INFO, "path_planning.cbit") << "Successfully Built a Path in generate_pq.cpp and Displayed log";
    
}

// note made this pretty quick and dirty, need to refine with interpolation next
Pose CBITPath::interp_pose(double p_in)
{
    int p_iter = 0;
    while (p_in > this->p[p_iter])
    {
        p_iter = p_iter+1;
    }
    Pose interpolated_pose(this->disc_path[p_iter].x, this->disc_path[p_iter].y, this->disc_path[p_iter].z, this->disc_path[p_iter].roll, this->disc_path[p_iter].pitch, this->disc_path[p_iter].yaw);
    return interpolated_pose;
}

// Calculating the distance between se(3) poses including a heading contribution
double CBITPath::delta_p_calc(Pose start_pose, Pose end_pose, double alpha)
{
    double dx = start_pose.x - end_pose.x;
    double dy = start_pose.y - end_pose.y;
    double dz = start_pose.z - end_pose.z;

    // For the angular contribution, we need to be very careful about angle wrap around, cant simply just take the difference
    double dyaw = end_pose.yaw - start_pose.yaw;

    double yaw_arr[3] = {std::abs(dyaw), std::abs(dyaw + (2 * M_PI)), std::abs(dyaw - (2 * M_PI))};
    
    double abs_angle = 2.0 * M_PI;
    for (int i = 0; i < 3; i++)
    {
        if (yaw_arr[i] < abs_angle)
        {
            abs_angle = yaw_arr[i];
        }
    }

    return sqrt((dx * dx) + (dy * dy) + (dz * dz) + (alpha * abs_angle * abs_angle));
}

void CBITPath::spline_curvature()
{
    // TODO
    auto test = 1;
    
}


// Corridor Path constructor:
CBITCorridor::CBITCorridor(CBITConfig config, std::shared_ptr<CBITPath> global_path_ptr)
{
    q_max = config.q_max;
    sliding_window_width = config.sliding_window_width + config.sliding_window_freespace_padding;
    curv_to_euclid_discretization = config.curv_to_euclid_discretization;
    double length_p = global_path_ptr->p.back();
    int num_bins = ceil(length_p / config.corridor_resolution);
    // Initialize vector lengths
    p_bins.reserve(num_bins);
    q_left.reserve(num_bins);
    q_right.reserve(num_bins);
    x_left.reserve(num_bins);
    x_right.reserve(num_bins);
    y_left.reserve(num_bins);
    y_right.reserve(num_bins);

    // Initialize bins
    p_bins = linspace(0, length_p, num_bins);
    for (int i = 0; i < num_bins; i++)
    {
        q_left.push_back(config.q_max);
        q_right.push_back(-1.0 * config.q_max);
    }

    // I think its wise if here we also initialize the euclidean corridor points as well. This is annoying though because out curv to euclid
    // is not in utils (yet). TODO I really should move all the collision checking things external and independant from the planner for this reason

    // For now I may instead brute force the euclid generate at the end of each corridor update, I feel like even a 50000 bin loop (2.5km) wouldnt take
    // more then a few ms (I believe early experiments showed you could do like 4million loop iterations every second or so in c++)

    // debug prints to make sure this happened correctly
    CLOG(DEBUG, "path_planning.corridor_debug") << "Length of P is: " << length_p;
    CLOG(DEBUG, "path_planning.corridor_debug") << "Number of Bins is: " << num_bins;
    CLOG(DEBUG, "path_planning.corridor_debug") << "P_bins are: " << p_bins;
    CLOG(DEBUG, "path_planning.corridor_debug") << "Q_left is: " << q_left;
    CLOG(DEBUG, "path_planning.corridor_debug") << "Q_right is: " << q_right;
    CLOG(DEBUG, "path_planning.corridor_debug") << "Size of p_bins: " << p_bins.size();
    CLOG(DEBUG, "path_planning.corridor_debug") << "Size of q_left,q_right: " << q_left.size();
    
}