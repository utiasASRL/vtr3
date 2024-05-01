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

    path = disc_path;

    // Iterate through all poses in disc_path, assign a p coordinate value to the pose to use for the curvilinear space reference
    unsigned vect_size = disc_path.size();
    p.reserve(vect_size);
    p.push_back(0.0);
    sid.reserve(vect_size);
    sid.push_back(0);
    for (unsigned i=1; i < vect_size; i++)
    {
        p.push_back(p[i-1] + delta_p_calc(disc_path[i-1], disc_path[i], alpha));
        sid.push_back(i);
    }

    if (vect_size < 2) {
        throw std::runtime_error("Path of length 1 cannot be interpolated!");
    }

    // Generate XY, XZ Curvature Estimates

    // 2D spline interpolation using Cubic B-Spline
    Eigen::Spline<double, 2> spline_xy = spline_path_xy(disc_path);

    Eigen::Spline<double, 2> spline_xz_yz = spline_path_xz_yz(disc_path);

    // Calculate curvature along the teach path at each vertex
    for (size_t i=0; i < p.size(); i++)
    {
        disc_path_curvature_xy.push_back(1.0 / radius_of_curvature(p[i]/p.back(), spline_xy)); // the input chord length should be normalized to [0,1] along the length of the path
        disc_path_curvature_xz_yz.push_back(1.0 / radius_of_curvature(p[i]/p.back(), spline_xz_yz)); // the input chord length should be normalized to [0,1] along the length of the path
    }


    CLOG(INFO, "cbit_planner.path_planning") << "Successfully Built a Path in generate_pq.cpp";
    
}

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

Eigen::Spline<double, 2> CBITPath::spline_path_xy(const std::vector<Pose> &input_path)
{
    std::vector<Eigen::Vector2d> valid_points;
    int spacing = (input_path.size() < 20) ? input_path.size() / 4 : 5;
    if (spacing == 0)
        spacing = 1;

    // Usually use every fifth point. 
    // If the input path is shorter than 20 choose a spacing between 1 and 5

    
    for (size_t i = 0; i < input_path.size(); i++) {
        if (i % spacing == 0) {
            valid_points.push_back(Eigen::Vector2d(input_path[i].x, input_path[i].y));
        }
    }

    Eigen::MatrixXd points(2, valid_points.size());

    for (size_t i = 0; i < valid_points.size(); i++) 
    {
        points(0, i) = valid_points[i](0);
        points(1, i) = valid_points[i](1);
    }


    Eigen::Spline<double, 2> spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(points, 2);
    // To query the spline, we use spline(test), where test is a normalized distance along the spline from 0 to 1
    //double test_p = p[1] / p.back();
    //double pred_x = spline(test_p)[0];
    //double pred_y = spline(test_p)[1];

    return spline;
}

Eigen::Spline<double, 2> CBITPath::spline_path_xz_yz(const std::vector<Pose> &input_path) {
    std::vector<Eigen::Vector2d> valid_points;

    // Usually use every fifth point. 
    // If the input path is shorter than 20 choose a spacing between 1 and 5
    int spacing = (input_path.size() < 20) ? input_path.size() / 4 : 5;
    if (spacing == 0)
        spacing = 1;

    for (size_t i = 0; i < input_path.size(); i++) {
        if (i % spacing == 0) {
            valid_points.push_back(Eigen::Vector2d(sqrt((input_path[i].x * input_path[i].x) + (input_path[i].y * input_path[i].y)), input_path[i].z));
        }
    }

    Eigen::MatrixXd points(2, valid_points.size());

    for (size_t i = 0; i < valid_points.size(); i++) 
    {
        points(0, i) = valid_points[i](0);
        points(1, i) = valid_points[i](1);
    }

    Eigen::Spline<double, 2> spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(points, 2);
    // To query the spline, we use spline(test), where test is a normalized distance along the spline from 0 to 1
    //double test_p = p[1] / p.back();
    //double pred_x = spline(test_p)[0];
    //double pred_y = spline(test_p)[1];

    return spline;
}


double CBITPath::radius_of_curvature(double dist, Eigen::Spline<double, 2> spline) 
{
    // The spline object cant generate state vector interpolations given an input p distance (chord length) using spline(p)
    // Note p must be normalized between [0,1] on the total length of the path
    double pred_x = spline(dist)[0];
    double pred_y = spline(dist)[1];
    // derivatives functions returns a matrix of size (spline dimension, min(spline_order, derivative))
    // Where the rows correspond to each spline dimension and the columns the 0th, 1st, 2nd... derivatives
    auto curvature = spline.derivatives(dist, 2); // returns a vector of size (spline dimension, min(spline_order, derivative))

    

    // Calculating radius of curvature from the derivatives
    double dx_dt = curvature(0,1);
    double dy_dt = curvature(1,1);

    double d2x_dt2 = curvature(0,2);
    double d2y_dt2 = curvature(1,2);

    // If using only the magnitude of radius of curvature:
    double roc_magnitude = std::pow(std::pow(dx_dt, 2) + std::pow(dy_dt, 2), 1.5) / std::abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2);


    // TODO, return both signed and magnitude ROC's
    // We use the magnitude ROC's for speed scheduling and the signed ones for generating wormhole regions
    /**
    // Calculating the sign of the radius of curvature, positive means curving left, negative means curving right
    double cross_prod = dx_dt * d2y_dt2 - dy_dt * d2x_dt2; // compute the z-component of the cross product
    int sign = (cross_prod > 0) ? 1 : -1; // determine the sign of the radius of curvature
    double roc_signed = sign * std::pow(std::pow(dx_dt, 2) + std::pow(dy_dt, 2), 1.5) / std::abs(cross_prod);

    double x_pred = curvature(0,0);
    double y_pred = curvature(1,0);
    
    */
    return roc_magnitude;
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

}