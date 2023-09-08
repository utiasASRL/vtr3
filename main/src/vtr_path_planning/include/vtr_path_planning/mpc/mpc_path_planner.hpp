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
 * \file mpc_path_planner.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/cbit/cbit.hpp"
#include "steam.hpp"

#pragma once

struct MPCConfig
{
    Eigen::Matrix<double, 2, 1> previous_vel;
    lgmath::se3::Transformation T0;
    std::vector<lgmath::se3::Transformation> tracking_reference_poses;
    std::vector<lgmath::se3::Transformation> homotopy_reference_poses;
    std::vector<double> barrier_q_left;
    std::vector<double> barrier_q_right;
    int K;
    double DT;
    double VF;
    Eigen::Matrix<double, 1, 1> lat_noise_vect;
    Eigen::Matrix<double, 6, 6> pose_noise_vect;
    Eigen::Matrix<double, 2, 2> vel_noise_vect;
    Eigen::Matrix<double, 2, 2> accel_noise_vect;
    Eigen::Matrix<double, 6, 6> kin_noise_vect;
    bool point_stabilization;
    double pose_error_weight;
    double vel_error_weight;
    double acc_error_weight;
    double kin_error_weight;
    double lat_error_weight;
    bool verbosity;
    bool homotopy_mode;
};

struct MPCResult
{
    Eigen::Matrix<double, 2, 1> applied_vel;
    std::vector<lgmath::se3::Transformation> mpc_poses;
};

struct PoseResultTracking
{
    std::vector<lgmath::se3::Transformation> poses;
    bool point_stabilization;
    std::vector<double> p_interp_vec;
    std::vector<double> q_interp_vec;
};

struct PoseResultHomotopy
{
    std::vector<lgmath::se3::Transformation> poses;
    bool point_stabilization;
    std::vector<double> barrier_q_left;
    std::vector<double> barrier_q_right;
};

struct InterpResult
{
    lgmath::se3::Transformation pose;
    double p_interp;
    double q_interp;
};


// Declaring helper functions

// Primary optimization function: Takes in the input configurations and the extrapolated robot pose, outputs a vector for the velocity to apply and the predicted horizon
struct MPCResult SolveMPC(const MPCConfig& config);

// Helper function for generating reference measurements poses from a discrete path to use for tracking the path at a desired forward velocity
struct PoseResultTracking GenerateTrackingReference(std::shared_ptr<std::vector<Pose>> cbit_path_ptr,  std::tuple<double, double, double, double, double, double> robot_pose, int K, double DT, double VF);

// Helper function for generating reference measurements poses from a discrete path to use for tracking the path at a desired forward velocity
struct PoseResultHomotopy GenerateHomotopyReference(std::shared_ptr<CBITPath> global_path_ptr, std::shared_ptr<CBITCorridor> corridor_ptr, std::tuple<double, double, double, double, double, double> robot_pose, int K, double DT, double VF, int current_sid, std::vector<double> p_interp_vec);

// Helper function for post-processing and saturating the velocity command
Eigen::Matrix<double, 2, 1> SaturateVel(Eigen::Matrix<double, 2, 1> applied_vel, double v_lim, double w_lim);

// Helper function in Generate Reference Meas which interpolates a Transformation measurement gen the cbit_path and the desired measurements p value along the path
struct InterpResult InterpolatePose(double p_meas, std::vector<double> cbit_p, std::vector<Pose> cbit_path);