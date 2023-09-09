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
 * \file cbit_config.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

// Initialize planner configs:

#pragma once 

class CBITConfig {
    public:
        CBITConfig() = default; 

        // Environment
        double obs_padding = 2.0;
        int curv_to_euclid_discretization = 20;
        double sliding_window_width = 15.0;
        double sliding_window_freespace_padding = 0.5;
        double corridor_resolution = 0.05;
        double state_update_freq = 1; // In Hz
        bool update_state = true;
        int rand_seed = 1;

        // ROC
        double roc_lookahead = 2.0;
        int roc_discretization = 40;
        double roc_q_tolerance = 0.001;

        // Planner Tuning Params
        int initial_samples = 250;
        int batch_samples = 100;
        double pre_seed_resolution = 0.5;
        double alpha = 0.5;
        double q_max = 2.5;
        int frame_interval = 50;
        int iter_max = 2000;
        double eta = 1.1;
        double rad_m_exhange = 1.00;
        double initial_exp_rad = 1.00;
        bool extrapolation = true;

        // Misc
        bool incremental_plotting = true;
        bool plotting = true;   

        // Speed Scheduler
        double planar_curv_weight = 2.50; // hardocded for now, make a param
        double profile_curv_weight = 0.5; // hardocded for now, make a param
        double eop_weight = 1.0; // hardocded for now, make a param

        // MPC Configs 
        int horizon_steps = 10;
        double horizon_step_size = 0.5;
        double forward_vel = 0.75;
        double max_lin_vel = 1.25;
        double max_ang_vel = 0.75;
        double robot_linear_velocity_scale = 1.0;
        double robot_angular_velocity_scale = 1.0;

        // Add unicycle model param

        // Covariance Tuning Weights
        //Eigen::Matrix<double, 6, 6> pose_error_cov = Eigen::Matrix<double, 6, 6>::Zero();
        //Eigen::Matrix<double, 2, 2> vel_error_cov = Eigen::Matrix<double, 2, 2>::Zero();
        //Eigen::Matrix<double, 2, 2> acc_error_cov = Eigen::Matrix<double, 2, 2>::Zero();
        //Eigen::Matrix<double, 6, 6> kin_error_cov = Eigen::Matrix<double, 6, 6>::Zero();

        int command_history_length = 100;

        // COSTMAP PARAMS;
        double costmap_filter_value = 0.01;
        int costmap_history = 5;

};