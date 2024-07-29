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
 * \file cbit_path_planner.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

#include <vector>
#include <memory>
#include <cstdio>
#include <iostream>
#include <cmath>
#include <tuple>
#include <algorithm>
#include <random>
#include <chrono> // For benchmarking
#include <unistd.h> // For debug sleeps

#include "vtr_logging/logging.hpp"
#include "vtr_path_planning/base_path_planner.hpp"
#include "vtr_path_planning/cbit/utils.hpp"
#include "vtr_path_planning/cbit/generate_pq.hpp"
#include "vtr_path_planning/cbit/cbit_config.hpp"
#include "vtr_path_planning/cbit/cbit_costmap.hpp"
#include "vtr_path_planning/cbit/cbit_plotting.hpp"
#include "vtr_tactic/tactic.hpp"

#pragma once

// Note long term, this class should probably be inherited by the base path planner

using namespace vtr;


class CBITPlanner {
    public:
        CBITConfig conf;
        std::shared_ptr<CBITPath> global_path;
        Node::Ptr p_start;
        Node::Ptr p_goal;
        std::vector<double> path_x;
        std::vector<double> path_y;
        double sample_box_height;
        double sample_box_width;
        double dynamic_window_width;
        Tree tree;
        Node::Path samples;

        // Repair mode variables
        bool repair_mode = false; // Flag for whether or not we should resume the planner in repair mode to update the tree following a state update
        Node::Ptr repair_vertex;
        double repair_g_T_old;
        double repair_g_T_weighted_old;

        Node::Ptr p_goal_backup;


        // For storing the most up-to-date euclidean robot pose
        std::unique_ptr<Pose> new_state;

        // For storing the Output Path
        std::shared_ptr<Pose::Path> cbit_path_ptr;

        // Flag that tells the cbit.cpp planning interface to stop the mpc if there is no current cbit solution
        std::shared_ptr<bool> valid_solution_ptr;

        // Pointer to the dynamic corridor width from planner GUI
        std::shared_ptr<double> q_max_ptr;
        
        // Temporary obstacles
        std::vector<std::vector<double>>  obs_rectangle;

        // Costmap pointer
        std::shared_ptr<CBITCostmap> cbit_costmap_ptr;

        CBITPlanner(CBITConfig conf_in, std::shared_ptr<CBITPath> path_in, vtr::path_planning::BasePathPlanner::RobotState& robot_state, std::shared_ptr<std::vector<Pose>> path_ptr, std::shared_ptr<CBITCostmap> costmap_ptr, std::shared_ptr<CBITCorridor> corridor_ptr, std::shared_ptr<bool>solution_ptr, std::shared_ptr<double>width_ptr);

        void plan();
        void stopPlanning();
        void resetPlanner();
    protected:
    struct ChainInfo {
        vtr::tactic::Timestamp stamp;
        Eigen::Matrix<double, 6, 1> w_p_r_in_r;
        // planning frame is the current localization frame
        vtr::tactic::EdgeTransform T_p_r;  // T_planning_robot
        vtr::tactic::EdgeTransform T_w_p;  // T_world_planning
        unsigned curr_sid;
    };
    private:
        void InitializePlanningSpace();
        std::shared_ptr<Node> UpdateStateSID(size_t SID, vtr::tactic::EdgeTransform T_p_r);
        std::vector<std::shared_ptr<Node>> SampleBox(int m);
        std::vector<std::shared_ptr<Node>> SampleFreeSpace(int m);
        double BestVertexQueueValue();
        double BestEdgeQueueValue();
        vtr::path_planning::BasePathPlanner::RobotState& robot_state_;
        std::shared_ptr<Node> BestInVertexQueue();
        void ExpandVertex(std::shared_ptr<Node> v);
        std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> BestInEdgeQueue();
        std::tuple<std::vector<double>, std::vector<double>> ExtractPath(vtr::path_planning::BasePathPlanner::RobotState& robot_state, std::shared_ptr<CBITCostmap> costmap_ptr);
        std::vector<Pose> ExtractEuclidPath();
        void Prune(double c_best, double c_best_weighted);
        bool edge_in_tree_v2(std::shared_ptr<Node> v, std::shared_ptr<Node> x);
        bool node_in_tree_v2(std::shared_ptr<Node> x);
        double cost_col(std::vector<std::vector<double>> obs, Node start, Node end);
        double weighted_cost_col(std::vector<std::vector<double>> obs, Node start, Node end);
        Node curve_to_euclid(Node node);
        Pose lin_interpolate(int p_ind, double p_val);
        bool costmap_col(Node node);
        bool costmap_col_tight(Node node);
        bool discrete_collision(std::vector<std::vector<double>> obs, double discretization, Node start, Node end);
        std::shared_ptr<Node> col_check_path_v2(double max_lookahead_p);

        mutable bool planning_active_ = false;
};