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
 * \file utils.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

// Contains some useful helper functions and classes for the cbit planner

#include <vector>
#include <memory>
#include <tuple>
#include <cmath>
#include <iostream>
#include <map>

#include "vtr_tactic/types.hpp"

#pragma once


namespace vtr::path_planning {

    struct ChainInfo {
        tactic::Timestamp stamp;
        Eigen::Matrix<double, 6, 1> w_p_r_in_r;
        // planning frame is the current localization frame
        tactic::EdgeTransform T_p_r;  // T_planning_robot
        tactic::EdgeTransform T_w_p;  // T_world_planning
        tactic::EdgeTransform T_w_v_odo;  // T_planning_robot
        tactic::EdgeTransform T_r_v_odo;  // T_world_planning
        unsigned curr_sid;
    };

    /** \brief Retrieve information for planning from localization chain */
    ChainInfo getChainInfo(const tactic::LocalizationChain& robot_state);
} // namespace vtr::path_planning




// Some useful Classes
class Pose {
    public:
        using Ptr = std::shared_ptr<Pose>;
        using Path = std::vector<Pose>;

        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
        double p;
        double q;
        
        Pose* parent = nullptr; // Use a pointer to a Pose class to keep track of a parent
        Pose(double x_in, double y_in, double z_in, double roll_in, double pitch_in, double yaw_in) // Pose constructor
            : x{x_in}
            , y{y_in}
            , z{z_in}
            , roll{roll_in}
            , pitch{pitch_in}
            , yaw{yaw_in}
            {
            }

        Pose() = default;
        // I think we may need to make another function for setting the parent pointer?
};


class Node {
    public:
        using Ptr = std::shared_ptr<Node>;
        using Path = std::vector<Ptr>;
        using Edge = std::tuple<Ptr, Ptr>;

        double p;
        double q;
        double z; // experimental
        double g_T = NAN;
        double g_T_weighted = NAN;
        Node::Ptr parent; // Shared pointer method (safer) // NOTE: Shared pointers initialize to null
        Node::Ptr child; // Shared pointer method (safer)
        bool worm_hole = false;
        Node(double p_in, double q_in) // Node constructor
        : p{p_in}
        , q{q_in}
        , z{0}
        {
        }

        // Experimental, secondary constuctor for Nodes with a z component
        // This z component is only going to be used in the output of curve to euclid for both collision checks and for publishing the final 3D path
        Node(double p_in, double q_in, double z_in) // Node constructor
        : p{p_in}
        , q{q_in}
        , z{z_in}
        {
        }
        Node() = default;
        // I think we may need to make another function for setting the parent/child pointer?
};


// Class for storing the tree in unordered sets
class Tree {
    public:
        std::vector<std::shared_ptr<Node>> V;
        std::vector<std::shared_ptr<Node>> V_Old;
        std::vector<std::shared_ptr<Node>> V_Repair_Backup;
        std::vector<std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>  E;
        std::vector<std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>  E_Old;
        std::vector<std::shared_ptr<Node>> QV; // using shared pointers
        std::multimap<double, std::shared_ptr<Node>> QV2;
        std::vector<std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>> QE;
        std::multimap<double, std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>> QE2;
        Tree() = default;
};

// Some repetitive calculation functions

double h_estimated(Node node, Node goal, double alpha);

double h_estimated_admissible(Node node, Node goal);

double g_estimated(Node node, Node start, double alpha);

double g_estimated_admissible(Node node, Node start);

double f_estimated(Node node, Node start, Node goal, double alpha);

double calc_weighted_dist(Node start, Node end, double alpha);

double calc_dist(Node start_node, Node end_node);

double exp_radius(double q, double sample_box_height, double sample_box_width, double eta);

int bisection(std::vector<double> array, double value);

std::vector<double> linspace(double start_in, double end_in, int num_in);

// signum function I got online
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
