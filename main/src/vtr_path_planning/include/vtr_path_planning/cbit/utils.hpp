#include <vector>
#include <memory>
#include <tuple>
#include <cmath>
#include <iostream>

#pragma once


// Some useful Classes
class Pose {
    public:
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
        
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
        double p;
        double q;
        double z; // experimental
        double g_T = NAN;
        double g_T_weighted = NAN;
        std::shared_ptr<Node> parent; // Shared pointer method (safer) // NOTE: Shared pointers initialize to null
        std::shared_ptr<Node> child; // Shared pointer method (safer)
        bool worm_hole = false;
        Node(double p_in, double q_in) // Node constructor
        : p{p_in}
        , q{q_in}
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
        std::vector<std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>  E;
        std::vector<std::shared_ptr<Node>> QV; // using shared pointers
        std::vector<std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>> QE;
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

//bool is_inside_obs(std::vector<std::vector<double>> obs, Node node);

//bool discrete_collision(std::vector<std::vector<double>> obs, double discretization, Node start, Node end);

double exp_radius(double q, double sample_box_height, double sample_box_width, double eta);

int bisection(std::vector<double> array, double value);

std::vector<double> linspace(double start_in, double end_in, int num_in);

//Pose lin_interpolate(int p_ind, double p_val, std::shared_ptr<Path> global_path);

//Node curve_to_euclid(Node node, std::shared_ptr<Path> global_path);

// signum function I got online
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//Function for checking if a tuple of nodes representing an edge is already in the queue/tree or not
// I think I might put this in the cbit code so it has easy access to view the tree
//bool edge_in_tree(std::tuple<Node, Node> edge);