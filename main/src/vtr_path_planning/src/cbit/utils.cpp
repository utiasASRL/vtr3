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
 * \file utils.cpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

// Library holding many useful object classes used by the planner as well as some of the
// functions for common computations

#include "vtr_path_planning/cbit/utils.hpp"

// Misc functions:

// Weighted cost to go heuristic
double h_estimated(Node node, Node goal, double alpha)
{

    // instead of using straight line euclidean distance like in normal BIT*,
    // we use a more conservative heuristic which is simply the longitudinal distance to the goal
    //Node projected_node(node.p, 0);
    //double horz_cost = calc_dist(projected_node, goal);
    return calc_weighted_dist(node, goal, alpha);
}

double h_estimated_admissible(Node node, Node goal)
{
    // instead of using straight line euclidean distance like in normal BIT*,
    // we use a more conservative heuristic which is simply the longitudinal distance to the goal
    double horz_cost = calc_dist(Node(node.p, 0), Node(goal.p, 0));
    return horz_cost;
}

// weighted cost to come heuristic
double g_estimated(Node node, Node start, double alpha)
{
    // instead of using straight line euclidean distance like in normal BIT*,
    // we use a more conservative heuristic which is simply the longitudinal distance to the goal
    return calc_weighted_dist(start, node, alpha);
}

// Admissible cost to come heuristic
double g_estimated_admissible(Node node, Node start)
{
    // instead of using straight line euclidean distance like in normal BIT*,
    // we use a more conservative heuristic which is simply the longitudinal distance to the goal
    double horz_cost = calc_dist(start, Node(node.p, 0));
    return horz_cost;
}



// Admissible cost to go + cost to come heuristic for pruning
double f_estimated(Node node, Node start, Node goal, double alpha)
{
    double horz_cost = g_estimated(node, start, alpha) + h_estimated(node, goal, alpha);
    return horz_cost;
}

double calc_weighted_dist(Node start_node, Node end_node, double alpha)
{
    double dist_weight;

    // Handle divide by 0 case:
    if (start_node.q == end_node.q)
    {
        dist_weight = 1.0 + (alpha * end_node.q * end_node.q);
    }
    else
    {
        dist_weight = (1.0 + (alpha / 3.0) * (((end_node.q * end_node.q * end_node.q) - (start_node.q * start_node.q * start_node.q)) / (end_node.q - start_node.q)));
    }
    return dist_weight * calc_dist(start_node,end_node);
}

double calc_dist(Node start_node, Node end_node)
{
    double dp = start_node.p - end_node.p;
    double dq = start_node.q - end_node.q;

    return sqrt((dp * dp) + (dq * dq));
}


// Function for calculating the expansion radius based on the sample density
double exp_radius(double q, double sample_box_height, double sample_box_width, double eta)
{   /*
    # New radius expansion calc for sliding window sampling region
    # Note, we might need to account for the fact that we have several samples outside of the sampling region due to pre-seeds
    # which is going to inflate the value of q slightly higher than normal, might need to chop the tree at the edge of the sliding window when we call this function
    # q is the number of samples + vertices
    # d is the dimensions of the space
    # eta is a tuning param, usually = 1
    # Lambda_X is the Lebesgue measure of the obstacle free space (just area in R2)
    # Zeta is the Lebesgue measure of a unit-d ball (constant! For R2, just the area of the unit circle = pi*(1^2))
    */

    double d = 2;
    double lambda_x = sample_box_height * sample_box_width;
    double zeta = M_PI;
    double radius = 2.0 * eta * (pow((1.0 + (1.0/d)),(1.0/d))) * (pow((lambda_x/zeta),0.5)) * (pow((log(q) / q),(1.0/d)));
    return radius;
}


// Function for quickly finding the index j for the value in a sorted vector which is immediately below the function value.
// in curve_to_euclid we use this function to efficiently find the index of the euclidean se(3) pose stored in the discrete path which
// immediately preceders the current p,q point we are trying to convert back to euclidean.
int bisection(std::vector<double> array, double value)
{
    int n = array.size();
    if (value < array[0])
    {
        return -1;
    }
    if (value > array[(n-1)])
    {
        return n;
    }

    int jl = 0; // Initialize lower
    int ju = n-1; // Initialize upper
    int jm;
    while ((ju - jl) > 1)
    {
        jm = (ju + jl) >> 1; // Compute midpoint using bitshit
        if (value >= array[jm])
        {
            jl = jm; // Replace either lower limit
        }
        else
        {
            ju = jm; // Or upper limit, as appropriate
        }
    }

    if (value == array[0]) // Edge case at bottom of vector
    {
        return 0;
    }   
    else if (value == array[(n-1)]) // Edge case at top of the vector
    {
        return (n-1);
    }
    else
    {
        return jl;
    }
}


// Implementation of Pythons linspace function
std::vector<double> linspace(double start_in, double end_in, int num_in)
{

  std::vector<double> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) 
    {
      linspaced.push_back(start);
      return linspaced;
    }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
  linspaced.push_back(end); // I want to ensure that start and end
                            // are exactly the same as the input
  return linspaced;
}