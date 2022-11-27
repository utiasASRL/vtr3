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

// Pose Constructor (ended up initializing these in the header)
/*
Pose::Pose(float x_in, float y_in, float z_in, float roll_in, float pitch_in, float yaw_in)
{
    x = x_in;
    y = y_in;
    z = z_in;
    roll = roll_in;
    pitch = pitch_in;
    yaw = yaw_in;
}
*/

// P,Q Node Constructor
/*
Node::Node(float p_in, float q_in)
{
    p = p_in;
    q = q_in;
}
*/

// Misc functions:

// Weighted cost to go heuristic
double h_estimated(Node node, Node goal, double alpha)
{
    // TODO: Need to wormhole proof this function still

    // instead of using straight line euclidean distance like in normal BIT*,
    // we use a more conservative heuristic which is simply the longitudinal distance to the goal
    //Node projected_node(node.p, 0);
    //double horz_cost = calc_dist(projected_node, goal);
    return calc_weighted_dist(node, goal, alpha);
}

double h_estimated_admissible(Node node, Node goal)
{
    // TODO: Need to wormhole proof this function still

    // instead of using straight line euclidean distance like in normal BIT*,
    // we use a more conservative heuristic which is simply the longitudinal distance to the goal
    //Node projected_node(node.p, 0);
    double horz_cost = calc_dist(Node(node.p, 0), Node(goal.p, 0));
    return horz_cost;
}

// weighted cost to come heuristic
double g_estimated(Node node, Node start, double alpha)
{
    // TODO: Need to wormhole proof this function still

    // instead of using straight line euclidean distance like in normal BIT*,
    // we use a more conservative heuristic which is simply the longitudinal distance to the goal
    //Node projected_node(node.p, 0);
    //double horz_cost = calc_dist(start, projected_node);
    return calc_weighted_dist(start, node, alpha);
}

// Admissible cost to come heuristic
double g_estimated_admissible(Node node, Node start)
{
    // TODO: Need to wormhole proof this function still

    // instead of using straight line euclidean distance like in normal BIT*,
    // we use a more conservative heuristic which is simply the longitudinal distance to the goal
    //Node projected_node(node.p, 0);
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
    if (dist_weight * calc_dist(start_node,end_node) < 0.0 )
    {
        std::cout <<"Something wrong here" <<std::endl;
    }
    return dist_weight * calc_dist(start_node,end_node);
}

double calc_dist(Node start_node, Node end_node)
{
    double dp = start_node.p - end_node.p;
    double dq = start_node.q - end_node.q;

    return sqrt((dp * dp) + (dq * dq));
}


// No longer doing collision checks this way, query the obstacle costmap directly
/*
bool is_inside_obs(std::vector<std::vector<double>> obs, Node node)
{
    for (int i = 0; i < obs.size(); i++)
    {
        double x = obs[i][0];
        double y = obs[i][1];
        double w = obs[i][2];
        double h = obs[i][3];
        //bool test1 = (0 <= (node.p - x) <= w);
        //bool test2 = ((0 <= (node.p -x)) && ((node.p -x) <= w) && (0 <= (node.p -x)) && ((node.p -x) <= h));
        //bool test3 = ((0 <= (node.p -x)) && ((node.p -x) <= w));
        //bool test4 = ((0 <= (node.p -x)) && ((node.p -x) <= h));
        if ((0 <= (node.p - x)) && ((node.p - x) <= w) && (0 <= (node.q - y)) && ((node.q - y) <= h))
        {
            return true;
        }
    }
    return false;
}
*/

// Collision checks an edge from start to end nodes
// TODO Needs to add in wormhole and curve to euclid conversions
/*
bool discrete_collision(std::vector<std::vector<double>> obs, double discretization, Node start, Node end)
{
    // We dynamically determine the discretization based on the length of the edge
    discretization = round(calc_dist(start, end) * discretization);

    // Generate discretized test nodes
    std::vector<double> p_test;
    std::vector<double> q_test;

    double p_step = fabs(end.p - start.p) / discretization;
    double q_step = fabs(end.q - start.q) / discretization;
    
    p_test.push_back(start.p);
    q_test.push_back(start.q);

    for (int i = 0; i < discretization-1; i++)
    {
        p_test.push_back(p_test[i] + p_step*sgn(end.p-start.p) );
        q_test.push_back(q_test[i] + q_step*sgn(end.q-start.q) );
    }
    p_test.push_back(end.p);
    q_test.push_back(end.q);



    // Loop through the test curvilinear points, convert to euclid, collision check obstacles
    for (int i = 0; i < p_test.size(); i++)
    {
        Node curv_pt = Node(p_test[i], q_test[i]);

        // Convert to euclid TODO:
        Node euclid_pt = curv_pt; // DEBUG DO NOT LEAVE THIS HERE, NEED TO REPLACE WITH COLLISION CHECK FUNCTION
        if (is_inside_obs(obs, euclid_pt))
        {
            return true;
        }
    }

    return false;
}
*/


// Function for calculating the expansion radius based on the sample density
// TODO: This needs to be updated for my sliding window radius
// - I may code it with sliding radius in mind for now, but will probably need some changes when I get there.
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
    //std::cout << "Expansion Radius: " << radius << std::endl;
    return radius;
}

// Moving this to cbit.cpp so I dont need to pass the whole giant path vector every time I call
/*
// Function for converting a p,q coordinate value into a euclidean coordinate using the pre-processed path to follow
Node curve_to_euclid(Node node, std::vector<)
{
    double p_val = node.p;
    double q_val = node.q;
    int p_ind =  bisection()
}
*/

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


// Moved the following functions to cbit_path_planner.cpp
/*
Pose lin_interpolate(int p_ind, double p_val, std::shared_ptr<Path> global_path)
{
  double p_max = global_path->p[(global_path->p.size() - 1)]; //TODO: Replace this with se(3)
  double p_lower;
  double p_upper;
  if (p_val >= p_max) // if p_val is exactly the max (goal p) then return the final euclid pose
  {
    return Pose(global_path->path[(global_path->path.size() - 1)]);
  }

  else
  {
    p_upper = global_path->p[p_ind + 1];
    p_lower = global_path->p[p_ind];
  }

  Pose start_pose = global_path->path[p_ind];
  Pose end_pose = global_path->path[p_ind + 1];

  double x_c = start_pose.x + ((p_val - p_lower) / (p_upper - p_lower)) * (end_pose.x - start_pose.x);
  double y_c = start_pose.y + ((p_val - p_lower) / (p_upper - p_lower)) * (end_pose.y - start_pose.y);
  double z_c = start_pose.z + ((p_val - p_lower) / (p_upper - p_lower)) * (end_pose.z - start_pose.z);

  // For angles, we dont really care about roll and pitch, these can be left 0 (atleast for now)
  // For yaw need to be very careful of angle wrap around problem:
  double angle_difference = std::fmod((std::fmod((end_pose.yaw - start_pose.yaw),(2.0*M_PI)) + (3.0*M_PI)),(2.0*M_PI)) - M_PI; // CHECK THIS!
  double yaw_c = start_pose.yaw + ((p_val - p_lower) / (p_upper - p_lower)) * angle_difference;

  return Pose({x_c, y_c, z_c, 0.0, 0.0, yaw_c});

}
*/

// Function for converting a p,q coordinate value into a euclidean coordinate using the pre-processed path to follow
/*
Node curve_to_euclid(Node node, std::shared_ptr<Path> global_path)
{
  double p_val = node.p;
  double q_val = node.q;
  int p_ind =  bisection(global_path->p, p_val);

  // Linearly interpolate a Euclidean Pose using the euclidean path and the relative p_val,q_val
  // TODO: need to use steam or lgmath se(3) classes for these poses, for now just using a vector
  Pose pose_c = lin_interpolate(p_ind, p_val, global_path);

  double x_i = pose_c.x - sin(pose_c.yaw)*q_val;
  double y_i = pose_c.y + cos(pose_c.yaw)*q_val;
  return Node(x_i,y_i);
}
*/