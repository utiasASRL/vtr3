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
 * \file cbit_path_planner.cpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

// This is the core of the curvilinear batch informed trees planner (CBIT) which runs asychronously to plan from the goal to the robot

#include "vtr_path_planning/cbit/cbit_path_planner.hpp"

using namespace vtr;
using namespace vtr::path_planning;

namespace {
// Function for converting Transformation matrices into se(2) [x, y, z, roll, pitch, yaw]
// Note we also might be able to do this with just lgmaths tran2vec operation?
inline std::tuple<double, double, double, double, double, double> T2xyzrpy(
    const tactic::EdgeTransform& T) {
  const auto Tm = T.matrix();
  return std::make_tuple(Tm(0, 3), Tm(1, 3), Tm(2,3), std::atan2(Tm(2, 1), Tm(2, 2)), std::atan2(-1.0*Tm(2, 0), sqrt(pow(Tm(2, 1),2) + pow(Tm(2, 2),2))), std::atan2(Tm(1, 0), Tm(0, 0)));
}
}


// Class Constructor:
CBITPlanner::CBITPlanner(CBITConfig conf_in, std::shared_ptr<CBITPath> path_in, BasePathPlanner::RobotState& robot_state, std::shared_ptr<std::vector<Pose>> path_ptr, std::shared_ptr<CBITCostmap> costmap_ptr, std::shared_ptr<CBITCorridor> corridor_ptr, std::shared_ptr<bool> solution_ptr, std::shared_ptr<double> width_ptr)
: robot_state_{robot_state} { 
  // Setting random seed
  srand((unsigned int)time(NULL));

  // Access the pointer to memory where the final result will be stored:
  cbit_path_ptr = path_ptr;
  cbit_costmap_ptr = costmap_ptr;
  valid_solution_ptr = solution_ptr;
  q_max_ptr = width_ptr;

  
  CLOG(INFO, "cbit_planner.path_planning") << "Planner is trying to initialize";
  conf = conf_in;
  global_path = path_in; 
  p_goal = std::make_shared<Node> (global_path->p[0], 0.0);
  p_goal_backup = p_goal; // store a backup of the goal for resets during repair mode
  p_start = std::make_shared<Node> (global_path->p.back(), 0.0);

  dynamic_window_width = conf.sliding_window_width;
  
  // DEBUG PLOTTING
  //initialize_plot();

  InitializePlanningSpace();
}

void CBITPlanner::InitializePlanningSpace()
{
  // Process the Transforms of the Tree to define the p,q space and its corresponding euclidean poses
  // Create global path class, which processes the discrete path and generates the p,q space reference vectors for conversions

  // Reserve some vector space, 2000 samples should be about good for our rolling window, but need to experiment with this and set as config
  tree.V.reserve(10000);
  tree.V_Old.reserve(10000);
  tree.E.reserve(10000);
  //tree.QV.reserve(10000);
  //tree.QE.reserve(10000);

  // Set initial cost to comes
  p_start->g_T = 0.0;
  p_start->g_T_weighted = 0.0;

  p_goal->g_T = INFINITY;
  p_goal->g_T_weighted = INFINITY;

  // Initialize tree
  tree.V.push_back(std::shared_ptr<Node> (p_start));
  samples.push_back(std::shared_ptr<Node> (p_goal));


  // Initialize obstacles
  // Permanent simulated obstacles can be populated here like so, but in the latest version obstacles are updated from the costmap
  //obs_rectangle = {{2.0, -2.0, 1, 3}, {3.0, 4.0, 3.0, 1.0}, {3.0, 8.0, 2.0, 2.0}}; // Obstacle format is {{x,y,w,h}} where x,y is the lower left corner coordinate
  //obs_rectangle = {{4.0, -1.0, 2, 2}}; // for plotting debugger tests
  obs_rectangle = {}; 

  // Initialize sliding window dimensions for plotting and radius expansion calc;
  sample_box_height = (*q_max_ptr) * 2.0;
  sample_box_width = conf.sliding_window_width + 2 * conf.sliding_window_freespace_padding;
}

// If we ever exit the planner due to a fault, we will do a hard reset, everything but the current robot_state (p_goal) and the inputs will be reinitialized
void CBITPlanner::stopPlanning() {
  planning_active_ = false;
}

void CBITPlanner::resetPlanner()
{
  stopPlanning();
  tree.V.clear();
  tree.E.clear();
  tree.QV2.clear();
  tree.QE2.clear();
  tree.V_Repair_Backup.clear();
  tree.V_Old.clear();
  samples.clear();
  path_x.clear();
  path_y.clear();

  dynamic_window_width = conf.sliding_window_width;
  repair_mode = false;
}


// Main planning function
void CBITPlanner::plan() {
  double prev_path_cost = INFINITY;
  int compute_time = 0;
  int vertex_rej_prob = 100;
  int dyn_batch_samples = conf.batch_samples;

  planning_active_ = true;

  // Grab the amount of time in ms between robot state updates
  int control_period_ms = (1.0 / conf.state_update_freq) * 1000.0;
  auto state_update_time = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(control_period_ms);

  repair_mode = false; // If we need to reset the planner, then we should also reset the repair mode here as well

  // benchmarking example code
  auto start_time = std::chrono::high_resolution_clock::now();

  // Variables for rolling average compute time metrics
  int batches_completed = 0;
  auto average_batch_time = 0.0;

  bool localization_flag = true; // Set the fact we are localized if we make it to this point

  for (int k = 0; planning_active_; k++)
  {
    // Check whether a robot state update should be applied
    // We only update the state if A: we have first found a valid initial solution, and B: if the current time has elapsed the control period
    if (conf.update_state == true)
    {
      //if ((p_goal->parent != nullptr) && (std::chrono::high_resolution_clock::now() >= state_update_time))
      if (std::chrono::high_resolution_clock::now() >= state_update_time) // Removed condition on valid solution, I think this helps overall
      {
        // Update timers
        CLOG(INFO, "cbit_planner.path_planning") << "Attempting to Update Robot State";
        state_update_time = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(control_period_ms);

        // get the euclidean robot state in the world frame from vt&r
        auto& chain = *robot_state_.chain;
        if (chain.isLocalized() == 0) {
          // If we ever become unlocalized, I think we just need to break, then set a flag to exit the outer loop
          // This also triggers after we reach end of path and effectively shuts down the planner
          localization_flag = false;
          CLOG(WARNING, "cbit_planner.path_planning") << "Localization was Lost, Exiting Inner Planning Loop";
          break;
        }
        
        std::tuple<double, double, double, double, double, double> robot_pose;

        const auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] = getChainInfo(chain);

        robot_pose = T2xyzrpy(T_w_p * T_p_r);
        CLOG(INFO, "cbit_planner.path_planning") << "Displaying Current Robot Transform: " << T_p_r;

        Pose se3_robot_pose = Pose(std::get<0>(robot_pose),(std::get<1>(robot_pose)),std::get<2>(robot_pose),std::get<3>(robot_pose),std::get<4>(robot_pose),std::get<5>(robot_pose));
        new_state = std::make_unique<Pose> (se3_robot_pose);

        // Perform a state update to convert the actual robot position to its corresponding pq space:
        p_goal = UpdateStateSID(curr_sid, T_p_r);
   
        // There is a case where the robot is actually starting behind the world frame of the teach path (or the localization thinks it is at first anyways)
        // This results in a situation where the updated state p value is 0, and the q value is the euclidean distance from the world frame to the robot state
        // (Which is a significant overestimate of the q_min). I think easiest way to handle this situation is to basically ignore the state update until p is non-zero
        // So here what we do is check p, if it is equal to zero still, then set q to be zero as well.
        if (p_goal->p == 0.0)
        {
          CLOG(WARNING, "cbit_planner.path_planning") << "Current Robot State is behind the first vertex, ignoring state update";
          p_goal->q = 0.0;
        }


        // Add the new goal (robot state) to the samples so it can be found again
        samples.push_back(p_goal); 
        
        // Generate some additional samples in a 2m local ring around the robot state
        double r_s;
        double theta_s;
        double x_s;
        double y_s;
        for (int i = 0; i < 50; i++)
        {
          r_s = 2.0 * sqrt(static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) ; 
          theta_s = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * 2.0 * 3.1415;
          x_s = r_s * cos(theta_s) + p_goal->p;
          y_s = r_s * sin(theta_s) + p_goal->q;
          Node new_sample(x_s, y_s);
          if (costmap_col(curve_to_euclid(new_sample)) == false)
          {
            samples.push_back(std::make_shared<Node> (new_sample));
          }
        }

        // Find vertices in the tree which are close to the new state, then populate the vertex queue with only these values.
        tree.QV2.clear();
        tree.QE2.clear();

       // Alternative: Take all points in a radius of 2.0m around the new robot state (only if they are a ahead though)
        double sample_dist;
        for (size_t i = 0; i < tree.V.size(); i++)
        {
          //tree.QV2.insert(std::pair<double, std::shared_ptr<Node>>((tree.V[i]->g_T_weighted + h_estimated_admissible(*tree.V[i], *p_goal)), tree.V[i]));
          sample_dist = calc_dist(*(tree.V[i]), *p_goal);
          //if ((sample_dist <= 5.0) && ((tree.V[i])->p > (p_goal->p + (conf.initial_exp_rad/2)))) // TODO: replace magic number with a param, represents radius to search for state update rewires
          if (sample_dist <= 5.0) // TODO: replace magic number with a param, represents radius to search for state update rewires
          {
            tree.QV2.insert(std::pair<double, std::shared_ptr<Node>>((tree.V[i]->g_T_weighted + h_estimated_admissible(*tree.V[i], *p_goal)), tree.V[i]));
          }
        }

        CLOG(INFO, "cbit_planner.path_planning") << "Robot State Updated Successfully, p: " << p_goal->p << " q: " << p_goal->q;
        CLOG(INFO, "cbit_planner.path_planning") << "QV size: " << tree.QV2.size();

        // When the planner resumes, this will cause it to immediately try to rewire locally to the new robot state in a short amount of time
      }
    }


    int m;
    if (tree.QV2.size() == 0 && tree.QE2.size() == 0)
    {
      CLOG(INFO, "cbit_planner.path_planning") << "STARTING NEW PLANNING BATCH!";
      if (k == 0)
      {
        m = conf.initial_samples;
      }
      else
      {
        m = conf.batch_samples;
      }

      // Only run this code if we have reached the end of a batch and have a new solution
      if (p_goal->parent != nullptr)
      {
        // Set that we have a valid solution
        *valid_solution_ptr = true;

        // Benchmark current compute time
        auto stop_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time);
        batches_completed = batches_completed + 1;
        average_batch_time = ((batches_completed * average_batch_time) + duration.count()) / (batches_completed + 1);
        

        compute_time = static_cast <int>(duration.count());

        // Debug message for online use (wont use std::out long term)
        CLOG(INFO, "cbit_planner.path_planning") << "New Batch - Iteration: " << k << "   Path Cost: " << p_goal->g_T_weighted << "   Batch Compute Time (ms): " << duration.count();
        CLOG(INFO, "cbit_planner.path_planning") << "Tree Vertex Size: " << tree.V.size() << " Tree Edge Size: " << tree.E.size() << " Sample Size: " << samples.size();

        // If the solution does not improve, backup the tree to restrict unnecessary growth
        if (abs(p_goal->g_T_weighted - prev_path_cost) < 1e-6)
        {
          CLOG(INFO, "cbit_planner.path_planning") << "There Was No Path Improvement This Batch, Restoring Tree to Previous Batch To Restrict Growth.";
          tree.V = tree.V_Old;
          tree.E = tree.E_Old;
        }
        prev_path_cost = p_goal->g_T_weighted;

        // if the batch compute time starts to creep up, it means the tree is getting abit to bloated
        // Its likely that more samples will not really help us much while going around obstacles, so we should reduce the batch size
        if (compute_time >= 200)
        {
          if (vertex_rej_prob > 60) // never want to exceed some minimums
          {
            dyn_batch_samples = dyn_batch_samples - 10;
            // Also increase the probability of rejecting vertex queue vertices to improve performance
            vertex_rej_prob = vertex_rej_prob - 10;
            CLOG(DEBUG, "cbit_planner.path_planning") << "Compute Falling Behind, Reducing Batch Size To " << dyn_batch_samples << " and Decreasing Vertex Acceptance Probability to " << vertex_rej_prob << "%";
          }
        }
        // Conversely if batch compute time is fast, we can handle more samples (up to a max)
        else
        {
          if (vertex_rej_prob < 100)
          {
            vertex_rej_prob = vertex_rej_prob + 2;
            dyn_batch_samples = dyn_batch_samples + 2;
            CLOG(DEBUG, "cbit_planner.path_planning") << "Compute Catching Up, Increasing Batch Size To " << dyn_batch_samples << " and Decreasing Vertex Acceptance Probability to " << vertex_rej_prob << "%";
          }
        }
        m = dyn_batch_samples;

        // Extract the solution
        std::tuple<std::vector<double>, std::vector<double>> curv_path = ExtractPath(robot_state_, cbit_costmap_ptr);
        path_x = std::get<0>(curv_path); // p coordinates of the current path
        path_y = std::get<1>(curv_path); // q coordinates of the current path

        // Store the Euclidean solution in the shared pointer memory (vector of Pose classes) so it can be accessed in the CBIT class
        std::vector<Pose> euclid_path = ExtractEuclidPath();
        //*cbit_path_ptr = euclid_path; // previously I update the cbit path ptr right away, but I think that because there is quite abit of delay in the corridor update process, I should wait to update them at the same time
        // This helps prevent synchronization issues

        // Reset the start time
        start_time = std::chrono::high_resolution_clock::now();

        //DEBUG PLOTTING (Very slow, do not use in normal operation)
        //auto plot_start_time = std::chrono::high_resolution_clock::now();
        //plot_tree(tree, *p_goal, path_x, path_y, samples);
        //auto plot_stop_time = std::chrono::high_resolution_clock::now();
        //auto duration_plot = std::chrono::duration_cast<std::chrono::milliseconds>(plot_stop_time - plot_start_time);
        //CLOG(ERROR, "cbit_planner.path_planning") << "Plot Time: " << duration_plot.count() << "ms";
        
        // Collision Check the batch solution:
        std::shared_ptr<Node> col_free_vertex = col_check_path_v2((p_goal->p + conf.sliding_window_width + conf.sliding_window_freespace_padding)); // outputs NULL if no collision
        if (col_free_vertex != nullptr)
        {
          // If there is a collision, prune the tree of all vertices to the left of the this vertex
          CLOG(WARNING, "cbit_planner.path_planning") << "Collision Detected:";
          CLOG(WARNING, "cbit_planner.path_planning") << "Collision Free Vertex is - p: " << col_free_vertex->p << " q: " << col_free_vertex->q;

          // Vertex Prune (maintain only vertices to the right of the collision free vertex)
          std::vector<std::shared_ptr<Node>> pruned_vertex_tree;
          pruned_vertex_tree.reserve(tree.V.size());
          for (size_t i =0; i < tree.V.size(); i++)
          {
            if (tree.V[i]->p >= col_free_vertex->p)
            {
              pruned_vertex_tree.push_back(tree.V[i]);
            }
          }
          tree.V = pruned_vertex_tree;


          // Edge Prune (maintain only edges to the right of the collision free vertex)
          std::vector<std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>  pruned_edge_tree;
          pruned_edge_tree.reserve(tree.E.size());
          for (size_t i = 0; i < tree.E.size(); i++)
          {
            if (std::get<1>(tree.E[i])->p >= col_free_vertex->p)
            {
              pruned_edge_tree.push_back(tree.E[i]);
            }
          }
  
          tree.E = pruned_edge_tree;

          // Reset the goal, and add it to the samples
          p_goal->parent = nullptr;
          p_goal->g_T = INFINITY;
          p_goal->g_T_weighted = INFINITY;
          samples.clear(); // first clear samples so they dont accumulate excessively
          samples.push_back(p_goal);

          // This will cause samplefreespace to run this iteration, but if we made it here m will be set to conf.batch_samples (as its reset every iteration)
          m = conf.initial_samples;  

          // Re-initialize sliding window dimensions for plotting and radius expansion calc;
          sample_box_height = (*q_max_ptr) * 2.0;
          sample_box_width = conf.sliding_window_width + 2 * conf.sliding_window_freespace_padding;

          // Disable pre-seeds from being regenerated
          repair_mode = true;

          // Sample free-space wont generate pre-seeds, so we should generate our own in the portion of the tree that was dropped (col_free_vertex.p to p_goal.p)
          int pre_seeds = abs(col_free_vertex->p - p_goal->p) / 0.25; // Note needed to change p_goal to p_zero. When the sliding window padding is large, pre-seeds wont get generated all the way to the goal
          double p_step = 0.25;
          double p_val = p_goal->p;
          for (int i = 0; i < (pre_seeds-1); i++) 
          {
            samples.push_back(std::make_shared<Node>(p_val + p_step , 0));
            p_val += p_step;
          }

        }
        else
        {
          // Reset the free space flag in sample freespace
          repair_mode = false;
        }
        
        *cbit_path_ptr = euclid_path; // Update the pointer to point to the latest euclidean path update
        
      }
      else
      {
        *valid_solution_ptr = false;
        CLOG(WARNING, "cbit_planner.path_planning") << "There is currently no valid solution to the planning problem - Trying again";
      }
      Prune(p_goal->g_T, p_goal->g_T_weighted);

      // Resample
      if (p_goal->g_T_weighted < INFINITY)
      {
        Node::Path new_samples = SampleBox(m);
        samples.insert(samples.end(), new_samples.begin(), new_samples.end());
        CLOG(INFO, "cbit_planner.path_planning") << "Sampling Box";
      }
      else
      {
        Node::Path new_samples = SampleFreeSpace(m);
        samples.insert(samples.end(), new_samples.begin(), new_samples.end());
        CLOG(INFO, "cbit_planner.path_planning") << "Sample Free Space";
      }

      //TODO Use iterator to move vectors around!
    
      // Backup the old tree:
      tree.V_Old.clear();
      tree.E_Old.clear();
      for (size_t i = 0; i < tree.V.size(); i++)
      {
        tree.V_Old.push_back((tree.V[i]));
      }

      for (size_t i = 0; i < tree.E.size(); i++)
      {
        tree.E_Old.push_back(tree.E[i]);
      }

      // Initialize the Vertex Queue with all nearby vertices in the tree;
      std::random_device rd;     // Only used once to initialise (seed) engine
      std::mt19937 rng(rd());    // Random-number engine used (Mersenne-Twister in this case)
      std::uniform_int_distribution<int> uni(1,100); // Guaranteed unbiased 
      for (size_t i = 0; i < tree.V.size(); i++)
      {
        auto random_integer = uni(rng);
        random_integer = static_cast <double>(random_integer);
        // if we have no 1st batch solution (we are in the first iteration or have just reset), add the whole tree to the queue
        if (k == 0)
        {
          tree.QV2.insert(std::pair<double, Node::Ptr>((tree.V[i]->g_T_weighted + h_estimated_admissible(*tree.V[i], *p_goal)), tree.V[i]));
        }
        // Otherwise, only add the portions of the tree within the sliding window to avoid processing preseeded vertices which are already optimal
        else if (((vertex_rej_prob / random_integer) >= 1.0)) // for some reason using the lookahead queue doesnt work reliably for collisions, not sure why, need to investigate
        {
          tree.QV2.insert(std::pair<double, Node::Ptr>((tree.V[i]->g_T_weighted + h_estimated_admissible(*tree.V[i], *p_goal)), tree.V[i]));
        }

      }
      CLOG(INFO, "cbit_planner.path_planning") << "QV size after batch: " << tree.QV2.size();

      // Dynamic Expansion radius selection:
      // I found in practice constant radii value work best, I think because of rewiring invalidating the expansion radius calculations. Leave as constant for now
      //if (p_goal->g_T != INFINITY)
      //{
      //  conf.initial_exp_rad = exp_radius((tree.V.size() + samples.size()), sample_box_height, sample_box_width, conf.eta);
      //  CLOG(DEBUG, "path_planning.cbit_planner") << "New expansion radius is: " << conf.initial_exp_rad;
      //}
    }

    auto start_bench_time = std::chrono::high_resolution_clock::now();



    // Planning loop starts here
    while (BestVertexQueueValue() < BestEdgeQueueValue())
    {
      //auto test = BestInVertexQueue();
      ExpandVertex(BestInVertexQueue());
    }
    auto stop_bench_time = std::chrono::high_resolution_clock::now();
    auto duration_bench = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_bench_time - start_bench_time);
    CLOG(DEBUG, "cbit_planner.compute_metrics") << "Compute 1: " << duration_bench.count();
    start_bench_time = std::chrono::high_resolution_clock::now();

    
    // Generate prospective nodes
    Node::Edge prospective_edge = BestInEdgeQueue();

    std::shared_ptr<Node> vm = std::get<0>(prospective_edge);
    std::shared_ptr<Node> xm = std::get<1>(prospective_edge);

    // Handle edge case where a perfect path results in empty queues (see python code)
    if (vm == NULL) 
    {
      // If we just continue here, we enter a state where after every single iteration we will complete a batch, but I think this is fine
      // It will just stay this way until we move off the path and have a non perfect path solution
      continue;
    }
    // Remove the edge from the queue (I think best to do this inside BestInEdgeQueue function)

    stop_bench_time = std::chrono::high_resolution_clock::now();
    duration_bench = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_bench_time - start_bench_time);
    CLOG(DEBUG, "cbit_planner.compute_metrics") << "Compute 2: " << duration_bench.count();
    start_bench_time = std::chrono::high_resolution_clock::now();

    // Collision check and update tree/queues
    if (vm->g_T_weighted + calc_weighted_dist(*vm, *xm, conf.alpha) + h_estimated_admissible(*xm, *p_goal) < p_goal->g_T_weighted)
    {
      double actual_cost = cost_col(obs_rectangle, *vm, *xm);
      double weighted_cost = weighted_cost_col(obs_rectangle, *vm, *xm);

      stop_bench_time = std::chrono::high_resolution_clock::now();
      duration_bench = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_bench_time - start_bench_time);
      CLOG(DEBUG, "cbit_planner.compute_metrics") << "Compute 3: " << duration_bench.count();
      start_bench_time = std::chrono::high_resolution_clock::now();

      if (g_estimated_admissible(*vm, *p_start) + weighted_cost + h_estimated_admissible(*xm, *p_goal) < p_goal->g_T_weighted)
      {
        if (vm->g_T_weighted + weighted_cost < xm->g_T_weighted)
        {
          // Check if xm is in the tree, if it is, we need to do some rewiring of any other edge which has xm as an endpoint
          if (node_in_tree_v2(xm) == true)
          {
            // loop through edges, delete those which have the same xm end node
            int upper_edge_index = tree.E.size();
            for (int i = 0; i < upper_edge_index; i++)
            {
              if (std::get<1>(tree.E[i]) == xm) // changed to equating pointer addresses
              {
                  // Once we process the vertex and have it ready to return, remove it from the edge queue (This is a fast way of doing so)
                  auto it = tree.E.begin() + i; 
                  *it = std::move(tree.E.back());
                  tree.E.pop_back();

                  // NEED TO BE VERY CAREFUL WITH THE INDEX HERE, ANYTIME WE DO A DELETION WE NEED TO DECREMENT, AS WELL AS ADJUST THE UPPER ITERATOR SIZE
                  i = i - 1;
                  upper_edge_index = upper_edge_index - 1;
              }
            }

            // Set cost to comes
            xm->g_T_weighted = vm->g_T_weighted + weighted_cost;
            xm->g_T = vm->g_T + actual_cost;
            xm->parent = vm;
          }
          
          // Otherwise we can remove xm from the samples, add xm to the tree vertices and queue
          else
          {
            // If the end point is not in the tree, it must have come from a random sample.
            // Remove it from the samples, add it to the vertex tree and vertex queue:
            for (size_t i = 0; i < samples.size(); i++)
            {
              if (samples[i] == xm)
              {
                auto it = samples.begin() + i;
                *it = std::move(samples.back());
                samples.pop_back();

                break; // Once we find the one to delete, it is safe to break, there should only every be one
              }
            }
            // Set cost to comes
            xm->g_T_weighted = vm->g_T_weighted + weighted_cost;
            xm->g_T = vm->g_T + actual_cost;
            xm->parent = vm;

            tree.V.push_back(xm);
            tree.QV2.insert(std::pair<double, Node::Ptr>(xm->g_T_weighted + h_estimated_admissible(*xm, *p_goal), xm));
          }
          stop_bench_time = std::chrono::high_resolution_clock::now();
          duration_bench = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_bench_time - start_bench_time);
          CLOG(DEBUG, "cbit_planner.compute_metrics") << "Compute 4: " << duration_bench.count();
          start_bench_time = std::chrono::high_resolution_clock::now();

          // Generate edge, create parent chain
          tree.E.push_back(std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {vm, xm});
        
          // Filter edge queue as now any other edge with worse cost heuristic can be ignored
          std::multimap<double, std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>::iterator itr = tree.QE2.begin();
          while (itr != tree.QE2.end())
          {
            auto edge = itr->second;
            Node v = *std::get<0>(edge);
            std::shared_ptr<Node> x = std::get<1>(edge);

            if ((x == xm) && (v.g_T_weighted + calc_weighted_dist(v, *xm, conf.alpha) >= xm->g_T_weighted))
            {
              // Once again, be very careful with the iteration indices when deleting
              // Need to decrement both the iterator and the total size
              itr = tree.QE2.erase(itr); // Erase returns the itr of the value in the map following the previous iterator
            }
            else
            {
              itr++;
            }
          } 
          stop_bench_time = std::chrono::high_resolution_clock::now();
          duration_bench = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_bench_time - start_bench_time);
          CLOG(DEBUG, "cbit_planner.compute_metrics") << "Compute 5: " << duration_bench.count();
          start_bench_time = std::chrono::high_resolution_clock::now();
        }
      }
      stop_bench_time = std::chrono::high_resolution_clock::now();
      duration_bench = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_bench_time - start_bench_time);
      CLOG(DEBUG, "cbit_planner.compute_metrics") << "Compute 6: " << duration_bench.count();
    }
    // If there is no edges in the queue which can improve the current tree, clear the queues which will consequently trigger the end of a batch
    else
    {
      tree.QV2.clear();
      tree.QE2.clear();
    }
 
  } // End of main planning for loop
  
  CLOG(INFO, "cbit_planner.path_planning") << "Reached End of Plan, Exiting cleanly";
  cbit_path_ptr->clear();
} // End of main planning function



Node::Path CBITPlanner::SampleBox(int m)
{
  // Create a vector to store the pointers to the new samples we are going to generate.
  Node::Path new_samples;

  // Initialize sample box parameters

  // path_x,y stores the current best path solution
  double c_best = p_goal->g_T;
  double c_min = p_start->p - p_goal->p;
  double padding = (c_best - c_min) / 2.0; // Padding to apply to maintain the heuristic ellipse additional width for probabilstic completeness
  double lookahead = dynamic_window_width;
  double box_tolerance = 0.1; // Need to add a little extra height to the box when using ROC regions TODO make this config param

  // Calculate dimensions
  double q_max = *std::max_element(path_y.begin(), path_y.end()) + box_tolerance; // apparently this function can be abit slow, so be aware
  double q_min = *std::min_element(path_y.begin(), path_y.end()) - box_tolerance;
  
  q_max = std::max(fabs(q_max), fabs(q_min));

  // maintain a minimum sample box height, this prevents sample density from becoming too ridiculous when following the path
  if (q_max < 0.5)
  {
    q_max = 0.5;
  }

  double p_max = p_goal->p + lookahead + padding;
  double p_zero = p_goal->p - padding;

  // Set the sliding window box height so it can be used for radius expansion purposes
  sample_box_height = q_max * 2;

  // Generate random samples
  int ind = 0;
  while (ind < m)
  {
    // uniformly sample the box
    double rand_p = p_zero + (rand() / (RAND_MAX / (p_max-p_zero)));
    double rand_q = (-1.0*q_max) + (rand() / (RAND_MAX / (q_max- (-1.0*q_max))));
    Node node(rand_p, rand_q);

    // Before we add the sample to the sample vector, we need to collision check it
    if (costmap_col(curve_to_euclid(node))) // Using costmap for collision check
    {
      continue;
    }
    else
    {
      new_samples.push_back(std::make_shared<Node>(node));
      ind++;
    }
  }
  return new_samples;
}



Node::Path CBITPlanner::SampleFreeSpace(int m)
{
  Node::Path new_samples;
  
  double p_max = p_goal->p + dynamic_window_width + conf.sliding_window_freespace_padding;
  double p_zero = p_goal->p - conf.sliding_window_freespace_padding;
  double q_max = *q_max_ptr;

  int ind = 0;
  while (ind < m)
  {
    // uniformly sample the configuration space
    double rand_p = p_zero + (rand() / (RAND_MAX / (p_max-p_zero)));
    double rand_q = (-1.0*q_max) + (rand() / (RAND_MAX / (q_max- (-1.0*q_max))));
    auto node = std::make_shared<Node>(rand_p, rand_q);

    if (costmap_col(curve_to_euclid(*node)))
    {
      continue;
    }
    else
    {
      new_samples.push_back(node);
      ind++; // only increment if we do not have a collision
    }
  }

  // Generating Pre-seeds
  if (repair_mode == false)
  {

    // hardcoded pre-seed interval of 25cm for now (really no reason for it to be much smaller or bigger than this)
    int pre_seeds = abs(p_zero - p_start->p) / 0.25; // Note needed to change p_goal to p_zero. When the sliding window padding is large, pre-seeds wont get generated all the way to the goal

    // In the python version I do this line thing which is more robust, but for now im going to do this quick and dirty
    double p_step = 0.25;
    double p_val = p_zero; 
    for (int i = 0; i < (pre_seeds-1); i++) 
    {
      new_samples.push_back(std::make_shared<Node>(p_val+p_step, 0.0));

      p_val += p_step;
    }
  }
  return new_samples;

}



void CBITPlanner::Prune(double c_best, double c_best_weighted)
{
  // Determine an upper bound cost threshold for the prune by looping through all nodes in the current best path and computing an admissible
  // f_estimated = h_estimate + g_estimated heuristic
  double cost_threshold = 0.0;
  if (c_best_weighted < INFINITY)
  {
    double vertex_cost;
    double p;
    double q;
    for (size_t i = 0; i < path_x.size(); i++)
    {
      p = path_x[i];
      q = path_y[i];
      vertex_cost = f_estimated(Node(p,q), *p_start, *p_goal, conf.alpha);
      if (vertex_cost > cost_threshold)
      {
        cost_threshold = vertex_cost;
      }
    }
  }
  else
  {
    cost_threshold = c_best_weighted;
  }

  // Using the cost threshold, prune the samples (weighted eyeball prune)
  // I think for vectors, its probably faster to just copy the samples to a new pointer vector which do satisfy the constraint,
  // Then overwrite the samples, instead of doing many many deletions
  std::vector<std::shared_ptr<Node>> samples_pruned;
  for (size_t i = 0; i < samples.size(); i++)
  {
    if (f_estimated(*samples[i], *p_start, *p_goal, conf.alpha) < cost_threshold)// Also handles inf flagged values
    {
      samples_pruned.push_back(samples[i]);
    }
  }

  // We also check the tree and add samples for unconnected vertices back to the sample set
  std::vector<Node::Ptr> vertex_pruned;
  int vertices_size = tree.V.size(); 
  for (size_t i = 0; i < vertices_size; i++)
  {
    if (tree.V[i]->g_T_weighted == INFINITY)
    {
      samples_pruned.push_back(tree.V[i]);
    }
    if ((f_estimated(*tree.V[i], *p_start, *p_goal, conf.alpha) <= cost_threshold) && (tree.V[i]->g_T_weighted < INFINITY))
    {
      
      vertex_pruned.push_back(tree.V[i]);
    }
  }
  // After we do both the above loops update the sample vector
  samples = samples_pruned;
  tree.V = vertex_pruned;


  // Similar Prune of the Edges
  std::vector<std::tuple<Node::Ptr, Node::Ptr>>  edge_pruned;
  for (size_t i = 0; i <tree.E.size(); i++)
  {
    // In the below condition, I also include the prune of vertices with inf cost to come values
    if ((f_estimated(*std::get<0>(tree.E[i]), *p_start, *p_goal, conf.alpha) <= cost_threshold) && (f_estimated(*std::get<1>(tree.E[i]), *p_start, *p_goal, conf.alpha) <= cost_threshold))
    {
      edge_pruned.push_back(std::tuple<Node::Ptr, Node::Ptr> (tree.E[i]));
    }
  }
  tree.E = edge_pruned;
}



// New Simpler State Update function that instead uses the robots localization frame SID to help find the robot p,q space state
Node::Ptr CBITPlanner::UpdateStateSID(size_t SID, vtr::tactic::EdgeTransform T_p_r)
{
  // Find the corresponding global pose p,q value at the current SID (which should be just behind the actual current state)
  double current_pq_sid_p = global_path->p[SID];

  std::vector<Node> euclid_subset;
  euclid_subset.reserve((conf.roc_lookahead * conf.roc_lookahead * conf.curv_to_euclid_discretization));

  // The length of the subset is determined by the configuration parameter lookahead distance and the desired discretization
  // Use the SID values to help predict how far we should look ahead look ahead without falling into false minim on crossing paths.
  double lookahead_range = (*p_goal).p + conf.roc_lookahead; //default value from config
  for (size_t ID = SID; ID < global_path->p.size(); ID += 1)
  {
    if ((*p_goal).p < global_path->p[ID])
    {
      lookahead_range = global_path->p[ID];
    }
  }

  for (double i = (*p_goal).p; i < lookahead_range; i += (1.0 / (conf.roc_lookahead * conf.curv_to_euclid_discretization)))
  {
    euclid_subset.push_back(curve_to_euclid(Node(i,0)));
  }

  // calc q_min
  double q_min = *q_max_ptr;
  double q;
  Node closest_pt;
  size_t closest_pt_ind;

  for (size_t i = 0; i < euclid_subset.size(); i++)
  {
    double dx = new_state->x - euclid_subset[i].p;
    double dy = new_state->y - euclid_subset[i].q;
    q = sqrt((dy * dy) + (dx * dx));

    if (q < q_min)
    {
      q_min = q;
      closest_pt = euclid_subset[i];
      closest_pt_ind = i;
    }
  }
  // Handle weird seg fault case
  if ((closest_pt_ind) > (euclid_subset.size()-2))
  {
    closest_pt_ind = euclid_subset.size()-2;
  }

  double test_dx = euclid_subset[closest_pt_ind+1].p - closest_pt.p;
  double test_dy = euclid_subset[closest_pt_ind+1].q - closest_pt.q;
  double test_yaw = atan2(test_dy,test_dx);

  // Ensure that yaw1 and yaw2 are in the range -pi to pi
  double yaw1 = fmod(test_yaw + 2*M_PI, 2*M_PI);
  if (yaw1 < 0) {
      yaw1 += 2*M_PI;
  }
  double yaw2 = fmod(new_state->yaw + 2*M_PI, 2*M_PI);
  if (yaw2 < 0) {
      yaw2 += 2*M_PI;
  }

  // Calculate the angle between the yaws and ensure it is in the range 0 to pi
  double angle = fmod(fabs(yaw1 - yaw2), 2*M_PI);
  if (angle > M_PI) {
      angle = 2*M_PI - angle;
  }

  // Check if the angle is greater than 90 degrees
  double local_path_direction;
  if (angle > M_PI/2) {
      local_path_direction = -1.0;
  } else {
      local_path_direction = 1.0;
  }

  // Once we have the closest point, we need to try to find the sign (above or below the path)
  // This is a little tricky, but I think what is reasonable is to create a plane with the closest point and its neighbours
  // Then depending on which side the new state point is of the plane, is the sign we use for q_min:

  Node A;
  Node B;
  if (closest_pt_ind == 0)
  {
    A = euclid_subset[closest_pt_ind];
  }
  else
  {
    A = euclid_subset[closest_pt_ind-1];
  }

  if (closest_pt_ind == euclid_subset.size() - 1)
  {
    B = euclid_subset[closest_pt_ind];
  }
  else
  {
    B = euclid_subset[closest_pt_ind + 1];
  }


  int q_sign = sgn((B.p - A.p) * (new_state->y - A.q) - ((B.q - A.q) * (new_state->x - A.p))); 
  
  // Note I think we also need to take into account the direction the robot is facing on the path for reverse planning too
  q_min = q_min * q_sign * local_path_direction;

  // Once we have the closest point on the path, it may not actually be the correct p-value because of singularities in the euclid to curv conversion
  // We need to use this points p-value as a starting point, then search p, qmin space in either direction discretely and find the point with
  // The lowest pose error (including heading) (Should be able to ignore roll and pitch though)
  double pose_err = INFINITY;
  Node test_pt;
  double test_err;
  Node new_state_pq;

  for (double p = p_goal->p; p < (p_goal->p + conf.roc_lookahead); p += (1.0 / (conf.roc_lookahead * conf.curv_to_euclid_discretization)))
  {
    test_pt = curve_to_euclid(Node(p,q_min));
    double dx = test_pt.p - new_state->x;
    double dy = test_pt.q - new_state->y;
    test_err = sqrt((dy * dy) + (dx * dx));
    if (test_err < pose_err)
    {
      pose_err = test_err;
      new_state_pq = Node(p, q_min);
    }
  }

  // Now update the goal and its cost to come:
  p_goal = std::make_shared<Node>(new_state_pq);
  p_goal->g_T = INFINITY;
  p_goal->g_T_weighted = INFINITY;

  return p_goal;
}



double CBITPlanner::BestVertexQueueValue()
{
  if (tree.QV2.size() == 0)
  {
    return INFINITY;
  }
  
  // New multimap implementation of this:
  std::multimap<double, std::shared_ptr<Node>>::iterator itr = tree.QV2.begin();
  double min_vertex_cost = itr -> first;
  return min_vertex_cost;
}



double CBITPlanner::BestEdgeQueueValue()
{
  if (tree.QE2.size() == 0)
  {
    return INFINITY;
  }

  // Using multimaps to accomplish the same thing:
  std::multimap<double, std::tuple<Node::Ptr, Node::Ptr>>::iterator itr = tree.QE2.begin();
  double min_edge_cost = itr -> first;
    
  return min_edge_cost;
}



std::shared_ptr<Node> CBITPlanner::BestInVertexQueue()
{
  if (tree.QV2.size() == 0)
  {
    CLOG(DEBUG, "cbit_planner.path_planning") << "Vertex Queue is Empty, Something went Wrong!";
  }
  // New multimap implementation of this:
  std::multimap<double, std::shared_ptr<Node>>::iterator itr = tree.QV2.begin();
  std::shared_ptr<Node> best_vertex = itr->second;
  tree.QV2.erase(itr);

  return best_vertex;
}



void CBITPlanner::ExpandVertex(std::shared_ptr<Node> v)
{
  // Note its easier in c++ to remove the vertex from the queue in the bestinvertexqueue function instead
  // Find nearby samples and filter by heuristic potential
  for (size_t i = 0; i < samples.size(); i++)
  {
    if ((calc_dist(*samples[i], *v) <= conf.initial_exp_rad) && ((g_estimated_admissible(*v, *p_start) + calc_weighted_dist(*v, *samples[i], conf.alpha) + h_estimated_admissible(*samples[i], *p_goal)) <= p_goal->g_T_weighted))
    {
      samples[i]->g_T = INFINITY;
      samples[i]->g_T_weighted = INFINITY;

      // direct method
      tree.QE2.insert(std::pair<double, std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>((v->g_T_weighted + calc_weighted_dist(*v,*samples[i],conf.alpha) + h_estimated_admissible(*samples[i], *p_goal)), std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {(v), (samples[i])}));

    }
  }  

  // find nearby vertices and filter by heuristic potential
  for (size_t i = 0; i < tree.V.size(); i++)
  {
    if ((calc_dist(*tree.V[i], *v) <= conf.initial_exp_rad) && ((g_estimated_admissible(*v, *p_start) + calc_weighted_dist(*v, *tree.V[i], conf.alpha) + h_estimated_admissible(*tree.V[i], *p_goal)) < p_goal->g_T_weighted) && ((v->g_T_weighted + calc_weighted_dist(*v, *tree.V[i], conf.alpha)) < tree.V[i]->g_T_weighted))
    {
      if (edge_in_tree_v2(v, tree.V[i]) == false)
      {
        // If all conditions satisfied, add the edge to the queue
        tree.QE2.insert(std::pair<double, std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>((v->g_T_weighted + calc_weighted_dist(*v,*tree.V[i],conf.alpha) + h_estimated_admissible(*tree.V[i], *p_goal)), std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {(v), (tree.V[i])}));

      }
    } 
  }
}


Node::Edge CBITPlanner::BestInEdgeQueue()
{
  if (tree.QE2.size() == 0) // need to handle a case where the return path is 100% optimal in which case things get stuck and need ot be flagged to break
  {
    CLOG(DEBUG, "cbit_planner.path_planning") << "Edge Queue is Empty, Solution Could Not be Improved This Batch";
    repair_mode = true;
    return Node::Edge {NULL, NULL};
  }

  // Equivalent code using multimaps:
  std::multimap<double, std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>::iterator itr = tree.QE2.begin();
  tree.QE2.erase(itr);

  return itr->second;
}


std::tuple<std::vector<double>, std::vector<double>> CBITPlanner::ExtractPath(BasePathPlanner::RobotState& robot_state, std::shared_ptr<CBITCostmap> costmap_ptr)
{
  int inf_loop_counter = 0;

  Node node = *p_goal;
  
  std::vector<double> path_p = {node.p};
  std::vector<double> path_q = {node.q};

  while(node.parent != nullptr)
  {
    inf_loop_counter = inf_loop_counter + 1;
    if (inf_loop_counter >= 5000) {
      CLOG(DEBUG, "cbit_planner.path_planning") << "Something Went Wrong - Infinite Loop Detected, Initiating Hard Reset:";
    }
    node = *node.parent;
    path_p.push_back(node.p);
    path_q.push_back(node.q);

  }
  return {path_p, path_q};
}


// Function to convert the resulting p,q path into a finely discretized euclidean path

Pose::Path  CBITPlanner::ExtractEuclidPath()
{
  Node node = *p_goal;
  
  std::vector<double> path_x;
  std::vector<double> path_y;

  std::vector<double> path_z;
  std::vector<double> path_yaw;
  std::vector<double> path_p;
  std::vector<double> path_q;

  std::vector<double> p = global_path->p;
  Pose::Path disc_path = global_path->disc_path;


  std::vector<Node> node_list = {node};
  node_list.reserve(1000);
  double discretization = conf.curv_to_euclid_discretization;


  while(node.parent != nullptr)
  {
    node = *node.parent;
    node_list.push_back(node);
  }

  for (size_t i=0; i < (node_list.size()-1); i++)
  {
    Node start = node_list[i];
    Node end = node_list[i+1];
    std::vector<double> p_disc;
    std::vector<double> q_disc;

    if (start.p == end.p) // handle edge case for vertical lines
    {

      p_disc = linspace(start.p, end.p, discretization);
      q_disc = linspace(start.q, end.q, discretization);
    }
    else // Generate a vector of discrete subpoints for the current edge we are processing
    {
      double slope = (end.q - start.q) / (end.p - start.p);
      double b = start.q - (slope * start.p);
      p_disc = linspace(start.p, end.p, discretization);
      q_disc.reserve(p_disc.size());
      for (size_t i=0; i < p_disc.size(); i++)
      {
        q_disc.push_back((p_disc[i] * slope + b));
      }
    }

    // Once we have built the discretized vectors, loop through them, convert each point to Euclidean space and store it in the Euclidean path vector
    for (size_t i=0; i < p_disc.size(); i ++)
    {
      Node euclid_pt = curve_to_euclid(Node(p_disc[i], q_disc[i]));
      path_x.push_back(euclid_pt.p);
      path_y.push_back(euclid_pt.q);
      path_z.push_back(euclid_pt.z);

      // Yaw estimation from teach path
      int p_iter = 0;
      while (p_disc[i] > p[p_iter])
      {
          p_iter = p_iter+1;
      }
      path_yaw.push_back(disc_path[p_iter-1].yaw);
      path_p.push_back(p_disc[i]);
      path_q.push_back(q_disc[i]);
    }
  }
  

  // Temporarily reshaping the output into the form we need (vector of poses)
  std::vector<Pose> pose_vector;
  for (size_t i = 0; i < path_x.size(); i++)
  {
    Pose next_pose = Pose(path_x[i], path_y[i], path_z[i], 0.0, 0.0, path_yaw[i]);
    next_pose.p = path_p[i];
    next_pose.q = path_q[i];
    pose_vector.push_back(next_pose);// Temporarily set pose with x,y coordinate, no orientation
  }
  return pose_vector;
}



std::shared_ptr<Node> CBITPlanner::col_check_path_v2(double max_lookahead_p)
{

  // Generate path to collision check
  Node::Path curv_path;
  std::shared_ptr<Node> node_ptr = p_goal;
  curv_path.push_back(node_ptr);

  while ((node_ptr->parent != nullptr))
  {
    node_ptr = node_ptr->parent;
    curv_path.push_back(node_ptr);
  }

  // find the first vertex which results in a collision (curv_path is generated from left to right, so we should search in reverse)
  std::shared_ptr<Node> col_free_vertex = nullptr;

  for (long i = curv_path.size()-1; i >= 0; i--)
  {
    Node vertex = *curv_path[i];
    // If the vertex in the path is outside of our sliding window then skip it
    // This prevents us from having to rewire the path for distance obstacles outside of our window (especially in cases where our path crosses itself)
    if (vertex.p > max_lookahead_p)
    {
      continue;
    }
    Node euclid_pt = curve_to_euclid(vertex);

    if (costmap_col_tight(euclid_pt))
    {
      if (i+4 < curv_path.size()-1)
      {
        col_free_vertex = curv_path[i+4]; // im actually finding that the next vertex may be a little too close to the obstacles. Better to take one slightly further ahead if we can
      }
      else
      {
        col_free_vertex = curv_path[i+1]; 
      }
    }
  }
  return col_free_vertex;
}


// Function which takes in a beginning vertex v, and an end vertex x, and checks whether its in the tree or not already
// This version uses address matching (safer)
bool CBITPlanner::edge_in_tree_v2(std::shared_ptr<Node> v, std::shared_ptr<Node> x)
{
  int edges_size = tree.E.size();
  for (size_t i = 0; i < edges_size; i++ )
  {
    // Alternative way to do this using node addresses
    if ((std::get<0>(tree.E[i]) == v) && (std::get<1>(tree.E[i]) == x))
    {
      return true;
    }
  }
  return false;
}



// Function for checking whether a node lives in the Vertex tree, updated to use address matching (safer)
bool CBITPlanner::node_in_tree_v2(std::shared_ptr<Node> x)
{
  for (size_t i = 0; i < tree.V.size(); i++ )
  {
    if (x == tree.V[i])
    {
        return true;
    }
  }
  return false;
}


// Edge cost calculation with collision check
double CBITPlanner::cost_col(std::vector<std::vector<double>> obs, Node start, Node end)
{
  if (discrete_collision(obs, conf.curv_to_euclid_discretization, start, end))
  {
    return INFINITY;
  }
  return calc_dist(start, end);
}

// Weighted Edge cost calculation with collision check
double CBITPlanner::weighted_cost_col(std::vector<std::vector<double>> obs, Node start, Node end)
{
  if (discrete_collision(obs, conf.curv_to_euclid_discretization, start, end))
  {
    return INFINITY;
  }
  return calc_weighted_dist(start, end, conf.alpha);
}



// Function for converting a p,q coordinate value into a euclidean coordinate using the pre-processed path to follow
Node CBITPlanner::curve_to_euclid(Node node)
{
  double p_val = node.p;
  double q_val = node.q;
  int p_ind =  bisection(global_path->p, p_val);

  // Linearly interpolate a Euclidean Pose using the euclidean path and the relative p_val,q_val
  // TODO: need to use steam or lgmath se(3) classes for these poses, for now just using a vector
  Pose pose_c = lin_interpolate(p_ind, p_val);

  double x_i = pose_c.x - sin(pose_c.yaw)*q_val;
  double y_i = pose_c.y + cos(pose_c.yaw)*q_val;

  // Experimental, also interpolate the z value
  double z_i = pose_c.z; //+ cos(pose_c.yaw)*q_val; // I think here we just want to set the z to whatever the pose_c.z value

  return Node(x_i,y_i,z_i);
}

Pose CBITPlanner::lin_interpolate(int p_ind, double p_val)
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



// This collision check is only used at the end of each batch and determines whether the path should be rewired using the bare minimum obstacle distance
// Under normal operation we plan paths around a slightly more conservative buffer around each obstacle (equal to influence dist + min dist)
bool CBITPlanner::costmap_col_tight(Node node)
{
  Eigen::Matrix<double, 4, 1> test_pt({node.p, node.q, node.z, 1});

  auto collision_pt = cbit_costmap_ptr->T_c_w * test_pt;

  // Round the collision point x and y values down to the nearest grid resolution so that it can be found in the obstacle unordered_map
  float x_key = floor(collision_pt[0] / cbit_costmap_ptr->grid_resolution) * cbit_costmap_ptr->grid_resolution;
  float y_key = floor(collision_pt[1] / cbit_costmap_ptr->grid_resolution) * cbit_costmap_ptr->grid_resolution;

  float grid_value;

  // Check to see if the point is in the obstacle map
  // We need to use a try/catch in this metod as if the key value pair doesnt exist (it usually wont) we catch an out of range error
  try  {
    grid_value = cbit_costmap_ptr->obs_map.at(std::pair<float, float> (x_key, y_key));
  } catch (std::out_of_range &e) {
    grid_value = 0.0;
  }

  if (grid_value >= 0.99) // By switching this from > 0.0 to 0.99, we effectively only collision check the path out to the "minimum_distance" obs config param
  {
    return true;
  }

  // If we make it here can return false for no collision
  return false;
}



// More conservative costmap checking out to a distance of "influence_distance" + "minimum_distance" away
bool CBITPlanner::costmap_col(Node node)
{
  Eigen::Matrix<double, 4, 1> test_pt({node.p, node.q, node.z, 1});
  auto collision_pt = cbit_costmap_ptr->T_c_w * test_pt;

  // Round the collision point x and y values down to the nearest grid resolution so that it can be found in the obstacle unordered_map
  float x_key = floor(collision_pt[0] / cbit_costmap_ptr->grid_resolution) * cbit_costmap_ptr->grid_resolution;
  float y_key = floor(collision_pt[1] / cbit_costmap_ptr->grid_resolution) * cbit_costmap_ptr->grid_resolution;

  float grid_value;

  // Check to see if the point is in the obstacle map
  // If it isn't in the map we will catch an out of range error
  try {
      grid_value = cbit_costmap_ptr->obs_map.at(std::pair<float, float> (x_key, y_key));
  } catch (std::out_of_range &e) {
    grid_value = 0.0;
  }

  if (grid_value > 0.0) {
    return true;
  }

  // If we make it here, return false for no collision
  return false;
}



bool CBITPlanner::discrete_collision(std::vector<std::vector<double>> obs, double discretization, Node start, Node end)
{
    // We dynamically determine the discretization based on the length of the edge
    discretization = ceil(calc_dist(start, end) * discretization);

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
    for (size_t i = 0; i < p_test.size(); i++)
    {
        Node curv_pt {p_test[i], q_test[i]};
        Node euclid_pt = curve_to_euclid(curv_pt);
        if (costmap_col_tight(euclid_pt))
        {
            return true;
        }
    }

    return false;
}
