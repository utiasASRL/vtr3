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


namespace {
// Function for converting Transformation matrices into se(2) [x, y, z, roll, pitch, yaw]
// Note we also might be able to do this with just lgmaths tran2vec operation?
inline std::tuple<double, double, double, double, double, double> T2xyzrpy(
    const vtr::tactic::EdgeTransform& T) {
  const auto Tm = T.matrix();
  return std::make_tuple(Tm(0, 3), Tm(1, 3), Tm(2,3), std::atan2(Tm(2, 1), Tm(2, 2)), std::atan2(-1.0*Tm(2, 0), sqrt(pow(Tm(2, 1),2) + pow(Tm(2, 2),2))), std::atan2(Tm(1, 0), Tm(0, 0)));
}
}

auto CBITPlanner::getChainInfo(vtr::path_planning::BasePathPlanner::RobotState& robot_state) -> ChainInfo {
  auto& chain = *robot_state.chain;
  auto lock = chain.guard();
  const auto stamp = chain.leaf_stamp();
  const auto w_p_r_in_r = chain.leaf_velocity();
  const auto T_p_r = chain.T_leaf_trunk().inverse();
  const auto T_w_p = chain.T_start_trunk();
  const auto curr_sid = chain.trunkSequenceId();
  return ChainInfo{stamp, w_p_r_in_r, T_p_r, T_w_p, curr_sid};
}

// Class Constructor:
CBITPlanner::CBITPlanner(CBITConfig conf_in, std::shared_ptr<CBITPath> path_in, vtr::path_planning::BasePathPlanner::RobotState& robot_state, std::shared_ptr<std::vector<Pose>> path_ptr, std::shared_ptr<CBITCostmap> costmap_ptr, std::shared_ptr<CBITCorridor> corridor_ptr, PathDirection path_direction)
{ 

  // Setting random seed
  srand((unsigned int)time(NULL));


  // Access the pointer to memory where the final result will be stored:
  cbit_path_ptr = path_ptr;

  // Store pointer to the costmap
  cbit_costmap_ptr = costmap_ptr;

  // Before beginning the planning phase, we need to wait for the robot to localize, and then update the goal state
  auto& chain = *robot_state.chain;
  do
  {
    CLOG(WARNING, "path_planning.cbit_planner") << "Robot is not localized, Planner is waiting to start";
  } while (chain.isLocalized() == 0);

  
  CLOG(INFO, "path_planning.cbit_planner") << "Planner is trying to initialize";
  conf = conf_in;
  global_path = path_in; //TODO: I dont really like this, maybe either used shared pointers or make the arg path_in just be named global path
  p_goal = std::make_shared<Node> (global_path->p[0], 0.0);
  p_goal_backup = p_goal; // store a backup of the goal for resets during repair mode
  p_start = std::make_shared<Node> (global_path->p.back(), 0.0);

  dynamic_window_width = conf.sliding_window_width;
  
  // DEBUG PLOTTING
  //initialize_plot();

  InitializePlanningSpace();
  Planning(robot_state, costmap_ptr, corridor_ptr, path_direction);

  // DEBUG CODE, ROBOT STATE UPDATE QUERY EXAMPLE
  /*
  // testing robot state query
  //auto& chain = *robot_state.chain; // Get the pointer to the robot state
  auto& chain = *robot_state.chain; // I think we actually only ever really use the chain to make sure we are actually localized
  // Which I guess I probably should be doing (see teb code) as we probably dont want to pass the robot state unless we are
  std::tuple<double, double, double, double, double, double> robot_pose;
  while (true)
  {
    const auto chain_info = getChainInfo(robot_state);
    auto [stamp, w_p_r_in_r, T_p_r, T_w_p, curr_sid] = chain_info;
    robot_pose= T2xyzrpy(T_w_p * T_p_r);
    CLOG(INFO, "path_planning.cbit_planner") << "Robot Pose: x: " << std::get<0>(robot_pose) << " y: " 
    << std::get<1>(robot_pose) << " z: " << std::get<2>(robot_pose) << " roll: " << std::get<3>(robot_pose) << " pitch: " 
    << std::get<4>(robot_pose) << " yaw: " << std::get<5>(robot_pose);
    CLOG(INFO, "path_planning.cbit_planner") << "Displaying Current Robot Transform: " << T_p_r;
    Planning();
    sleep(1);
  }

  CLOG(INFO, "path_planning.cbit_planner") << "Planner Executed successfully and found the optimal plan";
  */
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
  sample_box_height = conf.q_max * 2.0;
  sample_box_width = conf.sliding_window_width + 2 * conf.sliding_window_freespace_padding;

}

// Reset fuction which goes through the entire reset procedure (including calling ResetPlanner and restarting the planner itself)
void CBITPlanner::HardReset(vtr::path_planning::BasePathPlanner::RobotState& robot_state, std::shared_ptr<CBITCostmap> costmap_ptr, std::shared_ptr<CBITCorridor> corridor_ptr, PathDirection path_direction)
{
  // I think we may also want to add a small time delay just so we arent repeatedly spamming an optimal solution
  CLOG(ERROR, "path_planning.cbit_planner") << "Plan could not be improved, Initiating Reset";
  std::this_thread::sleep_for(std::chrono::milliseconds(100)); // short delay to prevent excessive looping

  //CLOG(ERROR, "path_planning.cbit_planner") << "P_goal_backup is: p: " << p_goal->p << " q: " << p_goal->q;
  //CLOG(ERROR, "path_planning.cbit_planner") << "P_goal backup is: p: " << p_goal_backup->p << " q: " << p_goal_backup->q;

  // Experimental, try a state update (this seems to work well)

  // First backup the goal pose incase we were in a repair (we actually should probably do this in the main loop too)
  p_goal = p_goal_backup;

  std::tuple<double, double, double, double, double, double> robot_pose;

  try
  {
    const auto chain_info = getChainInfo(robot_state);
    auto [stamp, w_p_r_in_r, T_p_r, T_w_p, curr_sid] = chain_info;

    robot_pose= T2xyzrpy(T_w_p * T_p_r);
    CLOG(INFO, "path_planning.cbit_planner") << "Robot Pose: x: " << std::get<0>(robot_pose) << " y: " 
    << std::get<1>(robot_pose) << " z: " << std::get<2>(robot_pose) << " roll: " << std::get<3>(robot_pose) << " pitch: " 
    << std::get<4>(robot_pose) << " yaw: " << std::get<5>(robot_pose);

    Pose se3_robot_pose = Pose(std::get<0>(robot_pose),(std::get<1>(robot_pose)),std::get<2>(robot_pose),std::get<3>(robot_pose),std::get<4>(robot_pose),std::get<5>(robot_pose));

    new_state = std::make_unique<Pose> (se3_robot_pose);

  }
  catch(...)
  {
    CLOG(ERROR, "path_planning.cbit_planner") << "Couldnt get robot state, must have reached end of path";
    return;
  }
  
  // Perform a state update to convert the actual robot position to its corresponding pq space:
  p_goal = UpdateState(path_direction);
  //p_goal = UpdateStateSID(curr_sid, T_p_r);


  p_goal_backup = p_goal;
  // There is a case where the robot is actually starting behind the world frame of the teach path (or the localization thinks it is at first anyways)
  // This results in a situation where the updated state p value is 0, and the q value is the euclidean distance from the world frame to the robot state
  // (Which is a significant overestimate of the q_min). I think easiest way to handle this situation is to basically ignore the state update until p is non-zero
  // So here what we do is check p, if it is equal to zero still, then set q to be zero as well.
  if (p_goal->p == 0.0)
  {
    CLOG(WARNING, "path_planning.cbit_planner") << "Current Robot State is behind the first vertex, ignoring state update";
    p_goal->q = 0.0;
  }

  //End of experimental




  // Reset the planning space
  ResetPlanner(); //TODO: Consider compressing the above state update and putting it inside ResetPlanner()
  InitializePlanningSpace();
  CLOG(INFO, "path_planning.cbit_planner") << "The p,q coordinate of the robots goal is now: p: " << p_goal->p << " q: " << p_goal->q;
  CLOG(INFO, "path_planning.cbit_planner") << "The p,q coordinate of the robots start is now: p: " << p_start->p << " q: " << p_start->q << " g_T: " << p_start->g_T;

  Planning(robot_state, costmap_ptr, corridor_ptr, path_direction);
}

// If we ever exit the planner due to a fault, we will do a hard reset, everything but the current robot_state (p_goal) and the inputs will be reinitialized
void CBITPlanner::ResetPlanner()
{
  tree.V.clear();
  tree.E.clear();
  //tree.QV.clear();
  tree.QV2.clear();
  //tree.QE.clear();
  tree.QE2.clear();
  tree.V_Repair_Backup.clear();
  tree.V_Old.clear();
  samples.clear();
  path_x.clear();
  path_y.clear();

  dynamic_window_width = conf.sliding_window_width;
  repair_mode = false;

  // first the goal needs to be backed up because if we restart in repair mode, the p_goal will be wrong
  //p_goal = p_goal_backup;
  //p_goal = std::make_shared<Node> (*p_goal_backup);

  // Make sure to clear the goal parent or else the first batch will think it immediately concluded and it will double sample
  //p_goal->parent = nullptr;
  //p_goal->g_T = INFINITY;
  //p_goal->g_T_weighted;

}


// Main planning function
void CBITPlanner::Planning(vtr::path_planning::BasePathPlanner::RobotState& robot_state, std::shared_ptr<CBITCostmap> costmap_ptr, std::shared_ptr<CBITCorridor> corridor_ptr, PathDirection path_direction)
{
  // find some better places to initialize these
  double prev_path_cost = INFINITY; // experimental
  int compute_time = 0; // exprimental
  int vertex_rej_prob = 100; // experimental
  int dyn_batch_samples = conf.batch_samples;

  // Grab the amount of time in ms between robot state updates
  int control_period_ms = (1.0 / conf.state_update_freq) * 1000.0;
  auto state_update_time = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(control_period_ms);

  repair_mode = false; // If we need to reset the planner, then we should also reset the repair mode here as well

  // benchmarking example code
  auto start_time = std::chrono::high_resolution_clock::now();

  // Variables for rolling average compute time for debug
  int batches_completed = 0;
  auto average_batch_time = 0.0;


  bool localization_flag = true; // Set the fact we are localized if we make it to this point

  for (int k = 0; k < conf.iter_max; k++)
  {
    // Debug, each iteration lets display the current costmap info, make sure it updates (as far as I can tell I think it is)
    //CLOG(DEBUG, "path_planning.cbit_planner") << "Iteration: " << k;
    //CLOG(DEBUG, "path_planning.cbit_planner") << "Now that we made the pointers, try to display them: " << costmap_ptr->obs_map_ptr;
    //CLOG(DEBUG, "path_planning.cbit_planner") << "Now that we made the pointers, try to display them: " << costmap_ptr->T_r_costmap_ptr;


    // Collision checking debug example:
    // Create 4x1 vector for the point to collision check:
    //Eigen::Matrix<double, 4, 1> test_pt({0.25, -3.25, 0, 1});
    //auto collision_pt = costmap_ptr->T_c_w * test_pt;
    //CLOG(DEBUG, "path_planning.cbit_planner") << "Displaying the point in the costmap frame we are trying to collision check: " << collision_pt;

    //DEBUG of new collision code:
    //Node test_pt = Node(-0.25, 3.25, 0.0);
    //costmap_col(test_pt);


    // Check whether a robot state update should be applied
    // We only update the state if A: we have first found a valid initial solution, and B: if the current time has elapsed the control period
    if (conf.update_state == true)
    {
      if ((p_goal->parent != nullptr) && (std::chrono::high_resolution_clock::now() >= state_update_time))
      {
        // Update timers
        //CLOG(INFO, "path_planning.cbit_planner") << "Attempting to Update Robot State";
        state_update_time = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(control_period_ms);

        // get the euclidean robot state in the world frame from vt&r
        auto& chain = *robot_state.chain;
          if (chain.isLocalized() == 0)
          {
          // If we ever become unlocalized, I think we just need to break, then set a flag to exit the outter loop
          // This also triggers after we reach end of path and effectively shuts down the planner
          bool localization_flag = false;
          CLOG(ERROR, "path_planning.cbit_planner") << "Localization was Lost, Exiting Inner Planning Loop";
          break;
          }
        
        std::tuple<double, double, double, double, double, double> robot_pose;

        const auto chain_info = getChainInfo(robot_state);
        auto [stamp, w_p_r_in_r, T_p_r, T_w_p, curr_sid] = chain_info;
        //CLOG(ERROR, "path_planning.cbit_planner") << "The T_p_r for robot state is: " << T_p_r;

        // Experimental Pose extrapolation (I find often this doesnt work well with te direct path tracking method)
        // It can be desireable to delay pruning the path behind where we think the robot is

        if (conf.extrapolation == true)
        {
          const auto curr_time = stamp + (1.0e9 / conf.state_update_freq);
          const auto dt = static_cast<double>(curr_time - stamp) * 1e-9;
          //CLOG(INFO, "path_planning.cbit_planner") << "current time is: " << curr_time; 
          //CLOG(INFO, "path_planning.cbit_planner") << "stamp is: " << stamp; 
          //CLOG(INFO, "path_planning.cbit_planner") << "dt is: " << dt ; 

          const auto T_p_r_extp = [&]() {
            //if (!config_->extrapolate) return T_p_r; // Config param for extrapolation
            // extrapolate based on current time
            Eigen::Matrix<double, 6, 1> xi_p_r_in_r(dt * w_p_r_in_r);
            return T_p_r * vtr::tactic::EdgeTransform(xi_p_r_in_r).inverse();
          }();

          robot_pose = T2xyzrpy(T_w_p * T_p_r_extp);
          //CLOG(INFO, "path_planning.cbit_planner") << "Extrapolated Robot Pose: x: " << std::get<0>(robot_pose) << " y: " 
          //<< std::get<1>(robot_pose) << " z: " << std::get<2>(robot_pose) << " roll: " << std::get<3>(robot_pose) << " pitch: " 
          //<< std::get<4>(robot_pose) << " yaw: " << std::get<5>(robot_pose);
        }
        else
        {
          robot_pose= T2xyzrpy(T_w_p * T_p_r);

          // experimental, finding the direction the robot is facing (planing forward or in reverse)
          //debug
          //auto test_pose = T2xyzrpy(T_p_r);
          //CLOG(ERROR, "path_planning.cbit_planner") << "Robot Pose in planing frame yaw: " << std::get<5>(test_pose);
        }


        //End Experimental Pose Interpolation

        //CLOG(INFO, "path_planning.cbit_planner") << "Displaying Current Robot Transform: " << T_p_r;

        Pose se3_robot_pose = Pose(std::get<0>(robot_pose),(std::get<1>(robot_pose)),std::get<2>(robot_pose),std::get<3>(robot_pose),std::get<4>(robot_pose),std::get<5>(robot_pose));


        new_state = std::make_unique<Pose> (se3_robot_pose);

        // Perform a state update to convert the actual robot position to its corresponding pq space:
        //p_goal = UpdateState(path_direction);
        //CLOG(ERROR, "path_planning.cbit_planner") << "The goal from old method is: " << " p: " << p_goal->p << " q: " << p_goal->q;
        p_goal = UpdateStateSID(curr_sid, T_p_r);
        //CLOG(ERROR, "path_planning.cbit_planner") << "The goal from new method is: " << " p: " << p_goal->p << " q: " << p_goal->q;

        //TODO: It could be useful to convert this p_goal back to euclid and compare with se3_robot_pose to verify the conversion worked properly (error should be very low)
        // If its not, we probably need to handle it or return an error


        // EXPERIMENTAL TODO: If robot pose nears the goal (p_start), exit the planner with the current solution
        // Note, not sure if this is super easy, because I think if the planner exits the vt&r code will immediately relaunch it
        //CLOG(ERROR, "path_planning") << "Trying the distance to goal is: " << abs(p_goal->p - p_start->p);
        //if (abs(p_goal->p - p_start->p) <= 2.0)
        //{
        //  CLOG(ERROR, "path_planning") << "Trying to exit the CBIT Planner Initialize route";
        //  return;
        //}

        // If we arent in repair mode, update the backup goal
      
        

        // There is a case where the robot is actually starting behind the world frame of the teach path (or the localization thinks it is at first anyways)
        // This results in a situation where the updated state p value is 0, and the q value is the euclidean distance from the world frame to the robot state
        // (Which is a significant overestimate of the q_min). I think easiest way to handle this situation is to basically ignore the state update until p is non-zero
        // So here what we do is check p, if it is equal to zero still, then set q to be zero as well.
        if (p_goal->p == 0.0)
        {
          CLOG(WARNING, "path_planning.cbit_planner") << "Current Robot State is behind the first vertex, ignoring state update";
          p_goal->q = 0.0;
        }


        // Debug, check to see what p value we are spitting out
        //CLOG(INFO, "path_planning.cbit_planner") << "The p,q coordinate of the robots updated state goal is now: p: " << p_goal->p << " q: " << p_goal->q;


        // Add the new goal (robot state) to the samples so it can be found again
        // EXPERIMENTAL - want to try flushing all the samples just before pushing the goal, then resampling locally in a radius around the new goal.
        // It probably makes sense that the sample density in the radius ring is based on the rgg radius as well
        //samples.clear();
        samples.push_back(p_goal); 
        //double r_s;
        //double theta_s;
        //double x_s;
        //double y_s;
        //for (int i = 0; i < 100; i++)
        //{
        //  r_s = 2.0 * sqrt(static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) ; // todo replace magic number 2.0 in this one (not the next)
        //  theta_s = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * 2.0 * 3.1415;
        //  x_s = r_s * cos(theta_s) + p_goal->p;
        //  y_s = r_s * sin(theta_s) + p_goal->q;
        //  Node new_sample(x_s, y_s);
        //  samples.push_back(std::make_shared<Node> (new_sample));
        //  CLOG(DEBUG, "path_planning.cbit_planner") << "Sample x: " << x_s << " Sample y: " << y_s;
        //}

        // Find vertices in the tree which are close to the new state, then populate the vertex queue with only these values.
        //tree.QV.clear();
        tree.QV2.clear();
        //tree.QE.clear();
        tree.QE2.clear();

        // EXPERIMENTAL, only take the closest point in the tree and use this to connect to the new robot state
        /*
        //int nearby_sample_counter = 0;
        double closest_sample_dist = INFINITY;
        int closest_sample_ind;
        double sample_dist;
        for (int i = 0; i < tree.V.size(); i++)
        {
          sample_dist = calc_dist(*(tree.V[i]), *p_goal);
          if ((sample_dist <= closest_sample_dist) && ((tree.V[i])->p > (p_goal->p + (conf.initial_exp_rad/2)))) // TODO: replace magic number with a param, represents radius to search for state update rewires
          {
            closest_sample_dist = sample_dist; // used when only taking closest point in the tree to consider
            closest_sample_ind = i; // used when only taking closest point in the tree to consider
            //tree.QV.push_back(tree.V[i]); // comment out when only taking closest point in the tree to consider
          }
        }
        tree.QV.push_back(tree.V[closest_sample_ind]); // used when only taking closest point in the tree to consider
        */

       // Alternative: Take all points in a radius of 2.0m around the new robot state (only if they are a ahead though)
        double sample_dist;
        for (int i = 0; i < tree.V.size(); i++)
        {
          sample_dist = calc_dist(*(tree.V[i]), *p_goal);
          if ((sample_dist <= 2.0) && ((tree.V[i])->p > (p_goal->p + (conf.initial_exp_rad/2)))) // TODO: replace magic number with a param, represents radius to search for state update rewires
          {
            //tree.QV.push_back(tree.V[i]); // comment out when only taking closest point in the tree to consider
            tree.QV2.insert(std::pair<double, std::shared_ptr<Node>>((tree.V[i]->g_T_weighted + h_estimated_admissible(*tree.V[i], *p_goal)), tree.V[i]));
          }
        }




        CLOG(DEBUG, "path_planning.cbit_planner") << "Robot State Updated Successfully, p: " << p_goal->p << " q: " << p_goal->q;
        //CLOG(DEBUG, "path_planning.cbit_planner") << "Nearest Vertex, p: " << tree.V[closest_sample_ind]->p << " q: " << tree.V[closest_sample_ind]->q;
        //CLOG(DEBUG, "path_planning.cbit_planner") << "QV size: " << tree.QV.size();
        CLOG(DEBUG, "path_planning.cbit_planner") << "QV size: " << tree.QV2.size();

        // When the planner resumes, this will cause it to immediately try to rewire locally to the new robot state in a short amount of time

      }
    }


    int m;
    //if (tree.QV.size() == 0 && tree.QE.size() == 0)
    if (tree.QV2.size() == 0 && tree.QE2.size() == 0)
    {
      //std::cout << "New Batch:" << std::endl;
      if (k == 0)
      {
        m = conf.initial_samples;
      }
      else
      {
        m = conf.batch_samples;
      }

      // Only run this code if we have reached the end of a batch and have a new solution
      // (displays results)
      
      if (p_goal->parent != nullptr)
      {
        //std::cout << "Iteration: " << k << std::endl;
        //std::cout << "Path Cost: " << p_goal->g_T_weighted << std::endl;

        // Benchmark current compute time
        auto stop_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time);
        //std::cout << "Batch Compute Time (ms): " << duration.count() << std::endl;
        batches_completed = batches_completed + 1;
        average_batch_time = ((batches_completed * average_batch_time) + duration.count()) / (batches_completed + 1);
        //std::cout << "Average Batch Compute Time (ms): " << average_batch_time << std::endl;
        

        compute_time = static_cast <int>(duration.count());

        // Debug message for online use (wont use std::out long term)
        CLOG(INFO, "path_planning.cbit_planner") << "New Batch - Iteration: " << k << "   Path Cost: " << p_goal->g_T_weighted << "   Batch Compute Time (ms): " << duration.count();
        CLOG(INFO, "path_planning.cbit_planner") << "Tree Vertex Size: " << tree.V.size() << " Tree Edge Size: " << tree.E.size() << " Sample Size: " << samples.size();

        // If the solution does not improve, backup the tree to restrict unnecessary growth
        if (abs(p_goal->g_T_weighted - prev_path_cost) < 1e-6)
        {
          CLOG(DEBUG, "path_planning.cbit_planner") << "There Was No Path Improvement, Restoring Tree to Previous Batch To Restrict Growth.";
          tree.V = tree.V_Old;
          tree.E = tree.E_Old;
        }
        prev_path_cost = p_goal->g_T_weighted;

        // EXPERIMENTAL
        // if the batch compute time starts to creep up, it means the tree is getting abit to bloated
        // Its likely that more samples will not really help us much while going around obstacles, so we should reduce the batch size
        if (compute_time >= 200)
        {
          if (vertex_rej_prob > 60) // never want to exceed some minimums
          {
            dyn_batch_samples = dyn_batch_samples - 10;
            // Also increase the probability of rejecting vertex queue vertices to improve performance
            vertex_rej_prob = vertex_rej_prob - 10;
            CLOG(DEBUG, "path_planning.cbit_planner") << "Compute Falling Behind, Reducing Batch Size To " << dyn_batch_samples << " and Decreasing Vertex Acceptance Probability to " << vertex_rej_prob << "%";
          }
        }
        // Conversely if batch compute time is fast, we can handle more samples (up to a max)
        else
        {
          if (vertex_rej_prob < 100)
          {
            vertex_rej_prob = vertex_rej_prob + 2;
            dyn_batch_samples = dyn_batch_samples + 2;
            CLOG(DEBUG, "path_planning.cbit_planner") << "Compute Catching Up, Increasing Batch Size To " << dyn_batch_samples << " and Decreasing Vertex Acceptance Probability to " << vertex_rej_prob << "%";
          }
        }
        m = dyn_batch_samples;

        // Extract the solution
        //std::cout << "Made it just before extractpath" << std::endl;
        std::tuple<std::vector<double>, std::vector<double>> curv_path = ExtractPath(robot_state, costmap_ptr);
        path_x = std::get<0>(curv_path); // p coordinates of the current path (I should probably rename, this is misleading its not x and y)
        path_y = std::get<1>(curv_path); // q coordinates of the current path (I should probably rename, this is misleading its not x and y)


        // Store the Euclidean solution in the shared pointer memory (vector of Pose classes) so it can be accessed in the CBIT class
        //std::cout << "Made it just before extracting euclid path" << std::endl;
        std::vector<Pose> euclid_path = ExtractEuclidPath();
        //*cbit_path_ptr = euclid_path; // previously I update the cbit path ptr right away, but I think that because there is quite abit of delay in the corridor update process, I should wait to update them at the same time
        // This helps prevent synchronization issues

        // Reset the start time
        start_time = std::chrono::high_resolution_clock::now();
        //std::cout << "Made it to end of new batch process code" << std::endl;

        //DEBUG PLOTTING
        //auto plot_start_time = std::chrono::high_resolution_clock::now();
        //plot_tree(tree, *p_goal, path_x, path_y, samples);
        //auto plot_stop_time = std::chrono::high_resolution_clock::now();
        //auto duration_plot = std::chrono::duration_cast<std::chrono::milliseconds>(plot_stop_time - plot_start_time);
        //CLOG(ERROR, "path_planning.cbit_planner") << "Plot Time: " << duration_plot.count() << "ms";
        


  
        // First grab the latest obstacles (could potentially take this opportunity to make a more compact approximate obs representation)
        
        // Collision Check the batch solution: TODO:, will need to make wormhole modifcations long term
        std::shared_ptr<Node> col_free_vertex = col_check_path_v2((p_goal->p + conf.sliding_window_width + conf.sliding_window_freespace_padding)); // outputs NULL if no collision
        if (col_free_vertex != nullptr)
        {
          // If there is a collision, prune the tree of all vertices to the left of the this vertex
          CLOG(WARNING, "path_planning.cbit_planner") << "Collision Detected:";
          CLOG(WARNING, "path_planning.cbit_planner") << "Collision Free Vertex is - p: " << col_free_vertex->p << " q: " << col_free_vertex->q;

          
          // Vertex Prune (maintain only vertices to the right of the collision free vertex)
          std::vector<std::shared_ptr<Node>> pruned_vertex_tree;
          pruned_vertex_tree.reserve(tree.V.size());
          int test_counter = 0;
          for (int i =0; i<tree.V.size(); i++)
          {
            if (tree.V[i]->p >= col_free_vertex->p)
            {
              pruned_vertex_tree.push_back(tree.V[i]);
            }
            if (tree.V[i]->p >= (p_goal->p + 11.0))
            {
              test_counter = test_counter + 1;
            }
          }
          tree.V = pruned_vertex_tree;
          CLOG(ERROR, "path_planning.cbit_planner") << "The number of vertices in the tree outside of sliding window is:" << test_counter;
          CLOG(ERROR, "path_planning.cbit_planner") << "The corresponding p threshold was: " << (p_goal->p + 11.0);


          // Edge Prune (maintain only edges to the right of the collision free vertex)
          std::vector<std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>  pruned_edge_tree;
          pruned_edge_tree.reserve(tree.E.size());
          for (int i = 0; i <tree.E.size(); i++)
          {
            if (std::get<1>(tree.E[i])->p >= col_free_vertex->p)
            {
              pruned_edge_tree.push_back(tree.E[i]);
            }
          }
  
          tree.E = pruned_edge_tree;
                 

          // Experimental, just trying to see whether not restoring at all yields better repair performance time
          //tree.V.clear();
          //tree.V_Repair_Backup.clear();
          //tree.V_Old.clear()
          //tree.E.clear();
          //tree.V.push_back(std::shared_ptr<Node> (p_start));
          // End of experimental

          // Reset the goal, and add it to the samples
          p_goal->parent = nullptr;
          p_goal->g_T = INFINITY;
          p_goal->g_T_weighted = INFINITY;
          samples.clear(); // first clear samples so they dont accumulate excessively
          samples.push_back(p_goal);

          // This will cause samplefreespace to run this iteration, but if we made it here m will be set to conf.batch_samples (as its reset every iteration)
          m = conf.initial_samples;  
          //k=0;

          // Re-initialize sliding window dimensions for plotting and radius expansion calc;
          sample_box_height = conf.q_max * 2.0;
          sample_box_width = conf.sliding_window_width + 2 * conf.sliding_window_freespace_padding;

          // Disable pre-seeds from being regenerated
          repair_mode = true;

          // Sample free-space wont generate pre-seeds, so we should generate our own in the portion of the tree that was dropped (col_free_vertex.p to p_goal.p)
          int pre_seeds = abs(col_free_vertex->p - p_goal->p) / 0.25; // Note needed to change p_goal to p_zero. When the sliding window padding is large, pre-seeds wont get generated all the way to the goal
          //int pre_seeds = abs(p_start->p - p_goal->p) / 0.25; // EXPERIMENTAL< DONT FORGET TO REMOVE THIS AND UNCOMMENT ABOVE IF NOT USING THE FULL RESET METHOD
          double p_step = 0.25;
          double p_val = p_goal->p;
          for (int i = 0; i < (pre_seeds-1); i++) 
          {
            Node node((p_val+p_step), 0);
            samples.push_back(std::make_shared<Node> (node));
            p_val = p_val + p_step;
          }

        }
        else
        {
          // Reset the free space flag in sample freespace
          //CLOG(WARNING, "path_planning.cbit_planner") << "No Collision:";
          repair_mode = false;


          // EXPERIMENTAL: Updating the dynamic corridor (now that we know our path is collision free):
          //auto corridor_start_time = std::chrono::high_resolution_clock::now();
          //update_corridor(corridor_ptr, path_x, path_y, *p_goal);
          //auto corridor_stop_time = std::chrono::high_resolution_clock::now();
          //auto duration_corridor = std::chrono::duration_cast<std::chrono::microseconds>(corridor_stop_time - corridor_start_time);
          //CLOG(ERROR, "path_planning.cbit_planner") << "Corridor Update Time: " << duration_corridor.count() << "us";
        }
        
        *cbit_path_ptr = euclid_path; // Update the pointer to point to the latest euclidean path update
        
      }
      //std::cout << "Made it just before prune" << std::endl;
      Prune(p_goal->g_T, p_goal->g_T_weighted);

      // Resample
      if (p_goal->g_T_weighted < INFINITY)
      {
        // sample box function here
        //std::cout << "Sample Box" << std::endl;
        std::vector<std::shared_ptr<Node>> new_samples = SampleBox(m); // TODO Sample rejection
        //std::vector<std::shared_ptr<Node>> new_samples = SampleFreeSpace(m); // DEBUG ONLY!!!
        samples.insert(samples.end(), new_samples.begin(), new_samples.end());

      }
      
      else
      {
        //std::cout << "Sample Free Space " << std::endl;
        std::vector<std::shared_ptr<Node>> new_samples = SampleFreeSpace(m); // TODO Pre Seeds, Sample Rejection
        samples.insert(samples.end(), new_samples.begin(), new_samples.end());


      }

    
      // Backup the old tree:
      tree.V_Old.clear();
      tree.E_Old.clear();
      for (int i = 0; i < tree.V.size(); i++)
      {
        tree.V_Old.push_back((tree.V[i]));
      }

      for (int i = 0; i < tree.E.size(); i++)
      {
        tree.E_Old.push_back(tree.E[i]);
      }


      // Initialize the Vertex Queue with all nearby vertices in the tree;
      //tree.QV = tree.V; // C++ is dumb so this doesnt work with shared pointer vectors, need to do the following instead    
      //bool reject_vertex = false; 
      std::random_device rd;     // Only used once to initialise (seed) engine
      std::mt19937 rng(rd());    // Random-number engine used (Mersenne-Twister in this case)
      std::uniform_int_distribution<int> uni(1,100); // Guaranteed unbiased 
      //CLOG(ERROR, "path_planning.cbit_planner") << "Vertex Lookahead: "<< p_goal->p + dynamic_window_width + conf.sliding_window_freespace_padding+5;
      for (int i = 0; i < tree.V.size(); i++)
      {
        // TODO: It may also be worth considering only adding a random subset of the vertex tree to the queue in times when batches are taking a long time
        //if (compute_time >= 200)
        //{
        auto random_integer = uni(rng);
        random_integer = static_cast <double>(random_integer);
        //reject_vertex = static_cast <bool>(random_integer);
        //CLOG(DEBUG, "path_planning.cbit_planner") << "Compute Falling Behind, Vertices, reject vertex: " << reject_vertex;
        //}
        //else
        //{
        //  reject_vertex = false;
        //}

        // if we have no 1st batch solution (we are in the first iteration or have just reset), add the whole tree to the queue
        if (k == 0)
        {
          //tree.QV.push_back(tree.V[i]);
          tree.QV2.insert(std::pair<double, std::shared_ptr<Node>>((tree.V[i]->g_T_weighted + h_estimated_admissible(*tree.V[i], *p_goal)), tree.V[i]));
        }
        // Otherwise, only add the portions of the tree within the sliding window to avoid processing preseeded vertices which are already optimal
        //else if (((tree.V[i]->p) <= p_goal->p + dynamic_window_width + (conf.sliding_window_freespace_padding*2)) && ((vertex_rej_prob / random_integer) >= 1.0))
        else if (((vertex_rej_prob / random_integer) >= 1.0)) // for some reason using the lookahead queue doesnt work reliably for collisions, not sure why, need to investigate
        {
          //tree.QV.push_back(tree.V[i]);
          tree.QV2.insert(std::pair<double, std::shared_ptr<Node>>((tree.V[i]->g_T_weighted + h_estimated_admissible(*tree.V[i], *p_goal)), tree.V[i]));
        }

      }
      //CLOG(DEBUG, "path_planning.cbit_planner") << "QV size after batch: " << tree.QV.size();
      CLOG(DEBUG, "path_planning.cbit_planner") << "QV size after batch: " << tree.QV2.size();

      // TODO: Dynamic Expansion radius selection:
      // Its debatable whether this works well or not, but going to try it
      
      //if (p_goal->g_T != INFINITY)
      //{
      //  conf.initial_exp_rad = exp_radius((tree.V.size() + samples.size()), sample_box_height, sample_box_width, conf.eta);
      //  CLOG(DEBUG, "path_planning.cbit_planner") << "New expansion radius is: " << conf.initial_exp_rad;
      //}
      
      
    }

    auto start_bench_time = std::chrono::high_resolution_clock::now();




    // Planning loop starts here
    // I think there is some gains to be had here, we actually iterate through the vertex queue twice finding minima,
    // I think we probably could combine the 2 vertex functions into 1 cutting compute in half (we can test to see if this is a big gain or not)

    while (BestVertexQueueValue() < BestEdgeQueueValue())
    {
      //auto test = BestInVertexQueue();
      ExpandVertex(BestInVertexQueue());
    }

    auto stop_bench_time = std::chrono::high_resolution_clock::now();
    auto duration_bench = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_bench_time - start_bench_time);
    CLOG(DEBUG, "path_planning_compute.cbit") << "Compute 1: " << duration_bench.count();
    start_bench_time = std::chrono::high_resolution_clock::now();



    
    // Generate prospective nodes
    std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> prospective_edge = BestInEdgeQueue();

    std::shared_ptr<Node> vm = std::get<0>(prospective_edge);
    std::shared_ptr<Node> xm = std::get<1>(prospective_edge);

    // TODO: Handle edge case where a perfect path results in empty queues (see python code)
    if (vm == NULL) 
    {
      // If we just continue here, we enter a state where after every single iteration we will complete a batch, but I think this is fine
      // It will just stay this way until we move off the path and have a non perfect path solution
      continue;
    }
    // Remove the edge from the queue (I think best to do this inside BestInEdgeQueue function)



    stop_bench_time = std::chrono::high_resolution_clock::now();
    duration_bench = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_bench_time - start_bench_time);
    CLOG(INFO, "path_planning_compute.cbit") << "Compute 2: " << duration_bench.count();
    start_bench_time = std::chrono::high_resolution_clock::now();

    // TODO: Collision check and update tree/queues
    if (vm->g_T_weighted + calc_weighted_dist(*vm, *xm, conf.alpha) + h_estimated_admissible(*xm, *p_goal) < p_goal->g_T_weighted)
    {
      // TODO: replace these next too lines with a function which first does a collision check, returning INF if there is one
      // Then runs the calc_dist functions if there is not a collision
      //double actual_cost = calc_dist(*vm, *xm);
      //double weighted_cost = calc_weighted_dist(*vm, *xm, conf.alpha);

      double actual_cost = cost_col(obs_rectangle, *vm, *xm);
      double weighted_cost = weighted_cost_col(obs_rectangle, *vm, *xm);

      stop_bench_time = std::chrono::high_resolution_clock::now();
      duration_bench = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_bench_time - start_bench_time);
      CLOG(INFO, "path_planning_compute.cbit") << "Compute 3: " << duration_bench.count();
      start_bench_time = std::chrono::high_resolution_clock::now();

      if (g_estimated_admissible(*vm, *p_start) + weighted_cost + h_estimated_admissible(*xm, *p_goal) < p_goal->g_T_weighted)
      {
        if (vm->g_T_weighted + weighted_cost < xm->g_T_weighted)
        {
          // Check if xm is in the tree, if it is, we need to do some rewiring of any other edge which has xm as an endpoint
          if (node_in_tree_v2(xm) == true)
          {
            // TODO: Needs some additional wormhole logic in here which I havent done yet

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
            // TODO: wormhole handling

          }
          
          // Otherwise we can remove xm from the samples, add xm to the tree vertices and queue
          else
          {
            // If the end point is not in the tree, it must have come from a random sample.
            // Remove it from the samples, add it to the vertex tree and vertex queue:
            for (int i = 0; i < samples.size(); i++)
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
            // TODO: wormhole handling

            tree.V.push_back(xm);
            //tree.QV.push_back(xm);
            tree.QV2.insert(std::pair<double, std::shared_ptr<Node>>(xm->g_T_weighted + h_estimated_admissible(*xm, *p_goal), xm));
          }
          stop_bench_time = std::chrono::high_resolution_clock::now();
          duration_bench = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_bench_time - start_bench_time);
          CLOG(INFO, "path_planning_compute.cbit") << "Compute 3.5: " << duration_bench.count();
          start_bench_time = std::chrono::high_resolution_clock::now();

          // Generate edge, create parent chain
          tree.E.push_back(std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {vm, xm});
        
          /*
          // Filter edge queue as now any other edge with worse cost heuristic can be ignored
          int upper_queue_index = tree.QE.size();
          for (int i = 0; i < upper_queue_index; i++)
          {
            Node v = *std::get<0>(tree.QE[i]);
            std::shared_ptr<Node> x = std::get<1>(tree.QE[i]);

            if ((x == xm) && (v.g_T_weighted + calc_weighted_dist(v, *xm, conf.alpha) >= xm->g_T_weighted))
            {
              auto it = tree.QE.begin() + i;
              *it = std::move(tree.QE.back());
              tree.QE.pop_back();

              // Once again, be very careful with the iteration indices when deleting
              // Need to decrement both the iterator and the total size
              i = i - 1;
              upper_queue_index = upper_queue_index - 1;
            }
          }
          */


          // Variation using multimap edge queue. In this case we kind of need to do the same procedure unfortunately (slow)
          // Only here we have to use iterators to go through the queue
          // NOTE THIS NEEDS SOME VERY GOOD DEBUGGING BEFORE TRUSTING THIS CODE, NOT SURE IF ITERATORS BEHAVE THE SAME WAY AS INDEXING WITH DELETION
          // For iterators, we need to do this process in a while loop because we cannot just decrement the index like before (leads to bad pointer)
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
          CLOG(DEBUG, "path_planning_compute.cbit") << "Compute 3.75: " << duration_bench.count();
          start_bench_time = std::chrono::high_resolution_clock::now();
        }
      }
      stop_bench_time = std::chrono::high_resolution_clock::now();
      duration_bench = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_bench_time - start_bench_time);
      CLOG(DEBUG, "path_planning_compute.cbit") << "Compute 4: " << duration_bench.count();
    }
    // If there is no edges in the queue which can improve the current tree, clear the queues which will consequently trigger the end of a batch
    else
    {
      //tree.QV.clear();
      tree.QV2.clear();
      //tree.QE.clear();
      tree.QE2.clear();
    }

    // TODO: Plotting (if we want)
    
    
  } // End of main planning for loop
  

  // Exiting cleanly:
  // If we make it here and are no longer localized, we've reached end of path and should clear the bitstar path from memory
  if (localization_flag == false)
  {
    CLOG(ERROR, "path_planning.cbit_planner") << "Reached End of Plan, Exiting cleanly";
    (*cbit_path_ptr).clear();
    
  }
  

  // FAULT AND EARLY MAIN LOOP EXIT HANDLING

  // If we make it here and are still localized, then try to reset the planner
  //if (localization_flag == true)
  //{
    //HardReset(robot_state, costmap_ptr);







    // The below code was all wrapped into HardReset, still experimental however.
    /*
    // I think we may also want to add a small time delay just so we arent repeatedly spamming an optimal solution
    CLOG(ERROR, "path_planning.cbit_planner") << "Plan could not be improved, Initiating Reset";
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // short delay to prevent excessive looping

    CLOG(ERROR, "path_planning.cbit_planner") << "P_goal_backup is: p: " << p_goal->p << " q: " << p_goal->q;
    CLOG(ERROR, "path_planning.cbit_planner") << "P_goal backup is: p: " << p_goal_backup->p << " q: " << p_goal_backup->q;

    // Experimental, try a state update (this seems to work well)

    // First backup the goal pose incase we were in a repair (we actually should probably do this in the main loop too)
    p_goal = p_goal_backup;

    std::tuple<double, double, double, double, double, double> robot_pose;

    const auto chain_info = getChainInfo(robot_state);
    auto [stamp, w_p_r_in_r, T_p_r, T_w_p, curr_sid] = chain_info;

    robot_pose= T2xyzrpy(T_w_p * T_p_r);
    CLOG(INFO, "path_planning.cbit_planner") << "Robot Pose: x: " << std::get<0>(robot_pose) << " y: " 
    << std::get<1>(robot_pose) << " z: " << std::get<2>(robot_pose) << " roll: " << std::get<3>(robot_pose) << " pitch: " 
    << std::get<4>(robot_pose) << " yaw: " << std::get<5>(robot_pose);

    Pose se3_robot_pose = Pose(std::get<0>(robot_pose),(std::get<1>(robot_pose)),std::get<2>(robot_pose),std::get<3>(robot_pose),std::get<4>(robot_pose),std::get<5>(robot_pose));

    new_state = std::make_unique<Pose> (se3_robot_pose);

    // Perform a state update to convert the actual robot position to its corresponding pq space:
    p_goal = UpdateState(path_direction);


    p_goal_backup = p_goal;
    // There is a case where the robot is actually starting behind the world frame of the teach path (or the localization thinks it is at first anyways)
    // This results in a situation where the updated state p value is 0, and the q value is the euclidean distance from the world frame to the robot state
    // (Which is a significant overestimate of the q_min). I think easiest way to handle this situation is to basically ignore the state update until p is non-zero
    // So here what we do is check p, if it is equal to zero still, then set q to be zero as well.
    if (p_goal->p == 0.0)
    {
      CLOG(WARNING, "path_planning.cbit_planner") << "Current Robot State is behind the first vertex, ignoring state update";
      p_goal->q = 0.0;
    }

    //End of experimental




    // Reset the planning space
    ResetPlanner(); //TODO: Consider compressing the above state update and putting it inside ResetPlanner()
    InitializePlanningSpace();
    CLOG(INFO, "path_planning.cbit_planner") << "The p,q coordinate of the robots goal is now: p: " << p_goal->p << " q: " << p_goal->q;
    CLOG(INFO, "path_planning.cbit_planner") << "The p,q coordinate of the robots start is now: p: " << p_start->p << " q: " << p_start->q << " g_T: " << p_start->g_T;

    Planning(robot_state, costmap_ptr, path_direction);
    */




  //}
  //else
  //{
    // But if we arent localized, then there is really nothing else we can do but return an error and end the planning session
  //  CLOG(ERROR, "path_planning.cbit_planner") << "Localization Failed, Terminating the Planning Instance";
  //}

} // End of main planning function



std::vector<std::shared_ptr<Node>> CBITPlanner::SampleBox(int m)
{

  //TODO
  // Create a vector to store the pointers to the new samples we are going to generate.
  std::vector<std::shared_ptr<Node>> new_samples;

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
  if (q_max < 0.5) // TODO make this config param
  {
    q_max = 0.5;
  }

  double p_max = p_goal->p + lookahead + padding;
  double p_zero = p_goal->p - padding;

  //std::cout << "p_max: " << p_max << std::endl;
  //std::cout << "p_zero: " << p_zero << std::endl;
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

    // TODO: Before we add the sample to the sample vector, we need to collision check it
    //if (is_inside_obs(obs_rectangle, curve_to_euclid(node))) // legacy version with obstacle vectors
    if (costmap_col(curve_to_euclid(node))) // Using costmap for collision check

    {
      continue;
    }
    else
    {
      new_samples.push_back(std::make_shared<Node> (node));
      ind++;
    }
  }

  // TODO: Generating Pre-seeds (Although I think actually in the current version I dont add additional pre-seeds in the sliding window)
  // I think practically I shouldnt have to and it would slow things down alot.

  return new_samples;
}



std::vector<std::shared_ptr<Node>> CBITPlanner::SampleFreeSpace(int m)
{
  std::vector<std::shared_ptr<Node>> new_samples;
  
  double p_max = p_goal->p + dynamic_window_width + conf.sliding_window_freespace_padding;
  double p_zero = p_goal->p - conf.sliding_window_freespace_padding;

  //std::cout << "p_max: " << p_max << std::endl;
  //std::cout << "p_zero: " << p_zero << std::endl;
  double q_max = conf.q_max;

  int ind = 0;
  while (ind < m)
  {
    // uniformly sample the configuration space
    double rand_p = p_zero + (rand() / (RAND_MAX / (p_max-p_zero)));
    double rand_q = (-1.0*q_max) + (rand() / (RAND_MAX / (q_max- (-1.0*q_max))));
    Node node(rand_p, rand_q);

    // Before we add the sample to the sample vector, we need to collision check it
    //if (is_inside_obs(obs_rectangle, curve_to_euclid(node)))
    if (costmap_col(curve_to_euclid(node)))
    {
      continue;
    }
    else
    {
      new_samples.push_back(std::make_shared<Node> (node));
      ind++; // only increment if we do not have a collision
    }
  }

  // TODO: Generating Pre-seeds
  if (repair_mode == false)
  {

    // hardcoded pre-seed interval for now
    //int pre_seeds = abs(p_goal->p - p_start->p) / 0.25;
    int pre_seeds = abs(p_zero - p_start->p) / 0.25; // Note needed to change p_goal to p_zero. When the sliding window padding is large, pre-seeds wont get generated all the way to the goal

    std::cout << "generating pre-seeds - number is:" << pre_seeds << std::endl;
    //std::cout << "The size of the tree is:" << tree.V.size() << std::endl;

    // In the python version I do this line thing which is more robust, but for now im going to do this quick and dirty
    double p_step = 0.25;
    double p_val = p_zero; // needed to modify the starting i to be p_zero
    for (int i = 0; i < (pre_seeds-1); i++) 
    {
      Node node((p_val+p_step), 0);

      // Before we add the sample to the sample vector, we need to collision check it in euclidean
      //if (is_inside_obs(obs_rectangle, curve_to_euclid(node)) == false)
      //if (costmap_col(curve_to_euclid(node)) == false)
      //{
      new_samples.push_back(std::make_shared<Node> (node)); // previously i was collision checking pre-seeds, but actually I think maybe I shouldnt
      //}

      p_val = p_val + p_step;
    }
  }


  // Cheat samples for simple rectangle experiment
  // NOTE BE VERY CAREFUL, I THINK PUTTING THESE IN SAMPLE BOX INSTEAD OF SAMPLE FREE SPACE CAUSES PROBLEMS DO TO REPOPULATING THE SAME SAMPLE EVERY BATCH
  //Node cheat1 = {8.6,1.1};
  //Node cheat2 = {7.4, 1.05};
  //Node cheat3 = {8.6,1.1};
  //Node cheat4 = {7.4, 1.05};
  //new_samples.push_back(std::make_shared<Node> (cheat1));
  //new_samples.push_back(std::make_shared<Node> (cheat2));

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
    for (int i = 0; i < path_x.size(); i++)
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
  for (int i = 0; i < samples.size(); i++)
  {
    if (f_estimated(*samples[i], *p_start, *p_goal, conf.alpha) < cost_threshold)// Also handles inf flagged values
    {
      samples_pruned.push_back(std::shared_ptr<Node> (samples[i]));
    }
  }

  // We also check the tree and add samples for unconnected vertices back to the sample set
  //We also do a prune of the vertex tree by heuristic
  //&& (samples[i]->g_T != INFINITY)) 
  std::vector<std::shared_ptr<Node>> vertex_pruned;
  int vertices_size = tree.V.size(); 
  for (int i = 0; i < vertices_size; i++)
  {
    if (tree.V[i]->g_T_weighted == INFINITY)
    {
      samples_pruned.push_back(std::shared_ptr<Node> (tree.V[i]));
    }
    if ((f_estimated(*tree.V[i], *p_start, *p_goal, conf.alpha) <= cost_threshold) && (tree.V[i]->g_T_weighted < INFINITY))
    {
      
      vertex_pruned.push_back(std::shared_ptr<Node> (tree.V[i]));
    }
  }
  // After we do both the above loops update the sample vector
  samples = samples_pruned;
  tree.V = vertex_pruned;

  // TODO: Wormhole accomodations  

  // Similar Prune of the Edges
  // TODO:Wormhole accomodations
  std::vector<std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>  edge_pruned;
  for (int i = 0; i <tree.E.size(); i++)
  {
    // In the below condition, I also include the prune of vertices with inf cost to come values
    if ((f_estimated(*std::get<0>(tree.E[i]), *p_start, *p_goal, conf.alpha) <= cost_threshold) && (f_estimated(*std::get<1>(tree.E[i]), *p_start, *p_goal, conf.alpha) <= cost_threshold))
    {
      edge_pruned.push_back(std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> (tree.E[i]));
    }

  }
  tree.E = edge_pruned;
  

}

// Function for updating the state of the robot, updates the goal and sliding window, and prunes the current tree
// Note that this stage requires us to convert the state of the robot in euclidean space into a singular curvilinear space pt, which is actually slightly tricky
// To do without singularities
std::shared_ptr<Node> CBITPlanner::UpdateState(PathDirection path_direction)
{
  //std::cout << "The new state is x: " << new_state->x << " y: " << new_state->y  << std::endl;
  //CLOG(ERROR, "path_planning.cbit_planner") << "Robot Pose: x: " << new_state->x << "Robot Pose: y: " << new_state->y  <<"Robot Pose: yaw: " << new_state->yaw;
  //First calc the qmin distance to the euclidean path (borrow code from ROC generation)
  //To prevent us from having to search the entire euclidean path (and avoid crosses/loops) in the path, use the previous pose as a reference (will always be p=0)
  //and define a subset of points with a lookahead distance
  std::vector<Node> euclid_subset;
  euclid_subset.reserve((conf.roc_lookahead * conf.roc_lookahead * conf.curv_to_euclid_discretization));

  // The length of the subset is determined by the configuration parameter lookahead distance and the desired discretization

  for (double i = (*p_goal).p; i < ((*p_goal).p + conf.roc_lookahead); i += (1.0 / (conf.roc_lookahead * conf.curv_to_euclid_discretization)))
  {
    euclid_subset.push_back(curve_to_euclid(Node(i,0)));
    
  }
  //std::cout << "The final euclid subset point is: x: " << euclid_subset[euclid_subset.size()-1].p << "y: " << euclid_subset[euclid_subset.size()-1].q << std::endl;

  // calc q_min
  double q_min = conf.q_max;
  double q;
  Node closest_pt;
  int closest_pt_ind;

  for (int i=0; i<euclid_subset.size(); i++)
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

  // Experimental code to dynamically predict path direction
  //CLOG(ERROR, "path_planning.cbit_planner") << "Closest Euclid Pt is - x: " << closest_pt.p << " y: " << closest_pt.q;
  //CLOG(ERROR, "path_planning.cbit_planner") << "The Next Euclid Pt is - x: " << euclid_subset[closest_pt_ind+1].p << " y: " << euclid_subset[closest_pt_ind+1].q;
  double test_dx = euclid_subset[closest_pt_ind+1].p - closest_pt.p;
  double test_dy = euclid_subset[closest_pt_ind+1].q - closest_pt.q;
  double test_yaw = atan2(test_dy,test_dx);
  //CLOG(ERROR, "path_planning.cbit_planner") << "Predicted Yaw is: " << test_yaw;
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
  double path_direction2;
  if (angle > M_PI/2) {
      //CLOG(ERROR, "path_planning.cbit_planner") << "Path direction is -1.0 (Reverse)";
      path_direction2 = -1.0;
  } else {
      //CLOG(ERROR, "path_planning.cbit_planner") << "Path direction is +1.0 (Forward)";
      path_direction2 = 1.0;
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
    //A = euclid_subset[closest_pt_ind-1];
    B = euclid_subset[closest_pt_ind];
  }
  else
  {
    //A = euclid_subset[closest_pt_ind];
    B = euclid_subset[closest_pt_ind + 1];
  }


  int q_sign = sgn((B.p - A.p) * (new_state->y - A.q) - ((B.q - A.q) * (new_state->x - A.p))); 
  
  //q_min = q_min * q_sign;
  //std::cout << "q_min is: " << q_min << std::endl; // debug;

  // Note I think we also need to take into account the direction the robot is facing on the path for reverse planning too
  q_min = q_min * q_sign * path_direction2;
  //std::cout << "q_min is: " << q_min << std::endl; // debug;

  /*
  switch (path_direction) {
    case PATH_DIRECTION_REVERSE:
      q_min = -q_min;
      break;
    case PATH_DIRECTION_FORWARD:
      // Do nothing, q_min already has the correct sign
      break;
    default:
      // Handle error case where path_direction is not a valid value
      break;
  }
  */


  // Once we have the closest point on the path, it may not actually be the correct p-value because of singularities in the euclid to curv conversion
  // We need to use this points p-value as a starting point, then search p, qmin space in either direction discretely and find the point with
  // The lowest pose error (including heading) (Should be able to ignore roll and pitch though)
  double pose_err = INFINITY;
  Node test_pt;
  double test_err;
  Node new_state_pq;

  for (double p = (*p_goal).p; p < ((*p_goal).p + conf.roc_lookahead); p += (1.0 / (conf.roc_lookahead * conf.curv_to_euclid_discretization)))
  {
    test_pt = curve_to_euclid(Node(p,q_min));
    double dx = test_pt.p - new_state->x;
    double dy = test_pt.q - new_state->y;
    test_err = sqrt((dy * dy) + (dx * dx));
    if (test_err < pose_err)
    {
      pose_err = test_err;
      new_state_pq = Node(p,q_min);
    }
  }

  // Now update the goal and its cost to come:
  
  std::shared_ptr<Node> new_state_pq_ptr = std::make_shared<Node>(new_state_pq);
  p_goal = new_state_pq_ptr;
  p_goal->g_T = INFINITY;
  p_goal->g_T_weighted = INFINITY;

  //std::cout << "Successfully Updated State: " << std::endl;

  return p_goal;
  // TODO: At this point we would pop out the first element of the new_state_arr so next state update we keep moving forward
  

}



// New Simpler State Update function that instead uses the robots localization frame SID to help find the robot p,q space state
std::shared_ptr<Node> CBITPlanner::UpdateStateSID(int SID, vtr::tactic::EdgeTransform T_p_r)
{
  //std::cout << "The new state is x: " << new_state->x << " y: " << new_state->y  << std::endl;
  //CLOG(ERROR, "path_planning.cbit_planner") << "Robot Pose: x: " << new_state->x << " Robot Pose: y: " << new_state->y  <<" Robot Pose: yaw: " << new_state->yaw;
  //CLOG(ERROR, "path_planning.cbit_planner") << "Current SID is: " << SID;
  
  // Find the corresponding global pose p,q value at the current SID (which should be just behind the actual current state)
  double current_pq_sid_p = global_path->p[SID];
  //CLOG(ERROR, "path_planning.cbit_planner") << "SID P,Q Pose is: " << " p: " << current_pq_sid_p << " q: " << 0.0;

  std::vector<Node> euclid_subset;
  euclid_subset.reserve((conf.roc_lookahead * conf.roc_lookahead * conf.curv_to_euclid_discretization));

  // The length of the subset is determined by the configuration parameter lookahead distance and the desired discretization
  // Use the SID values to help predict how far we should look ahead look ahead without falling into false minim on crossing paths.
  double lookahead_range = (*p_goal).p + conf.roc_lookahead; //default value from config
  for (int ID = SID; ID < global_path->p.size(); ID += 1)
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
  //std::cout << "The final euclid subset point is: x: " << euclid_subset[euclid_subset.size()-1].p << "y: " << euclid_subset[euclid_subset.size()-1].q << std::endl;

  // calc q_min
  double q_min = conf.q_max;
  double q;
  Node closest_pt;
  int closest_pt_ind;

  for (int i=0; i<euclid_subset.size(); i++)
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

  // Experimental code to dynamically predict path direction
  //CLOG(ERROR, "path_planning.cbit_planner") << "Closest Euclid Pt is - x: " << closest_pt.p << " y: " << closest_pt.q;
  //CLOG(ERROR, "path_planning.cbit_planner") << "The Next Euclid Pt is - x: " << euclid_subset[closest_pt_ind+1].p << " y: " << euclid_subset[closest_pt_ind+1].q;
  double test_dx = euclid_subset[closest_pt_ind+1].p - closest_pt.p;
  double test_dy = euclid_subset[closest_pt_ind+1].q - closest_pt.q;
  double test_yaw = atan2(test_dy,test_dx);
  //CLOG(ERROR, "path_planning.cbit_planner") << "Predicted Yaw is: " << test_yaw;
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
  double path_direction2;
  if (angle > M_PI/2) {
      //CLOG(ERROR, "path_planning.cbit_planner") << "Path direction is -1.0 (Reverse)";
      path_direction2 = -1.0;
  } else {
      //CLOG(ERROR, "path_planning.cbit_planner") << "Path direction is +1.0 (Forward)";
      path_direction2 = 1.0;
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
    //A = euclid_subset[closest_pt_ind-1];
    B = euclid_subset[closest_pt_ind];
  }
  else
  {
    //A = euclid_subset[closest_pt_ind];
    B = euclid_subset[closest_pt_ind + 1];
  }


  int q_sign = sgn((B.p - A.p) * (new_state->y - A.q) - ((B.q - A.q) * (new_state->x - A.p))); 
  
  //q_min = q_min * q_sign;
  //std::cout << "q_min is: " << q_min << std::endl; // debug;

  // Note I think we also need to take into account the direction the robot is facing on the path for reverse planning too
  q_min = q_min * q_sign * path_direction2;
  //std::cout << "q_min is: " << q_min << std::endl; // debug;

  /*
  switch (path_direction) {
    case PATH_DIRECTION_REVERSE:
      q_min = -q_min;
      break;
    case PATH_DIRECTION_FORWARD:
      // Do nothing, q_min already has the correct sign
      break;
    default:
      // Handle error case where path_direction is not a valid value
      break;
  }
  */


  // Once we have the closest point on the path, it may not actually be the correct p-value because of singularities in the euclid to curv conversion
  // We need to use this points p-value as a starting point, then search p, qmin space in either direction discretely and find the point with
  // The lowest pose error (including heading) (Should be able to ignore roll and pitch though)
  double pose_err = INFINITY;
  Node test_pt;
  double test_err;
  Node new_state_pq;

  for (double p = (*p_goal).p; p < ((*p_goal).p + conf.roc_lookahead); p += (1.0 / (conf.roc_lookahead * conf.curv_to_euclid_discretization)))
  {
    test_pt = curve_to_euclid(Node(p,q_min));
    double dx = test_pt.p - new_state->x;
    double dy = test_pt.q - new_state->y;
    test_err = sqrt((dy * dy) + (dx * dx));
    if (test_err < pose_err)
    {
      pose_err = test_err;
      new_state_pq = Node(p,q_min);
    }
  }

  // Now update the goal and its cost to come:
  
  std::shared_ptr<Node> new_state_pq_ptr = std::make_shared<Node>(new_state_pq);
  p_goal = new_state_pq_ptr;
  p_goal->g_T = INFINITY;
  p_goal->g_T_weighted = INFINITY;

  //std::cout << "Successfully Updated State: " << std::endl;

  return p_goal;
  // TODO: At this point we would pop out the first element of the new_state_arr so next state update we keep moving forward
  



}



double CBITPlanner::BestVertexQueueValue()
{
  //if (tree.QV.size() == 0)
  //{
  //  return INFINITY;
  //}
  if (tree.QV2.size() == 0)
  {
    return INFINITY;
  }
  // Iterate through all vertices in the queue, find the minimum cost heurstic
  /*
  double min_vertex_cost = INFINITY;
  for (int i = 0; i < tree.QV.size(); i++)
  {
    double weighted_heuristic = tree.QV[i]->g_T_weighted + h_estimated_admissible(*tree.QV[i], *p_goal);
    // TODO: Add in the h_estimated heuristic
    if (weighted_heuristic < min_vertex_cost)
    {
      min_vertex_cost = weighted_heuristic;
    }
  }
  */
  
  // New multimap implementation of this:
  std::multimap<double, std::shared_ptr<Node>>::iterator itr = tree.QV2.begin();
  double min_vertex_cost = itr -> first;

  return min_vertex_cost;
}



double CBITPlanner::BestEdgeQueueValue()
{
  //if (tree.QE.size() == 0)
  //{
  //  return INFINITY;
  //}
  if (tree.QE2.size() == 0)
  {
    return INFINITY;
  }
  /*
  double min_edge_cost = INFINITY;
  for (int i = 0; i < tree.QE.size(); i++)
  {
    // TODO: Add in the h_estimated heuristic
    // The edge queue is stored as a vector of tuples, the first of which represents the starting vertex, and the second is the vertex
    // which defines the end point of the connecting edge
    std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> vertex_tuple = tree.QE[i];
    Node v = *std::get<0>(vertex_tuple);
    Node x = *std::get<1>(vertex_tuple);

    double weighted_heuristic = v.g_T_weighted + calc_weighted_dist(v, x, conf.alpha) + h_estimated_admissible(x, *p_goal);
    
    if (weighted_heuristic < min_edge_cost)
    {
      min_edge_cost = weighted_heuristic;
    }
  }
  */

  // Using multimaps to accomplish the same thing:
  std::multimap<double, std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>::iterator itr = tree.QE2.begin();
  double min_edge_cost = itr -> first;
    
  return min_edge_cost;
}



std::shared_ptr<Node> CBITPlanner::BestInVertexQueue()
{
  //if (tree.QV.size() == 0)
  //{
    //std::cout << "Vertex Queue is Empty!" << std::endl;
    //CLOG(INFO, "path_planning.cbit_planner") << "Vertex Queue is Empty, Something went Wrong!";
  //}
  if (tree.QV2.size() == 0)
  {
    CLOG(INFO, "path_planning.cbit_planner") << "Vertex Queue is Empty, Something went Wrong!";
  }


  // Loop through the vertex queue, select the vertex node with the lowest cost to come + heuristic dist to goal
  /*
  double min_vertex_cost = INFINITY;
  std::shared_ptr<Node> best_vertex;
  int best_vertex_ind;
  for (int i = 0; i < tree.QV.size(); i++)
  {
    double weighted_heuristic = tree.QV[i]->g_T_weighted + h_estimated_admissible(*tree.QV[i], *p_goal);
    // TODO: Add in the h_estimated heuristic
    if (weighted_heuristic < min_vertex_cost)
    {
      min_vertex_cost = weighted_heuristic;
      best_vertex = tree.QV[i];
      best_vertex_ind = i;
    }
  }   
  // Once we process the vertex and have it ready to return, remove it from the vector
  auto it = tree.QV.begin() + best_vertex_ind;
  *it = std::move(tree.QV.back());
  tree.QV.pop_back();
  */


  // New multimap implementation of this:
  std::multimap<double, std::shared_ptr<Node>>::iterator itr = tree.QV2.begin();
  std::shared_ptr<Node> best_vertex = itr -> second;
  double min_vertex_cost = itr -> first;
  tree.QV2.erase(itr);

  return best_vertex;
}



void CBITPlanner::ExpandVertex(std::shared_ptr<Node> v)
{
  // Note its easier in c++ to remove the vertex from the queue in the bestinvertexqueue function instead
  // Find nearby samples and filter by heuristic potential
  for (int i = 0; i < samples.size(); i++)
  {
    if ((calc_dist(*samples[i], *v) <= conf.initial_exp_rad) && ((g_estimated_admissible(*v, *p_start) + calc_weighted_dist(*v, *samples[i], conf.alpha) + h_estimated_admissible(*samples[i], *p_goal)) <= p_goal->g_T_weighted))
    {
      samples[i]->g_T = INFINITY;
      samples[i]->g_T_weighted = INFINITY;

      // copying method
      //std::shared_ptr<Node> v_copy = v; 
      //std::shared_ptr<Node> x_copy = samples[i]; 
      //tree.QE.push_back(std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> (v_copy, x_copy));

      // direct method
      //tree.QE.push_back(std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {(v), (samples[i])});
      tree.QE2.insert(std::pair<double, std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>((v->g_T_weighted + calc_weighted_dist(*v,*samples[i],conf.alpha) + h_estimated_admissible(*samples[i], *p_goal)), std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {(v), (samples[i])}));

    }
  }

  // I found the below code, while does follow the original algorithm, tends not to hurt compute performance
  // yet allows the pre-seeded samples to much much better in subsequent iterations
  // Removing this lets the tree use old verticies to find better paths more often (but subequently means you check more)
  /* 
  // First check to see that v is not in the old tree already, if it is return early
  for (int i = 0; i < tree.V_Old.size(); i++)
  {
    if (v == tree.V_Old[i]) 
    {
      //CLOG(WARNING, "path_planning.cbit_planner") << "Returning early, expansion vertex is in V_old"; // For debug
      return;
    }
  }
  */
  

  // find nearby vertices and filter by heuristic potential
  for (int i = 0; i < tree.V.size(); i++)
  {
    //auto test1 = calc_dist(*tree.V[i], *v);
    //auto test2 = (g_estimated(*v, *p_start, conf.alpha) + calc_weighted_dist(*v, *tree.V[i], conf.alpha) + h_estimated(*tree.V[i], *p_goal, conf.alpha));
    //auto test3 = (v->g_T_weighted + calc_weighted_dist(*v, *tree.V[i], conf.alpha));
    if ((calc_dist(*tree.V[i], *v) <= conf.initial_exp_rad) && ((g_estimated_admissible(*v, *p_start) + calc_weighted_dist(*v, *tree.V[i], conf.alpha) + h_estimated_admissible(*tree.V[i], *p_goal)) < p_goal->g_T_weighted) && ((v->g_T_weighted + calc_weighted_dist(*v, *tree.V[i], conf.alpha)) < tree.V[i]->g_T_weighted))
    {
      if (edge_in_tree_v2(v, tree.V[i]) == false)
      {
        // If all conditions satisfied, add the edge to the queue
        //tree.V[i]->g_T = INFINITY; // huh actually looking at this again, im not sure if I should be doing this (yeah I think was a mistake)
        //tree.V[i]->g_T_weighted = INFINITY; // huh actually looking at this again, im not sure if I should be doing this (yeah I think its a mistake)
        //tree.QE.push_back(std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {(v), (tree.V[i])});
        tree.QE2.insert(std::pair<double, std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>((v->g_T_weighted + calc_weighted_dist(*v,*tree.V[i],conf.alpha) + h_estimated_admissible(*tree.V[i], *p_goal)), std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {(v), (tree.V[i])}));

      }
      /*
      // Also check whether the edge is in the tree or not
      if (edge_in_tree(*v, *tree.V[i]) == false)
      {
        // If all conditions satisfied, add the edge to the queue
        //tree.V[i]->g_T = INFINITY; //I think these two lines were a mistake, should not be here
        //tree.V[i]->g_T_weighted = INFINITY; //I think these two lines were a mistake, should not be here
        tree.QE.push_back(std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {(v), (tree.V[i])});
        //tree.QE2.insert(std::pair<double, std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>((v->g_T_weighted + calc_weighted_dist(*v,*tree.V[i],conf.alpha) + h_estimated_admissible(*tree.V[i], *p_goal)), std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {(v), (tree.V[i])}));

      */
    } 
  }
}


std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> CBITPlanner::BestInEdgeQueue()
{
  //if (tree.QE.size() == 0) // need to handle a case where the return path is 100% optimal in which case things get stuck and need ot be flagged to break
  //{
  //  CLOG(DEBUG, "path_planning.cbit_planner") << "Edge Queue is Empty, Solution Could Not be Improved This Batch";
  //  return std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {NULL, NULL};
  //}
  if (tree.QE2.size() == 0) // need to handle a case where the return path is 100% optimal in which case things get stuck and need ot be flagged to break
  {
    CLOG(DEBUG, "path_planning.cbit_planner") << "Edge Queue is Empty, Solution Could Not be Improved This Batch";
    repair_mode = true;
    return std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {NULL, NULL};
  }

  /*
  // Loop through edge queue, find the edge with the smallest heuristic cost
  std::shared_ptr<Node> v;
  std::shared_ptr<Node> x;
  double min_cost = INFINITY;
  double cost;
  int best_edge_ind;
  for (int i = 0; i < tree.QE.size(); i++)
  {
    cost = std::get<0>(tree.QE[i])->g_T_weighted + calc_weighted_dist(*std::get<0>(tree.QE[i]), *std::get<1>(tree.QE[i]), conf.alpha) + h_estimated_admissible(*std::get<1>(tree.QE[i]), *p_goal);
    if (cost < min_cost)
    {
      min_cost = cost;
      best_edge_ind = i;
      v = std::get<0>(tree.QE[i]);
      x = std::get<1>(tree.QE[i]);
    }    
  }

  // Once we process the vertex and have it ready to return, remove it from the edge queue (This is apparently a fast way of doing so)
  auto it = tree.QE.begin() + best_edge_ind;
  *it = std::move(tree.QE.back());
  tree.QE.pop_back();
  return {v,x};
  */

  // Equivalent code using multimaps:
  std::multimap<double, std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>::iterator itr = tree.QE2.begin();
  double min_edge_cost = itr -> first;
  std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> edge_tuple = itr -> second;
  tree.QE2.erase(itr);
  return edge_tuple;
}


std::tuple<std::vector<double>, std::vector<double>> CBITPlanner::ExtractPath(vtr::path_planning::BasePathPlanner::RobotState& robot_state, std::shared_ptr<CBITCostmap> costmap_ptr)
{
  // DEBUG, remove these lines longer term
  int inf_loop_counter = 0;
  bool debug_display = false;
  // end of debug lines

  Node node = *p_goal;
  
  std::vector<double> path_p = {node.p};
  std::vector<double> path_q = {node.q};

  while(node.parent != nullptr)
  {
    // Start of debug lines
    inf_loop_counter = inf_loop_counter + 1;
    if (inf_loop_counter >= 5000)
    {
      debug_display = true;
    }
    if (debug_display == true)
    {
      CLOG(DEBUG, "path_planning.cbit_planner") << "Something Went Wrong - Infinite Loop Detected, Initiating Hard Reset:";
      //std::this_thread::sleep_for(std::chrono::milliseconds(2000)); //temp, remove this
      //HardReset(robot_state, costmap_ptr);
      
      
      
      
      /*
      CLOG(ERROR, "path_planning.cbit_planner") << "Node p:" << node.p << " q: " << node.q;
      CLOG(ERROR, "path_planning.cbit_planner") << "Node Parent p:" << (*node.parent).p << " q: " << (*node.parent).q;
      CLOG(ERROR, "path_planning.cbit_planner") << "p_goal p:" << p_goal->p << " q: " << p_goal->q;
      CLOG(ERROR, "path_planning.cbit_planner") << "Attempting to display the tree";
      for (int i = 0; i < tree.V.size(); i++)
      {
        CLOG(ERROR, "path_planning.cbit_planner") << "Tree Node " << i << " with p:" << (tree.V[i])->p << " q: " << (tree.V[i])->q;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      */
    }
    // end of debug lines
    node = *node.parent;
    path_p.push_back(node.p);
    path_q.push_back(node.q);

  }
  return {path_p, path_q};
}


// Function to convert the resulting p,q path into a finely discretized euclidean path

std::vector<Pose>  CBITPlanner::ExtractEuclidPath()
{
  Node node = *p_goal;
  
  std::vector<double> path_x;
  std::vector<double> path_y;

  // Experimental
  std::vector<double> path_z;
  std::vector<double> path_yaw;
  std::vector<double> path_p;
  std::vector<double> path_q;

  // experimental, for grabing the yaw value from the teach path
  std::vector<double> p = global_path->p;
  std::vector<Pose> disc_path = global_path->disc_path;
  // End of experimental

  std::vector<Node> node_list = {node};
  node_list.reserve(1000); // Maybe dynamically allocate this size based on path length/discretization param
  double discretization = conf.curv_to_euclid_discretization;


  while(node.parent != nullptr)
  {
    node = *node.parent;
    node_list.push_back(node);
  }

  for (int i=0; i < (node_list.size()-1); i++)
  {
    Node start = node_list[i];
    Node end = node_list[i+1];
    std::vector<double> p_disc;
    std::vector<double> q_disc;

    // TODO, wormhole modification goes here

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
      for (int i=0; i < p_disc.size(); i++)
      {
        q_disc.push_back((p_disc[i] * slope + b));
      }
    }

    // Once we have built the discretized vectors, loop through them, convert each point to Euclidean space and store it in the Euclidean path vector
    for (int i=0; i < p_disc.size(); i ++)
    {
      Node euclid_pt = curve_to_euclid(Node(p_disc[i], q_disc[i]));
      path_x.push_back(euclid_pt.p);
      path_y.push_back(euclid_pt.q);
      path_z.push_back(euclid_pt.z);

      // Experimental yaw estimation from teach path
      int p_iter = 0;
      while (p_disc[i] > p[p_iter])
      {
          p_iter = p_iter+1;
      }
      path_yaw.push_back(disc_path[p_iter-1].yaw);
      // End Experimental

      
      
      //For debug
      //path_z.push_back(0.0);
      // Also adding the p,q to the euclidean path for reference
      path_p.push_back(p_disc[i]);
      path_q.push_back(q_disc[i]);

    }

  }
  

  // DEBUG Temporarily reshaping the output into the form we need (vector of poses)
  // TODO: Longer term I need to rework this whole function to output the poses directly in the loops so we dont have to redundently loop again
  // Also need to implement a new curve_to_euclid_se3 function which converts to 3D Poses instead of nodes which we will only use here
  std::vector<Pose> pose_vector;
  for (int i = 0; i<path_x.size(); i++)
  {
    Pose next_pose = Pose(path_x[i], path_y[i], path_z[i], 0.0, 0.0, path_yaw[i]);
    next_pose.p = path_p[i];
    next_pose.q = path_q[i];
    pose_vector.push_back(next_pose);// Temporarily set pose with x,y coordinate, no orientation
  }

  return pose_vector;
}



bool CBITPlanner::col_check_path()
{
  // Note this doesnt work, we need to implement a new method which actually stores the path in a vector of Node pointers so all the parents/g-T's get transffered
  //std::tuple<std::vector<double>, std::vector<double>> curv_path = ExtractPath();
  //std::vector<double> path_p = std::get<0>(curv_path);
  //std::vector<double> path_q = std::get<1>(curv_path);

  // Generate path to collision check
  std::vector<std::shared_ptr<Node>> curv_path;
  std::shared_ptr<Node> node_ptr = p_goal;
  curv_path.push_back(node_ptr);

  while ((node_ptr->parent != nullptr))
  {
    node_ptr = node_ptr->parent;
    curv_path.push_back(node_ptr);
  }

  // Get the current path for collision checking



  bool collision = false;
  int vertex_counter = 0;
  int vertex_lookahead = 0; // This is a param used to define how far ahead from the other side of a moved obstacle we should plan to. May just leave at 0
                        // But could consider making this a tuning param TODO.

  Node collision_free_vertex;

  // Iterate backwards through the path from end of the path to the robot state.
  // The first time we see a collision, we take the vertex previous to the collision point and set this as our collision free node
  // We continue collision checking, there may be several connected vertices in a row which are now inside an obstacle
  // Eventually we will reach the other side of the obstacle (left side) and there will be another collision free node, this is denoted as the repair vertex.
  // The tree from the end of the path to the collision free vertex can be left alone

  // In repair mode, we aim to fill in the gap between the collision free vertex and the repair node on the other side of the obstacle (left side)
  // Once we successfully repair it, we update the cost to comes and try to restore as much of the tree as we can.
  for (int i = curv_path.size()-1; i>=0; i--)
  {
    Node vertex = *curv_path[i];
    Node euclid_pt = curve_to_euclid(vertex);
    //if (is_inside_obs(obs_rectangle, euclid_pt)) // Legacy collision checking
    if (costmap_col(euclid_pt))
    {
      if (collision == false) // Enter this statement only the first time
      {
        int vertex_ind = i;
        collision_free_vertex = *(curv_path[i+1]);
        collision = true;
      }
    }
  

    else
    {
      if (collision == true)
      {
        if (vertex_counter == vertex_lookahead)
        {
          repair_vertex = std::make_shared<Node> (vertex);
          dynamic_window_width = collision_free_vertex.p - repair_vertex->p;

          // Store the old cost to come to get ot this repair vertex
          repair_g_T_old = repair_vertex->g_T;
          repair_g_T_weighted_old = repair_vertex->g_T_weighted;
          break;
        }

        vertex_counter = vertex_counter + 1;
      
      }
    }
  }

  // If we have found a collision, we need to reset the vertex tree
  if (collision == true)
  {
    // Store a repair backup of all the vertices we temporarily pruned for the repair
    std::vector<std::shared_ptr<Node>> pruned_vertex_tree;
    pruned_vertex_tree.reserve(tree.V.size());
    for (int i =0; i<tree.V.size(); i++)
    {
      if ((tree.V[i]->g_T_weighted > collision_free_vertex.g_T_weighted) && (tree.V[i]->g_T_weighted < 1.0e5)) // Second condition is for wormholes to be implemented TODO
      {
        tree.V_Repair_Backup.push_back(tree.V[i]);
      }

      if (tree.V[i]->g_T_weighted < collision_free_vertex.g_T_weighted) // TODO, add wormhole condition here
      {
        pruned_vertex_tree.push_back(std::shared_ptr<Node> (tree.V[i]));
      }
    }

    tree.V = pruned_vertex_tree;

    // Do something similar for edges (note in the offline cpp version, I guess I need to kind of reset the plots here too because they only incrementally plot?)
    std::vector<std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>>>  pruned_edge_tree;
    pruned_edge_tree.reserve(tree.E.size());
    for (int i = 0; i <tree.E.size(); i++)
    {
      if (std::get<1>(tree.E[i])->g_T_weighted <= collision_free_vertex.g_T_weighted) // Need to add wormhole or condition to this still
      {
        pruned_edge_tree.push_back(tree.E[i]);
      }
    }
    
    tree.E = pruned_edge_tree;

    // TODO: potentially check if plotting is enabled, and if so, clear the current plot and basically reset and replot all the current samples, vertices, edges
    // (Might be a good idea to just make a function for this, it would probably come in handy)

    //reset queues
    //tree.QV.clear();
    tree.QV2.clear();
    //tree.QE.clear();
    tree.QE2.clear();

    // Then set a temporary goal to be the repair vertex, and signal the planner to enter repair mode
    p_goal_backup = p_goal;
    
    p_goal = repair_vertex;
    p_goal->g_T = INFINITY;
    p_goal->g_T_weighted = INFINITY;
    samples.push_back(p_goal);  
    // DEBUG MESSAGE
    //std::cout << "Returning repair mode true (Collision Detected)" << std::endl;
    CLOG(INFO, "path_planning.cbit_planner") << "Returning repair mode true (Collision Detected)";
    return true;
  }

  else // no collsion found, restore the dynamic window and return repair mode is false indicated we do not need to repair
  {
    dynamic_window_width = conf.sliding_window_width;
    // DEBUG MESSAGE
    //std::cout << "Returning repair mode false (No Collision)" << std::endl;
    //CLOG(ERROR, "path_planning.cbit_planner") << "Returning repair mode false (No Collision)" ;
    return false;
  }

}

std::shared_ptr<Node> CBITPlanner::col_check_path_v2(double max_lookahead_p)
{

  // Generate path to collision check
  std::vector<std::shared_ptr<Node>> curv_path;
  std::shared_ptr<Node> node_ptr = p_goal;
  curv_path.push_back(node_ptr);

  while ((node_ptr->parent != nullptr))
  {
    node_ptr = node_ptr->parent;
    curv_path.push_back(node_ptr);
  }

  // find the first vertex which results in a collision (curv_path is generated from left to right, so we should search in reverse)
  std::shared_ptr<Node> col_free_vertex = nullptr;

  // TODO, actually we probably only want to consider edges in the lookahead (not way far away down the path) but I might deal with this later
  for (int i = curv_path.size()-1; i>=0; i--) // I decided to have it -3 instead of -1, this prevents us from collision checking the end of path, but I think I want that for now and it lets me in
  {
    Node vertex = *curv_path[i];
    // If the vertex in the path is outside of our sliding window then skip it
    // This prevents us from having to rewire the path for distance obstacles outside of our window (especially in cases where our path crosses itself)
    if (vertex.p > max_lookahead_p)
    {
      continue;
    }
    Node euclid_pt = curve_to_euclid(vertex);
    //if (is_inside_obs(obs_rectangle, euclid_pt)) // Legacy collision checking
    if (costmap_col_tight(euclid_pt))
    {
      //col_free_vertex = curv_path[i+1]; // take the vertex just before the collision vertex
      if (i+4 < curv_path.size()-1)
      {
        col_free_vertex = curv_path[i+4]; // im actually finding that this vertex may be a little too close to the obstacles. Better to take one slightly further ahead if we can
      }
      else
      {
        col_free_vertex = curv_path[i+1]; 
      }
    }

  }
  return col_free_vertex;
}



void CBITPlanner::restore_tree(double g_T_update, double g_T_weighted_update)
{
  //PSEUDOCODE:
  // Loop through all the vertices stored in the repair backup tree

  // Follow each vertex back up the parent chain, branches which have the repair_vertex at some point in the chain can be restored, the others cannot

  // While doing this, keep track of all the vertices and edges which we are going to combine in the tree.V and tree.E vectors

  // Experimental: (Consider replacing the below code with the above pseudocode implementation)
  // Quick and dirty version, only restore the primary trunk (which I think honestly might be faster longer term)
  std::shared_ptr<Node> node_ptr = p_goal;
  while (!((node_ptr->parent->p == repair_vertex->p) && (node_ptr->parent->q == repair_vertex->q))) // break when we reach the repair vertex
  {
    // Adjust costs
    node_ptr->g_T = node_ptr->g_T + g_T_update;
    node_ptr->g_T_weighted = node_ptr->g_T_weighted + g_T_weighted_update;

    // Add the vertex to the tree
    tree.V.push_back(node_ptr);

    // Add the vertex to parent edge to the tree
    tree.E.push_back(std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {node_ptr, node_ptr->parent});

    // continue up the chain
    node_ptr = node_ptr->parent;


    // consider adding if statement for replotting these edges that we restored if incremental plot is on
  }

  // For the last iteration, we also need to change the parent of the final vertex to be the repair vertex instead of the previous edge (which shouldnt exist)
  node_ptr->parent = repair_vertex;
  tree.E.push_back(std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {node_ptr, node_ptr->parent});
}




// Function which takes in a beginning vertex v, and an end vertex x, and checks whether its in the tree or not already
bool CBITPlanner::edge_in_tree(Node v, Node x)
{
  for (int i = 0; i < tree.E.size(); i++ )
  {
      
    if ((std::get<0>(tree.E[i])->p == v.p && std::get<0>(tree.E[i])->q == v.q) && (std::get<1>(tree.E[i])->p == x.p && std::get<1>(tree.E[i])->q == x.q))
    {
        return true;
    }
    /*
    if (std::get<1>(tree.E[i])->p == x.p && std::get<1>(tree.E[i])->q == x.q)
    {
        return true;
    }
    */
  }
  return false;
}
// Function which takes in a beginning vertex v, and an end vertex x, and checks whether its in the tree or not already
// This version uses address matching (safer)
bool CBITPlanner::edge_in_tree_v2(std::shared_ptr<Node> v, std::shared_ptr<Node> x)
{
  int edges_size = tree.E.size();
  for (int i = 0; i < edges_size; i++ )
  {
    // Alternative way to do this using node addresses
    if ((std::get<0>(tree.E[i]) == v) && (std::get<1>(tree.E[i]) == x))
    {
      return true;
    }
  }
  return false;
}





// Function for checking whether a node lives in the Vertex tree
bool CBITPlanner::node_in_tree(Node x)
{
  for (int i = 0; i < tree.V.size(); i++ )
  {
      
    if (tree.V[i]->p == x.p && tree.V[i]->q == x.q)
    {
        return true;
    }
  }
  return false;
  
}

// Function for checking whether a node lives in the Vertex tree, updated to use address matching (safer)
bool CBITPlanner::node_in_tree_v2(std::shared_ptr<Node>  x)
{
  int vertices_size = tree.V.size();
  for (int i = 0; i < vertices_size; i++ )
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

// Probably remove these, putting in utils

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


// TODO: Overhaul this function to work with the costmaps
bool CBITPlanner::is_inside_obs(std::vector<std::vector<double>> obs, Node node)
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



// This collision check is only used at the end of each batch and determines whether the path should be rewired using the bare minimum obstacle distance
// Under normal operation we plan paths around a slightly more conservative buffer around each obstacle (equal to influence dist + min dist)
bool CBITPlanner::costmap_col_tight(Node node)
{
  //CLOG(DEBUG, "path_planning.cbit_planner") << "Original Node: x: " << node.p << " y: " << node.q << " z: " << node.z;
  Eigen::Matrix<double, 4, 1> test_pt({node.p, node.q, node.z, 1});

  // I am no longer temporally filtering here, instead this takes place in the costmap itself in the change detection module
  // This means the costmap and transform size should never be more than 1
  //for (int i = 0; i < cbit_costmap_ptr->T_c_w_vect.size(); i++)
  //{


  auto collision_pt = cbit_costmap_ptr->T_c_w * test_pt;

  // Round the collision point x and y values down to the nearest grid resolution so that it can be found in the obstacle unordered_map
  float x_key = floor(collision_pt[0] / cbit_costmap_ptr->grid_resolution) * cbit_costmap_ptr->grid_resolution;
  float y_key = floor(collision_pt[1] / cbit_costmap_ptr->grid_resolution) * cbit_costmap_ptr->grid_resolution;

  //CLOG(DEBUG, "path_planning.cbit_planner") << "X_key:  " << x_key;
  //CLOG(DEBUG, "path_planning.cbit_planner") << "Y_key:  " << y_key;

  float grid_value;

  // Check to see if the point is in the obstacle map
  // We need to use a try/catch in this metod as if the key value pair doesnt exist (it usually wont) we catch an out of range error
  try 
  {
  // Block of code to try
    grid_value = cbit_costmap_ptr->obs_map.at(std::pair<float, float> (x_key, y_key));
    //CLOG(ERROR, "path_planning.cbit_planner") << "Key Value:  " << grid_value;
  }
  catch (std::out_of_range) 
  {
    grid_value = 0.0;
  }

  if (grid_value >= 0.89) // By switching this from > 0.0 to 0.9, we effectively only collision check the path out to the "minimum_distance" obs config param
  {
    return true;
  }
  //}

  // If we make it here can return false for no collision
  return false;
}



// More conservative costmap checking out to a distance of "influence_distance" + "minimum_distance" away
bool CBITPlanner::costmap_col(Node node)
{
  //CLOG(DEBUG, "path_planning.cbit_planner") << "Original Node: x: " << node.p << " y: " << node.q << " z: " << node.z;
  Eigen::Matrix<double, 4, 1> test_pt({node.p, node.q, node.z, 1});
  // I am no longer temporally filtering here, instead this takes place in the costmap itself in the change detection module
  // This means the costmap and transform size should never be more than 1
  //for (int i = 0; i < cbit_costmap_ptr->T_c_w_vect.size(); i++)
  //{

  //auto collision_pt = cbit_costmap_ptr->T_c_w * test_pt;
  auto collision_pt = cbit_costmap_ptr->T_c_w * test_pt;
  //CLOG(DEBUG, "path_planning.cbit_planner") << "Displaying the point in the costmap frame we are trying to collision check: " << collision_pt;
  //CLOG(DEBUG, "path_planning.cbit_planner") << "X:  " << collision_pt[0];
  //CLOG(DEBUG, "path_planning.cbit_planner") << "Y:  " << collision_pt[1];
  //CLOG(DEBUG, "path_planning.cbit_planner") << "Resolution:  " << cbit_costmap_ptr->grid_resolution;


  // Round the collision point x and y values down to the nearest grid resolution so that it can be found in the obstacle unordered_map
  float x_key = floor(collision_pt[0] / cbit_costmap_ptr->grid_resolution) * cbit_costmap_ptr->grid_resolution;
  float y_key = floor(collision_pt[1] / cbit_costmap_ptr->grid_resolution) * cbit_costmap_ptr->grid_resolution;

  //CLOG(DEBUG, "path_planning.cbit_planner") << "X_key:  " << x_key;
  //CLOG(DEBUG, "path_planning.cbit_planner") << "Y_key:  " << y_key;

  float grid_value;

  // Check to see if the point is in the obstacle map
  // If it isnt in the map we will catch an out of range error
  try 
  {
      grid_value = cbit_costmap_ptr->obs_map.at(std::pair<float, float> (x_key, y_key));
      //CLOG(DEBUG, "debug") << " The Grid value for this cell is: " << grid_value;
  }
  
  catch (std::out_of_range) 
  {

    grid_value = 0.0;
  }

  if (grid_value > 0.0)
    {
      return true;
    }

  //}
  // If we make it here, return false for no collision
  return false;
}


// note with new collision checker from global costmap I dont think we use this obs anymore?
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
    for (int i = 0; i < p_test.size(); i++)
    {
        Node curv_pt = Node(p_test[i], q_test[i]);

        // Convert to euclid TODO:
        //Node euclid_pt = curv_pt; // DEBUG DO NOT LEAVE THIS HERE, NEED TO REPLACE WITH COLLISION CHECK FUNCTION
        Node euclid_pt = curve_to_euclid(curv_pt);
        //if (is_inside_obs(obs, euclid_pt))
        if (costmap_col(euclid_pt))
        {
            return true;
        }
    }

    return false;
}



// Corridor Update functions
// DEBUG: Long term when I restructure things I want these to be inside the generate_pq.cpp file
// The only thing keeping me from doing this right away is that all the curvilinear conversions and collision checkers are here
// But long term I should move all those over to utils or even a different header and script

// Old code I wrote for this function

struct CBITPlanner::collision_result CBITPlanner::discrete_collision_v2(double discretization, Node start, Node end, bool tight)
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
    Node curv_pt;
    for (int i = 0; i < p_test.size(); i++)
    {
        curv_pt = Node(p_test[i], q_test[i]);

        // Convert to euclid TODO:
        //Node euclid_pt = curv_pt; // DEBUG DO NOT LEAVE THIS HERE, NEED TO REPLACE WITH COLLISION CHECK FUNCTION
        Node euclid_pt = curve_to_euclid(curv_pt);
        //if (is_inside_obs(obs, euclid_pt))
        if (tight == false)
        {
          if (costmap_col(euclid_pt))
          {
            return {true, curv_pt};
          }
        }
        else
        {
          if (costmap_col_tight(euclid_pt))
          {
            return {true, curv_pt};
          }
        }

    }

    return {false, curv_pt};
}



// ChatGPT optimized code for this function
/*
struct CBITPlanner::collision_result CBITPlanner::discrete_collision_v2(double discretization, Node start, Node end, bool tight)
{
    // We dynamically determine the discretization based on the length of the edge
    discretization = ceil(calc_dist(start, end) * discretization);

    // Generate discretized test nodes
    std::vector<double> p_test;
    std::vector<double> q_test;

    double p_diff = end.p - start.p;
    double q_diff = end.q - start.q;
    double p_sign = sgn(p_diff);
    double q_sign = sgn(q_diff);
    double p_step = p_sign * fabs(p_diff) / discretization;
    double q_step = q_sign * fabs(q_diff) / discretization;

    p_test.push_back(start.p);
    q_test.push_back(start.q);

    for (auto it = p_test.begin(), jt = q_test.begin(); it != p_test.end() - 1; ++it, ++jt)
    {
        *std::next(it) = *it + p_step * p_sign;
        *std::next(jt) = *jt + q_step * q_sign;
    }
    p_test.push_back(end.p);
    q_test.push_back(end.q);

    // Loop through the test curvilinear points, convert to euclid, collision check obstacles
    Node curv_pt;
    for (auto it = p_test.begin(), jt = q_test.begin(); it != p_test.end(); ++it, ++jt)
    {
        curv_pt = Node(*it, *jt);

        // Convert to euclid TODO:
        //Node euclid_pt = curv_pt; // DEBUG DO NOT LEAVE THIS HERE, NEED TO REPLACE WITH COLLISION CHECK FUNCTION
        Node euclid_pt = curve_to_euclid(curv_pt);
        //if (is_inside_obs(obs, euclid_pt))
        if (tight == false)
        {
            if (costmap_col(euclid_pt))
            {
                return {true, curv_pt};
            }
        }
        else
        {
            if (costmap_col_tight(euclid_pt))
            {
                return {true, curv_pt};
            }
        }
    }
    return {false, curv_pt};
}
*/

           

// dont love how this function is coded, could be done much more efficiently with some more effort I think
void CBITPlanner::update_corridor(std::shared_ptr<CBITCorridor> corridor, std::vector<double> homotopy_p, std::vector<double> homotopy_q, Node robot_state)
{
    //auto corridor_start_time = std::chrono::high_resolution_clock::now();
    // Reset q_left and q_right (I did this on the python side, but I dont know why exactly, in theory I should be able to just
    // update it incrementally in a sliding window? I must have had a reason for it but ill leave it out for now).
    // Ah the reason i did this was because I was not handling the no collision case (in which we reset q_left to q_right for that bin to the max)
    // Resetting at the beginning prevents needing to do this, but I think I prefer the way im doing it here.

    // Take a subset of the p_bins based on the current robot state and our dynamic window
    std::vector<double> p_bins_subset;
    
    // Note there is probably a much better way to do this which is faster exploiting that the vector is sorted, but for now just doing this for convenience
    for (int i = 0; i < corridor->p_bins.size(); i++)
    {
      double p_bin = corridor->p_bins[i];
      if ((p_bin >= robot_state.p) && (p_bin <= robot_state.p + corridor->sliding_window_width))
      {
        p_bins_subset.push_back(p_bin);
      }
      // Exit early if the p values become larger then the window 
      if (p_bin > robot_state.p + corridor->sliding_window_width)
      {
        break;
      }
    }


    // Iterate through each of the subset of p_bins
    double p_upper;
    double p_lower;
    double q_upper;
    double q_lower;
    for (int i = 0; i < p_bins_subset.size(); i++)
    {
      
      int ind_counter = 0;

      // iterate through the current path solution (in p,q space)
      for (int j = 0; j < homotopy_p.size(); j++)
      {
        // If the point on the path is just larger then the p_bin_subset, take that point and the previous one, interpolate a q_bin value at the place
        if (homotopy_p[j] >= p_bins_subset[i])
        {
          p_upper = homotopy_p[ind_counter];
          p_lower = homotopy_p[ind_counter - 1];
          q_upper = homotopy_q[ind_counter];
          q_lower = homotopy_q[ind_counter - 1];
          break;
        }
        ind_counter = ind_counter + 1;
      }

      double q_bin = q_lower + ((p_bins_subset[i] - p_lower) / (p_upper - p_lower)) * (q_upper - q_lower);
      //CLOG(DEBUG, "path_planning.corridor_debug") << "q_bin is: " << q_bin;
      Node start = Node(p_bins_subset[i], q_bin); // starting point for collision check
      Node end_left = Node(p_bins_subset[i], corridor->q_max + 0.01); // end point for 1st collision check + a small buffer
      Node end_right = Node(p_bins_subset[i], (-1.0 * corridor->q_max - 0.01)); // end point for 2nd collision check + a small buffer
      //CLOG(DEBUG, "path_planning.corridor_debug") << "start node is p: " << start.p << " q: " << start.q;
      //CLOG(DEBUG, "path_planning.corridor_debug") << "end_left node is p: " << end_left.p << " q: " << end_left.q;

      // Note long term we need to handle special case when the start point is in a wormhole region, but for now should be fine
      //Node euclid_start = CBITPlanner::curve_to_euclid(start);

      // debug, testing to see how often the bit* point is in a collision by the time we get here (in theory this should never happen but I think it is)
      //auto test_check = discrete_collision_v2(corridor->curv_to_euclid_discretization, start, start);
      //if (test_check.bool_result == true)
      //{
      //  CLOG(ERROR, "path_planning.corridor_debug") << "Something has gone wrong, cbit path has a collision, ignoring this p_bin update";
      //  continue;
      //}
      
      Node euclid_pt = curve_to_euclid(start);
      if (costmap_col_tight(euclid_pt))
      {
          CLOG(ERROR, "path_planning.corridor_debug") << "Something has gone wrong, cbit path has a collision, ignoring this p_bin update";
          continue;
      }


      // collision check left and right using a special version of discrete_collision check
      // In this version we output both a boolean and the 1st point that comes into collision if there is one
      auto collision_check_result1 = discrete_collision_v2(corridor->curv_to_euclid_discretization, start, end_left, true);
      auto collision_check_result2 = discrete_collision_v2(corridor->curv_to_euclid_discretization, start, end_right, true);

      // if there is a collision, set q_left at the location of the current p_bin being processed to the value of q_left/q_right
      if (collision_check_result1.bool_result == true)
      {
        //CLOG(DEBUG, "path_planning.corridor_debug") << "start node is p: " << start.p << " q: " << start.q;
        //CLOG(DEBUG, "path_planning.corridor_debug") << "end_left node is p: " << end_left.p << " q: " << end_left.q;
        double q_left = collision_check_result1.col_node.q;
        auto it = find(corridor->p_bins.begin(), corridor->p_bins.end(), p_bins_subset[i]);
        if (it != corridor->p_bins.end())
        {
          int index = it - corridor->p_bins.begin();
          corridor->q_left[index] = q_left;
          CLOG(DEBUG, "path_planning.corridor_debug") << "Q_left is: " << q_left;
        }
      }
      // else set it back to the maximums
      else
      {
        double q_left = corridor->q_max;
        auto it = find(corridor->p_bins.begin(), corridor->p_bins.end(), p_bins_subset[i]);
        if (it != corridor->p_bins.end())
        {
          int index = it - corridor->p_bins.begin();
          corridor->q_left[index] = q_left;
        }
      }

      // Repeat for the other side
      
      if (collision_check_result2.bool_result == true)
      {
        //CLOG(DEBUG, "path_planning.corridor_debug") << "start node is p: " << start.p << " q: " << start.q;
        //CLOG(DEBUG, "path_planning.corridor_debug") << "end_right node is p: " << end_right.p << " q: " << end_right.q;
        double q_right = collision_check_result2.col_node.q;
        auto it = find(corridor->p_bins.begin(), corridor->p_bins.end(), p_bins_subset[i]);
        if (it != corridor->p_bins.end())
        {
          int index = it - corridor->p_bins.begin();
          corridor->q_right[index] = q_right;
          CLOG(DEBUG, "path_planning.corridor_debug") << "Q_right is: " << q_right;
        }
      }
      else
      {
        double q_right = -1.0 * corridor->q_max;
        auto it = find(corridor->p_bins.begin(), corridor->p_bins.end(), p_bins_subset[i]);
        if (it != corridor->p_bins.end())
        {
          int index = it - corridor->p_bins.begin();
          corridor->q_right[index] = q_right;
        }
      }
      
    }


    // Updating the full euclidean corridor vectors by iterating through all bins.

    // TODO: Make whether we do this a configurable parameter, I realised that I dont actually need this from the control perspective at all
    // Its purely for visualization purposes and it will waste some compute (not much though so im not too concerned for everyday use)

    // Note for convenience I can do this in a separate loop, but really we could also be doing this incrementally the same way we update
    // q_left/q_right. This requires a proper initialization of the euclid corridor in generate_pq.cpp, which is certainly possible
    // But not currently because the curve_to_euclid is not in the utils package. When I change that we can do this. Until then, brute force it is.

    // Benchmarking the compute time for this operation since its likely much less efficient then it could be with the incremental approach
    // Need to first clear out the old corridor, otherwise it just keeps stacking
    corridor->x_left.clear();
    corridor->y_left.clear();
    corridor->x_right.clear();
    corridor->y_right.clear();
    //auto corridor_start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < corridor->p_bins.size(); i++)
    {
      Node euclid_left = curve_to_euclid(Node(corridor->p_bins[i],corridor->q_left[i]));
      Node euclid_right = curve_to_euclid(Node(corridor->p_bins[i],corridor->q_right[i]));

      //CLOG(ERROR, "path_planning.corridor_debug") << "Euclid Left is x: " << euclid_left.p << " y: " << euclid_right.p;
      corridor->x_left.push_back(euclid_left.p);
      corridor->y_left.push_back(euclid_left.q);
      corridor->x_right.push_back(euclid_right.p);
      corridor->y_right.push_back(euclid_right.q);
    }



    //auto corridor_stop_time = std::chrono::high_resolution_clock::now();
    //auto duration_corridor = std::chrono::duration_cast<std::chrono::milliseconds>(corridor_stop_time - corridor_start_time);
    //CLOG(ERROR, "path_planning.corridor_debug") << "Corridor Update Time: " << duration_corridor.count() << "ms";
    
}




