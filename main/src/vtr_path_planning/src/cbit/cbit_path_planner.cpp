#include "vtr_path_planning/cbit/cbit_path_planner.hpp"

namespace {
// Function for converting Transformation matrices into se(2) [x, y, z, roll, pitch, yaw]
// Note we also might be able to do this with just lgmaths tran2vec operation?
inline std::tuple<double, double, double, double, double, double> T2xyzrpy(
    const vtr::tactic::EdgeTransform& T) {
  const auto Tm = T.matrix();
  return std::make_tuple(Tm(0, 3), Tm(1, 3), Tm(2,3), std::atan2(Tm(2, 1), Tm(2, 2)), std::atan2(-1*Tm(2, 0), sqrt(pow(Tm(2, 1),2) + pow(Tm(2, 2),2))), std::atan2(Tm(1, 0), Tm(0, 0)));
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
CBITPlanner::CBITPlanner(CBITConfig conf_in, std::shared_ptr<CBITPath> path_in, vtr::path_planning::BasePathPlanner::RobotState& robot_state, std::shared_ptr<std::vector<Pose>> path_ptr)
{ 
  // Access the pointer to memory where the final result will be stored:
  cbit_path_ptr = path_ptr;

  // Before beginning the planning phase, we need to wait for the robot to localize, and then update the goal state
  auto& chain = *robot_state.chain;
  do
  {
    CLOG(WARNING, "path_planning.teb") << "Robot is not localized, Planner not started";
  } while (chain.isLocalized() == 0);

  
  CLOG(INFO, "path_planning.teb") << "Made it inside the planner, trying to build the planning object";
  conf = conf_in;
  global_path = path_in; //TODO: I dont really like this, maybe either used shared pointers or make the arg path_in just be named global path
  p_goal = std::make_shared<Node> (global_path->p[0], 0.0);
  p_start = std::make_shared<Node> (global_path->p.back(), 0.0);

  //std::vector<double> plot_x;
  //std::vector<double> plot_y;

  InitializePlanningSpace();
  Planning(robot_state);

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
    CLOG(INFO, "path_planning.teb") << "Robot Pose: x: " << std::get<0>(robot_pose) << " y: " 
    << std::get<1>(robot_pose) << " z: " << std::get<2>(robot_pose) << " roll: " << std::get<3>(robot_pose) << " pitch: " 
    << std::get<4>(robot_pose) << " yaw: " << std::get<5>(robot_pose);
    CLOG(INFO, "path_planning.teb") << "Displaying Current Robot Transform: " << T_p_r;
    Planning();
    sleep(1);
  }

  CLOG(INFO, "path_planning.teb") << "Planner Executed successfully and found the optimal plan";
  */
}

void CBITPlanner::InitializePlanningSpace()
{
  // Process the Transforms of the Tree to define the p,q space and its corresponding euclidean poses

  // TODO: Grab transforms sequentially from beginning of the route, convert to se(3) [x, y, z, roll, pitch, yaw] vectors and store in vector of vectors discrete_path:
    // for now ill just hardcode them and assume we have in se(3) already:
    //std::vector<std::vector>
  // Create global path class, which processes the discrete path and generates the p,q space reference vectors for conversions


  // Reserve some vector space, 2000 samples should be about good for our rolling window, but need to experiment with this and set as config
  tree.V.reserve(10000);
  tree.E.reserve(10000);
  tree.QV.reserve(10000);
  tree.QE.reserve(10000);

  // Set initial cost to comes
  p_start->g_T = 0.0;
  p_start->g_T_weighted = 0.0;

  p_goal->g_T = INFINITY;
  p_goal->g_T_weighted = INFINITY;

  // Initialize tree
  tree.V.push_back(std::shared_ptr<Node> (p_start));
  samples.push_back(std::shared_ptr<Node> (p_goal));


  // Initialize obstacles
  // Long term we will probably move this to an env class
  //obs_rectangle = {{2.0, -2.0, 1, 3}, {3.0, 4.0, 3.0, 1.0}, {3.0, 8.0, 2.0, 2.0}}; // Obstacle format is {{x,y,w,h}} where x,y is the lower left corner coordinate
  //obs_rectangle = {{2.0, -2.0, 1, 3}, {3.0, 4.0, 3.0, 1.0}};
  obs_rectangle = {}; 

  // Initialize sliding window dimensions for plotting and radius expansion calc;
  sample_box_height = conf.q_max * 2.0;
  sample_box_width = conf.sliding_window_width;
}

// Main planning function
void CBITPlanner::Planning(vtr::path_planning::BasePathPlanner::RobotState& robot_state)
{
  // Grab the amount of time in ms between robot state updates
  int control_period_ms = (1.0 / conf.state_update_freq) * 1000.0;
  auto state_update_time = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(control_period_ms);

  bool repair_mode = false; // Flag for whether or not we should resume the planner in repair mode to update the tree following a state update

  // benchmarking example code
  auto start_time = std::chrono::high_resolution_clock::now();

  // Debug infinite loop for now, TODO: Need to add break conditions
  while(true)
  {

    for (int k = 0; k < conf.iter_max; k++)
    {
      // Check whether a robot state update should be applied
      // We only update the state if A: we have first found a valid initial solution, and B: if the current time has elapsed the control period
      if (conf.update_state == true && repair_mode == false)
      {
        if ((p_goal->parent != nullptr) && (std::chrono::high_resolution_clock::now() >= state_update_time))
        {
          // Update timers
          CLOG(INFO, "path_planning.teb") << "Attempting to Update Robot State";
          state_update_time = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(control_period_ms);

          // get the euclidean robot state in the world frame from vt&r

          auto& chain = *robot_state.chain; // I think we actually only ever really use the chain to make sure we are actually localized
          // Which I guess I probably should be doing (see teb code) as we probably dont want to pass the robot state unless we are
          std::tuple<double, double, double, double, double, double> robot_pose;

          const auto chain_info = getChainInfo(robot_state);
          auto [stamp, w_p_r_in_r, T_p_r, T_w_p, curr_sid] = chain_info;
          robot_pose= T2xyzrpy(T_w_p * T_p_r);
          CLOG(INFO, "path_planning.teb") << "Robot Pose: x: " << std::get<0>(robot_pose) << " y: " 
          << std::get<1>(robot_pose) << " z: " << std::get<2>(robot_pose) << " roll: " << std::get<3>(robot_pose) << " pitch: " 
          << std::get<4>(robot_pose) << " yaw: " << std::get<5>(robot_pose);
          CLOG(INFO, "path_planning.teb") << "Displaying Current Robot Transform: " << T_p_r;

          Pose se3_robot_pose = Pose(std::get<0>(robot_pose),std::get<1>(robot_pose),std::get<2>(robot_pose),std::get<3>(robot_pose),std::get<4>(robot_pose),std::get<5>(robot_pose));

          new_state = std::make_unique<Pose> (se3_robot_pose);

          // Perform a state update to convert the actual robot position to its corresponding pq space:
          UpdateState();

          CLOG(INFO, "path_planning.teb") << "Made it past updatestate";

          // Add the new goal (robot state) to the samples so it can be found again
          samples.push_back(p_goal); 

          // Find vertices in the tree which are close to the new state, then populate the vertex queue with only these values.
          tree.QV.clear();
          tree.QE.clear();
          for (int i = 0; i < tree.V.size(); i++)
          {
            if (calc_dist(*(tree.V[i]), *p_goal) <= 1.0 ) // TODO: replace magic number with a param
            {
              tree.QV.push_back(tree.V[i]);
            }
          }

          CLOG(INFO, "path_planning.teb") << "Robot State Updated Successfully";


          // TODO: Could insert code here to update the obstacles like I was doing in python if need be for demos and debugging, but this probably isnt really necessary

          // When the planner resumes, this will cause it to immediately try to rewire locally to the new robot state in a short amount of time

        }
      }


      int m;
      if (tree.QV.size() == 0 && tree.QE.size() == 0)
      {
        std::cout << "New Batch:" << std::endl;
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
          // TODO, need to display results
          std::cout << "Iteration: " << k << std::endl;
          std::cout << "Path Cost: " << p_goal->g_T_weighted << std::endl;

          // Benchmark current compute time
          auto stop_time = std::chrono::high_resolution_clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time);
          std::cout << "Batch Compute Time (ms): " << duration.count() << std::endl;


          // Extract the solution
          std::tuple<std::vector<double>, std::vector<double>> curv_path = ExtractPath();
          path_x = std::get<0>(curv_path); // p coordinates of the current path (I should probably rename, this is misleading its not x and y)
          path_y = std::get<1>(curv_path); // q coordinates of the current path (I should probably rename, this is misleading its not x and y)


          // Store the Euclidean solution in the shared pointer memory (vector of Pose classes) so it can be accessed in the CBIT class

          std::vector<Pose> euclid_path = ExtractEuclidPath();
          *cbit_path_ptr = euclid_path;


          
        }

        Prune(p_goal->g_T, p_goal->g_T_weighted);

        // Resample
        if (p_goal->g_T_weighted < INFINITY)
        {
          // sample box function here
          std::cout << "Sample Box" << std::endl;
          std::vector<std::shared_ptr<Node>> new_samples = SampleBox(m); // TODO Sample rejection
          //std::vector<std::shared_ptr<Node>> new_samples = SampleFreeSpace(m); // DEBUG ONLY!!!
          samples.insert(samples.end(), new_samples.begin(), new_samples.end());
        }
        
        else
        {
          std::cout << "Sample Free Space " << std::endl;

          std::vector<std::shared_ptr<Node>> new_samples = SampleFreeSpace(m); // TODO Pre Seeds, Sample Rejection
          samples.insert(samples.end(), new_samples.begin(), new_samples.end());

        }

        
        
        // Initialize the Vertex Queue;
        //tree.QV = tree.V; // C++ is dumb so this doesnt work with shared pointer vectors, need to do the following instead      
        for (int i = 0; i < tree.V.size(); i++)
        {
          tree.QV.push_back(std::shared_ptr<Node> (tree.V[i]));
        }

        // TODO: Dynamic Expansion radius selection:
        /*
        if (p_goal->g_T != INFINITY)
        {
          conf.initial_exp_rad = exp_radius((tree.V.size() + samples.size()), sample_box_height, sample_box_width, conf.eta);
        }
        */
        
      }

      
      // Planning loop starts here
      // I think there is some gains to be had here, we actually iterate through the vertex queue twice finding minima,
      // I think we probably could combine the 2 vertex functions into 1 cutting compute in half (we can test to see if this is a big gain or not)
      while (BestVertexQueueValue() < BestEdgeQueueValue())
      {
        //auto test = BestInVertexQueue();
        ExpandVertex(BestInVertexQueue());
      }
      
      // Generate prospective nodes
      std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> prospective_edge = BestInEdgeQueue();

      std::shared_ptr<Node> vm = std::get<0>(prospective_edge);
      std::shared_ptr<Node> xm = std::get<1>(prospective_edge);

      // TODO: Handle edge case where a perfect path results in empty queues (see python code) (i think the fix will be switching to some auto declarations)
          // - Nah, its related to finding a perfect solution, things just get stuck
      if (vm == NULL) 
      {
        break;
      }
      // Remove the edge from the queue (I think best to do this inside BestInEdgeQueue function)

      // TODO: Collision check and update tree/queues
      if (vm->g_T_weighted + calc_weighted_dist(*vm, *xm, conf.alpha) + h_estimated_admissible(*xm, *p_goal) < p_goal->g_T_weighted)
      {
        // TODO: replace these next too lines with a function which first does a collision check, returning INF if there is one
        // Then runs the calc_dist functions if there is not a collision
        //double actual_cost = calc_dist(*vm, *xm);
        //double weighted_cost = calc_weighted_dist(*vm, *xm, conf.alpha);

        double actual_cost = cost_col(obs_rectangle, *vm, *xm);
        double weighted_cost = weighted_cost_col(obs_rectangle, *vm, *xm);

        if (g_estimated_admissible(*vm, *p_start) + weighted_cost + h_estimated_admissible(*xm, *p_goal) < p_goal->g_T_weighted)
        {
          if (vm->g_T_weighted + weighted_cost < xm->g_T_weighted)
          {
            // Check if xm is in the tree, if it is, we need to do some rewiring of any other edge which has xm as an endpoint
            if (node_in_tree(*xm) == true)
            {
              // TODO: Needs some additional wormhole logic in here which I havent done yet

              // loop through edges, delete those which have the same xm end node
              int upper_edge_index = tree.E.size();
              for (int i = 0; i < upper_edge_index; i++)
              {
                if (std::get<1>(tree.E[i])->p == xm->p && std::get<1>(tree.E[i])->q == xm->q)
                {
                    // Once we process the vertex and have it ready to return, remove it from the edge queue (This is apparently a fast way of doing so)
                    
                    // NOTE: Im not sure if this actually works more than once in a loop? because I might be changing the size when I do this and index may be wrong now
                    // try it though
                    auto it = tree.E.begin() + i; // need to double check this works correctly
                    *it = std::move(tree.E.back());
                    tree.E.pop_back();

                    // NEED TO BE VERY CAREFUL WITH THE INDEX HERE, ANYTIME WE DO A DELETION WE NEED TO DECREMENT, AS WELL AS ADJUST THE UPPER ITERATOR SIZE
                    i = i - 1;
                    upper_edge_index = upper_edge_index - 1;
                }
              }
            }
            
            // Otherwise we can remove xm from the samples, add xm to the tree vertices and queue
            else
            {
              // If the end point is not in the tree, it must have come from a random sample.
              // Remove it from the samples, add it to the vertex tree and vertex queue:
              // C++ is trash so I think the only way to do this is loop through samples and check for matches
              for (int i = 0; i < samples.size(); i++)
              {
                if (samples[i]->p == xm->p && samples[i]->q == xm->q)
                {
                  auto it = samples.begin() + i; // need to double check this works correctly
                  *it = std::move(samples.back());
                  samples.pop_back();

                  break; // Once we find the one to delete, it is safe to break, there should only every be one
                }
              }
              tree.V.push_back(xm);
              tree.QV.push_back(xm);
            }

            // Set cost to comes
            // Note need to verify that updating objects like this also updates them in the trees. it does in python, but c++ is shit
            // yeah it doesnt in c++, means I may need to rework this with pointers
            xm->g_T_weighted = vm->g_T_weighted + weighted_cost;
            xm->g_T = vm->g_T + actual_cost;

            // TODO: wormhole handling

            // Generate edge, create parent chain
            tree.E.push_back(std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {vm, xm});
            

            xm->parent = vm;

            // Filter edge queue as now any other edge with worse cost heuristic can be ignored
            int upper_queue_index = tree.QE.size();
            for (int i = 0; i < upper_queue_index; i++)
            {
              Node v = *std::get<0>(tree.QE[i]);
              Node x = *std::get<1>(tree.QE[i]);

              if (x.p == xm->p && x.q == xm->q && (v.g_T_weighted + calc_weighted_dist(v, *xm, conf.alpha) >= xm->g_T_weighted))
              {
                auto it = tree.QE.begin() + i; // need to double check this works correctly
                *it = std::move(tree.QE.back());
                tree.QE.pop_back();

                // Once again, be very careful with the iteration indices when deleting
                // Need to decrement both the iterator and the total size
                i = i - 1;
                upper_queue_index = upper_queue_index - 1;
              }
            }
          }
        }
      }
      // If there is no edges in the queue which can improve the current tree, clear the queues which will consequently trigger the end of a batch
      else
      {
        tree.QV.clear();
        tree.QE.clear();
      }

      // TODO: Plotting (if we want)
      
      
    }
  }

}



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
  double lookahead = conf.sliding_window_width;
  double box_tolerance = 0.1; // Need to add a little extra height to the box when using ROC regions

  // Calculate dimensions
  double q_max = *std::max_element(path_y.begin(), path_y.end()) + box_tolerance; // apparently this function can be abit slow, so be aware
  double q_min = *std::min_element(path_y.begin(), path_y.end()) - box_tolerance;
  
  q_max = std::max(fabs(q_max), fabs(q_min));

  double p_max = p_goal->p + lookahead - padding;
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

    // TODO: Before we add the sample to the sample vector, we need to collision check it
    if (is_inside_obs(obs_rectangle, curve_to_euclid(node)))
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
  
  double p_max = p_goal->p + conf.sliding_window_width +conf.sliding_window_freespace_padding;
  double p_zero = p_goal->p - conf.sliding_window_freespace_padding;

  double q_max = conf.q_max;

  int ind = 0;
  while (ind < m)
  {
    // uniformly sample the configuration space
    double rand_p = p_zero + (rand() / (RAND_MAX / (p_max-p_zero)));
    double rand_q = (-1.0*q_max) + (rand() / (RAND_MAX / (q_max- (-1.0*q_max))));
    Node node(rand_p, rand_q);

    // Before we add the sample to the sample vector, we need to collision check it
    if (is_inside_obs(obs_rectangle, curve_to_euclid(node)))
    {
      continue;
    }
    else
    {
      new_samples.push_back(std::make_shared<Node> (node));
      ind++; // only increment if we do not have a collision
    }
  }

  // TODO: Generating Pre-seeds (note i did this super quick and dirty for a meeting crunch, need to replace this longer term)

  // hardcoded pre-seed interval for now
  int pre_seeds = abs(p_goal->p - p_start->p) / 0.25;

  // In the python version I do this line thing which is more robust, but for now im going to do this quick and dirty
  double p_step = 0.25;
  double p_val = 0.0;
  for (int i = 0; i < (pre_seeds); i++)
  {
    Node node((p_val+p_step), 0);

    // Before we add the sample to the sample vector, we need to collision check it in euclidean
    if (is_inside_obs(obs_rectangle, curve_to_euclid(node)) == false)
    {
      new_samples.push_back(std::make_shared<Node> (node));
    }

    p_val = p_val + p_step;
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
  // paper says this is to maintain uniform sample density, but practically I dont see how any tree vertex would have infinite cost to come?
  //&& (samples[i]->g_T != INFINITY)) 
  for (int i = 0; i < tree.V.size(); i++)
  {
    if (tree.V[i]->g_T_weighted == INFINITY)
    {
      samples_pruned.push_back(std::shared_ptr<Node> (tree.V[i]));
    }
  }
  // After we do both the above loops update the sample vector
  samples = samples_pruned;



  // Similar prune of the Vertex Tree
  // TODO: Wormhole accomodations
  
  std::vector<std::shared_ptr<Node>> vertex_pruned;
  for (int i = 0; i <tree.V.size(); i++)
  {
    if ((f_estimated(*tree.V[i], *p_start, *p_goal, conf.alpha) <= cost_threshold) && (tree.V[i]->g_T_weighted < INFINITY))
    {
      
      vertex_pruned.push_back(std::shared_ptr<Node> (tree.V[i]));
    }
  }
  tree.V = vertex_pruned;
  

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
void CBITPlanner::UpdateState()
{
  CLOG(INFO, "path_planning.teb") << "Made it inside Update State";
  //std::cout << "The new state is x: " << new_state->x << " y: " << new_state->y  << std::endl;


  //First calc the qmin distance to the euclidean path (borrow code from ROC generation)
  //To prevent us from having to search the entire euclidean path (and avoid crosses/loops) in the path, use the previous pose as a reference (will always be p=0)
  //and define a subset of points with a lookahead distance
  std::vector<Node> euclid_subset;
  euclid_subset.reserve((conf.roc_lookahead * conf.roc_lookahead * conf.curv_to_euclid_discretization));
  CLOG(INFO, "path_planning.teb") << "Made it to the first for loop";
  // The length of the subset is determined by the configuration parameter lookahead distance and the desired discretization
  for (double i = (*p_goal).p; i < ((*p_goal).p + conf.roc_lookahead); i += (1.0 / (conf.roc_lookahead * conf.curv_to_euclid_discretization)))
  {
    euclid_subset.push_back(curve_to_euclid(Node(i,0)));
    
  }
  CLOG(INFO, "path_planning.teb") << "Made it to q_min calc";

  // calc q_min
  double q_min = conf.q_max;
  double q;
  Node closest_pt;
  int closest_pt_ind;
  CLOG(INFO, "path_planning.teb") << "Made it to q_min calc 2";
  CLOG(INFO, "path_planning.teb") << "the new state is: X: " << new_state->x << " Y: " << new_state->y;
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

  // Once we have the closest point, we need to try to find the sign (above or below the path)
  // This is a little tricky, but I think what is reasonable is to create a plane with the closest point and its neighbours
  // Then depending on which side the new state point is of the plane, is the sign we use for q_min:
  CLOG(INFO, "path_planning.teb") << "Made it to node calc";
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
  
  q_min = q_min * q_sign;
  
  
  CLOG(INFO, "path_planning.teb") << "Made it to pose err";
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

  CLOG(INFO, "path_planning.teb") << "Made it to goal update";
  // Now update the goal and its cost to come:
  std::shared_ptr<Node> new_state_pq_ptr = std::make_shared<Node>(new_state_pq);
  p_goal = new_state_pq_ptr;
  p_goal->g_T = INFINITY;
  p_goal->g_T_weighted = INFINITY;


  std::cout << "Successfully Updated State: " << std::endl;

  // TODO: At this point we would pop out the first element of the new_state_arr so next state update we keep moving forward
  

}





double CBITPlanner::BestVertexQueueValue()
{
  if (tree.QV.size() == 0)
  {
    return INFINITY;
  }
  // Iterate through all vertices in the queue, find the minimum cost heurstic
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

  return min_vertex_cost;
}



double CBITPlanner::BestEdgeQueueValue()
{
  if (tree.QE.size() == 0)
  {
    return INFINITY;
  }

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
    
  return min_edge_cost;
}



std::shared_ptr<Node> CBITPlanner::BestInVertexQueue()
{
  if (tree.QV.size() == 0)
  {
    std::cout << "Vertex Queue is Empty!" << std::endl;
    //return; // TODO: Need to fix these returns, I can potentially use all auto types to do something similar to what I did in python
    // But I think longer term I should find a better solution. This case only occurs if a solution cannot be found, which for now can be ignored
  }

  // Loop through the vertex queue, select the vertex node with the lowest cost to come + heuristic dist to goal

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
      tree.QE.push_back(std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {(v), (samples[i])});

    }
  }

  
  // find nearby vertices and filter by heuristic potential
  for (int i = 0; i < tree.V.size(); i++)
  {
    //auto test1 = calc_dist(*tree.V[i], *v);
    //auto test2 = (g_estimated(*v, *p_start, conf.alpha) + calc_weighted_dist(*v, *tree.V[i], conf.alpha) + h_estimated(*tree.V[i], *p_goal, conf.alpha));
    //auto test3 = (v->g_T_weighted + calc_weighted_dist(*v, *tree.V[i], conf.alpha));
    if ((calc_dist(*tree.V[i], *v) <= conf.initial_exp_rad) && ((g_estimated_admissible(*v, *p_start) + calc_weighted_dist(*v, *tree.V[i], conf.alpha) + h_estimated_admissible(*tree.V[i], *p_goal)) < p_goal->g_T_weighted) && ((v->g_T_weighted + calc_weighted_dist(*v, *tree.V[i], conf.alpha)) < tree.V[i]->g_T_weighted))
    {
      // Also check whether the edge is in the tree or not
      if (edge_in_tree(*v, *tree.V[i]) == false)
      {
        // If all conditions satisfied, add the edge to the queue
        //tree.V[i]->g_T = INFINITY; I think these two lines were a mistake, should not be here
        //tree.V[i]->g_T_weighted = INFINITY;
        tree.QE.push_back(std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {(v), (tree.V[i])});

      }
      
    }
  }
}


std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> CBITPlanner::BestInEdgeQueue()
{
  if (tree.QE.size() == 0) // need to handle a case where the return path is 100% optimal in which case things get stuck and need ot be flagged to break
  {
    std::cout << "Edge Queue is Empty! Optimal Solution Found" << std::endl;
    return std::tuple<std::shared_ptr<Node>, std::shared_ptr<Node>> {NULL, NULL};

  }


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
}


std::tuple<std::vector<double>, std::vector<double>> CBITPlanner::ExtractPath()
{
  Node node = *p_goal;
  
  std::vector<double> path_p = {node.p};
  std::vector<double> path_q = {node.q};

  while(node.parent != nullptr)
  {
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
    }

  }
  

  // DEBUG Temporarily reshaping the output into the form we need (vector of poses)
  // TODO: Longer term I need to rework this whole function to output the poses directly in the loops so we dont have to redundently loop again
  // Also need to implement a new curve_to_euclid_se3 function which converts to 3D Poses instead of nodes which we will only use here
  std::vector<Pose> pose_vector;
  for (int i = 0; i<path_x.size(); i++)
  {
    pose_vector.push_back(Pose(path_x[i], path_y[i], path_z[i], 0.0, 0.0, 0.0));// Temporarily set pose with x,y coordinate, no orientation
  }

  return pose_vector;
}





// Function which takes in a beginning vertex v, and an end vertex x, and checks whether its in the tree or not already
// C++ sucks so this is actually very annoying to do
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
  double z_i = pose_c.z + cos(pose_c.yaw)*q_val;

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

bool CBITPlanner::discrete_collision(std::vector<std::vector<double>> obs, double discretization, Node start, Node end)
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
        //Node euclid_pt = curv_pt; // DEBUG DO NOT LEAVE THIS HERE, NEED TO REPLACE WITH COLLISION CHECK FUNCTION
        Node euclid_pt = curve_to_euclid(curv_pt);
        if (is_inside_obs(obs, euclid_pt))
        {
            return true;
        }
    }

    return false;
}




