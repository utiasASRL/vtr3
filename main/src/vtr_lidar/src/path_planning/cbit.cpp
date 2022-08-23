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
 * \file teb_path_planner.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/path_planning/cbit.hpp"

#include "vtr_lidar/cache.hpp"


namespace vtr {
namespace lidar {

// this should be inherited from the cbit class, shouldnt need to define here

namespace {
// Function for converting Transformation matrices into se(2) [x, y, z, roll, pitch, yaw]
// Note we also might be able to do this with just lgmaths tran2vec operation?
inline std::tuple<double, double, double, double, double, double> T2xyzrpy(
    const tactic::EdgeTransform& T) {
  const auto Tm = T.matrix();
  return std::make_tuple(Tm(0, 3), Tm(1, 3), Tm(2,3), std::atan2(Tm(2, 1), Tm(2, 2)), std::atan2(-1*Tm(2, 0), sqrt(pow(Tm(2, 1),2) + pow(Tm(2, 2),2))), std::atan2(Tm(1, 0), Tm(0, 0)));
}
}


// Configure the class as a ROS2 node, get configurations from the ros parameter server
auto LidarCBIT::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();
  // Originally I thought I could get rid of this, but it is actually very important with how the codebase is setup.
  // The config class above is apart of the Base Planner class, which declares the default control period as 0. So if we dont update it, we have alot of problems
  config->control_period = (unsigned int)node->declare_parameter<int>(prefix + ".control_period", config->control_period);

  // robot configuration
  config->robot_model = node->declare_parameter<std::string>(prefix + ".teb.robot_model", config->robot_model);
  config->robot_radius = node->declare_parameter<double>(prefix + ".teb.robot_radius", config->robot_radius);

  // CBIT Configs
  // This is how parameters should be updated from the parameter server. prefix is hardcoded to path_planning in the header, tabs are represented by periods "."
  
  // ENVIRONEMNT:
  config->obs_padding = node->declare_parameter<double>(prefix + ".cbit.obs_padding", config->obs_padding);
  config->curv_to_euclid_discretization= node->declare_parameter<int>(prefix + ".cbit.curv_to_euclid_discretization", config->curv_to_euclid_discretization);
  config->sliding_window_width = node->declare_parameter<double>(prefix + ".cbit.sliding_window_width", config->sliding_window_width);
  config->sliding_window_freespace_padding = node->declare_parameter<double>(prefix + ".cbit.sliding_window_freespace_padding", config->sliding_window_freespace_padding);
  config->corridor_resolution = node->declare_parameter<double>(prefix + ".cbit.corridor_resolution", config->corridor_resolution);
  config->state_update_freq = node->declare_parameter<double>(prefix + ".cbit.state_update_freq", config->state_update_freq);
  config->update_state = node->declare_parameter<bool>(prefix + ".cbit.update_state", config->update_state);
  config->rand_seed = node->declare_parameter<int>(prefix + ".cbit.rand_seed", config->rand_seed);
  
  // PLANNER TUNING PARAMS:
  config->initial_samples = node->declare_parameter<int>(prefix + ".cbit.initial_samples", config->initial_samples);
  config->batch_samples = node->declare_parameter<int>(prefix + ".cbit.batch_samples", config->batch_samples);
  config->pre_seed_resolution = node->declare_parameter<double>(prefix + ".cbit.pre_seed_resolution", config->pre_seed_resolution);
  config->alpha = node->declare_parameter<double>(prefix + ".cbit.alpha", config->alpha);
  config->q_max = node->declare_parameter<double>(prefix + ".cbit.q_max", config->q_max);
  config->frame_interval = node->declare_parameter<int>(prefix + ".cbit.frame_interval", config->frame_interval); // going to get rid of this
  config->iter_max = node->declare_parameter<int>(prefix + ".cbit.iter_max", config->iter_max); // going to get rid of this
  config->eta = node->declare_parameter<double>(prefix + ".cbit.eta", config->eta);
  config->rad_m_exhange = node->declare_parameter<double>(prefix + ".rad_m_exhange", config->rad_m_exhange);
  config->initial_exp_rad = node->declare_parameter<double>(prefix + ".cbit.initial_exp_rad", config->initial_exp_rad);
  config->extrapolation = node->declare_parameter<bool>(prefix + ".cbit.extrapolation", config->extrapolation);

  // Removed alot of the other config stuff here, I think it was specifically for dynamic reconfigure, which atleast for cbit, I dont think should be dynamically reconfigurable for now
  return config;
}

// Declare class as inherited from the BasePathPlanner
LidarCBIT::LidarCBIT(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const Callback::Ptr& callback)
    : vtr::path_planning::CBIT(config, robot_state, callback), config_(config) {
  CLOG(ERROR, "path_planning.cbit") << "Constructing the LidarCBIT Class";
  
  // I dont think we need any of this, should all be inherited
  /*
  const auto node = robot_state->node.ptr();
  // Initialize the shared pointer the output of the planner
  cbit_path_ptr = std::make_shared<std::vector<Pose>> (cbit_path);
  // Might want to reserve a good chunk of memory for the vector here too? tbd, I think compute would be fairly marginal for the size of our paths
  
  // Create publishers
  tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("planning_path", 10);
  test_pub_ = node->create_publisher<std_msgs::msg::String>("test_string", 10);

  // Updating cbit_configs
  // Environment
  cbit_config.obs_padding = config->obs_padding;
  cbit_config.curv_to_euclid_discretization = config->curv_to_euclid_discretization;
  cbit_config.sliding_window_width = config->sliding_window_width;
  cbit_config.sliding_window_freespace_padding = config->sliding_window_freespace_padding;
  cbit_config.corridor_resolution = config->corridor_resolution;
  cbit_config.state_update_freq = config->state_update_freq;
  cbit_config.update_state = config->update_state;
  cbit_config.rand_seed = config->rand_seed;
  
  // Planner Tuning Params
  cbit_config.initial_samples = config->initial_samples;
  cbit_config.batch_samples = config->batch_samples;
  cbit_config.pre_seed_resolution = config->pre_seed_resolution;
  cbit_config.alpha = config->alpha;
  cbit_config.q_max = config->q_max;
  cbit_config.frame_interval = config->frame_interval;
  cbit_config.iter_max = config->iter_max;
  cbit_config.eta = config->eta;
  cbit_config.rad_m_exhange = config->rad_m_exhange;
  cbit_config.initial_exp_rad = config->initial_exp_rad;
  cbit_config.extrapolation = config->extrapolation;
  
  // Misc
  cbit_config.incremental_plotting = config->incremental_plotting;
  cbit_config.plotting = config->plotting;
  */
}

LidarCBIT::~LidarCBIT() { stop(); }

void LidarCBIT::initializeRoute(RobotState& robot_state0) {
  /// \todo reset any internal state
  //CLOG(INFO, "path_planning.cbit") << "We have just tried to run the lidarcbit initialize route";
  //CLOG(INFO, "path_planning.teb") << "The state_update_freq config param from ROS is: " << config_->state_update_freq; // This is how you would reference a param, need to use config_ not config!!!
  //CLOG(INFO, "path_planning.teb") << "The state_update_freq config param from ROS is: " << cbit_config.state_update_freq; 
  //CLOG(INFO, "path_planning.teb") << "The alpha config param from ROS is: " << cbit_config.alpha; 

  auto& robot_state = dynamic_cast<LidarOutputCache&>(robot_state0);
  auto& chain = *robot_state.chain;



  // Initially when the initializeRoute is called in the base planner, the chain has not yet been localized, so we cannot see the frames yet.
  // We need to loop for awhile until the chain localizes doing nothing on this thread (just takes a second or two)
  while (!chain.isLocalized()) 
  {
    // Longer term maybe want a timeout counter here or something? The rest of the stack may well handle it though
  }


  // Begin trying to process the route
  lgmath::se3::TransformationWithCovariance teach_frame;
  std::tuple<double, double, double, double, double, double> se3_vector;
  Pose se3_pose;
  std::vector<Pose> euclid_path_vec; // Store the se3 frames w.r.t the initial world frame into a path vector
  euclid_path_vec.reserve(chain.size());
  // Loop through all frames in the teach path, convert to euclidean coords w.r.t the first frame and store it in a cbit Path class (vector of se(3) poses)
  for (size_t i = 0; i < chain.size()-1; i++)
  {
    teach_frame = chain.pose(i);
    se3_vector = T2xyzrpy(teach_frame);
    se3_pose = Pose(std::get<0>(se3_vector), std::get<1>(se3_vector), std::get<2>(se3_vector), std::get<3>(se3_vector), std::get<4>(se3_vector), std::get<5>(se3_vector));
    euclid_path_vec.push_back(se3_pose);
    // Debug info to check to see that the path looks realistic
    /*
    CLOG(INFO, "path_planning.teb") << "My se(3) vector conversion: x: " << std::get<0>(se3_vector) << " y: " 
    << std::get<1>(se3_vector) << " z: " << std::get<2>(se3_vector) << " roll: " << std::get<3>(se3_vector) << " pitch: " 
    << std::get<4>(se3_vector) << " yaw: " << std::get<5>(se3_vector);
    */

    // Was trying to see here if I could use lgmaths built ins for doing this instead of doing my own T2xyzrpy function, but it was not agreeing in all cases
    // and I know that my custom function was giving correct output on the offline VTR3 tutorial dataset, so might ignore using this (its a one time thing anyways)
    //CLOG(INFO, "path_planning.teb") << "lgmath se(3) vector conversion: " << next_frame.vec();
  }

  // Create the path class object (Path preprocessing)
  CBITPath global_path(cbit_config, euclid_path_vec);

  // Make a pointer to this path
  std::shared_ptr<CBITPath> global_path_ptr = std::make_shared<CBITPath>(global_path);

  CLOG(INFO, "path_planning.cbit") << "Path has been pre-processed, Attempting to instantiate the Planner";


  /*
  CLOG(INFO, "path_planning.teb") << "Trying to get the costmap";
  //auto& robot_state_test = dynamic_cast<vtr::lidar::LidarOutputCache&>(robot_state); // I think this is an important line which allows us to use the correct output cache class
  auto change_detection_costmap_ref = robot_state.change_detection_costmap.locked();
  auto& change_detection_costmap = change_detection_costmap_ref.get();
  CLOG(INFO, "path_planning.teb") << "Costmap got got";
  if (change_detection_costmap.valid()) 
  {
    CLOG(INFO, "path_planning.teb") << "Costmap is valid, doing other stuff";
    // update change detection result
    const auto costmap_sid = change_detection_costmap->vertex_sid();
    const auto costmap_T_vertex_this =change_detection_costmap->T_vertex_this();
    //auto& chain = *robot_state_test.chain;
    //const auto T_start_vertex = chain.pose(costmap_sid);
    //CLOG(INFO, "path_planning.teb") << "The transform from costmap to robot is:" << T_start_vertex;
  }
  */




  // instantiate the planner
  //CLOG(ERROR, "path_planning.cbit") << "The pointer address (before visualizing): " << cbit_path_ptr;
  CBITPlanner cbit(cbit_config, global_path_ptr, robot_state, cbit_path_ptr, costmap_ptr);

  CLOG(INFO, "path_planning.cbit") << "Planner Successfully Created and resolved, end of initializeRoute function";
}

// TODO, experimental MPC implementation. For now I implement it directly inside here
// But longer term I think the move is to run an inherited mpc function inside of this computecommand function
// This way, mpc and the base version of the planner can be running whether we use the no obs stereo version or the full lidar version
auto LidarCBIT::computeCommand(RobotState& robot_state0) -> Command {
  auto& robot_state = dynamic_cast<LidarOutputCache&>(robot_state0);
  auto& chain = *robot_state.chain;
  if (!chain.isLocalized()) {
    CLOG(WARNING, "path_planning.cbit") << "Robot is not localized, command to stop the robot";
    return Command();
  }
  
  // Experimental:
  // - Update the CBIT costmap object with the current occupancy grid map of obstacles and robot to costmap transform so it is available to use by the planner

  //CLOG(INFO, "path_planning.teb") << "Trying to get the costmap";
  //auto& robot_state_test = dynamic_cast<vtr::lidar::LidarOutputCache&>(robot_state); // I think this is an important line which allows us to use the correct output cache class
  auto change_detection_costmap_ref = robot_state.change_detection_costmap.locked();
  auto& change_detection_costmap = change_detection_costmap_ref.get();
  //CLOG(INFO, "path_planning.teb") << "Costmap got got";
  if (change_detection_costmap.valid()) 
  {
    //CLOG(INFO, "path_planning.teb") << "Costmap is valid, doing other stuff";
    // update change detection result
    const auto costmap_sid = change_detection_costmap->vertex_sid();
    const auto costmap_T_vertex_this =change_detection_costmap->T_vertex_this();
    //auto& chain = *robot_state_test.chain;
    const auto T_start_vertex = chain.pose(costmap_sid);
    //CLOG(INFO, "path_planning.teb") << "The transform from costmap to robot is:" << T_start_vertex;
    
    // This is the important line, this is what is grabbing the data from the grid, and returning an unordered dicrete grid map which we can use for collision checking
    vtr::lidar::BaseCostMap::XY2ValueMap obs_map = change_detection_costmap->filter(0.01); //todo, there is probably a param here we need to retrieve for this
    //CLOG(ERROR, "path_planning.teb") << "the size of the map is: " << obs_map.size();


    // debug example code to visualize all the obstacle key value pairs
    // I should be able to use a filtered map like above to directly do collision checking on (instead of working with rectangles or anything)
    /*
    std::vector<std::pair<float, float>> keys;
    keys.reserve(obs_map.size());
    std::vector<float> vals;
    vals.reserve(obs_map.size());
    for(auto kv : obs_map) {
      keys.push_back(kv.first);
      vals.push_back(kv.second);  
    } 
    CLOG(ERROR, "path_planning.teb") << "Displaying all Keys: " << keys;
    CLOG(ERROR, "path_planning.teb") << "Displaying all Values: " << vals;
    */
    

    // Updating the costmap pointer

    CLOG(INFO, "path_planning.teb") << "Updating Costmap SID to: " <<change_detection_costmap->vertex_sid();
    costmap_ptr->obs_map = obs_map;
    // Store the transform T_c_w (from costmap to world)
    costmap_ptr->T_c_w = T_start_vertex.inverse(); // note that T_start_vertex is T_w_c if we want to bring keypoints to the world frame
    // Store the grid resoltuion
    costmap_ptr->grid_resolution = change_detection_costmap->dl();

    // Experimental: Storing sequences of costmaps for temporal filtering purposes
    // For the first x iterations, fill the obstacle vector (right now 5 is a magic num that could be a config param)
    if (costmap_ptr->obs_map_vect.size() < 5)
    {
      costmap_ptr->obs_map_vect.push_back(obs_map);
      costmap_ptr->T_c_w_vect.push_back(costmap_ptr->T_c_w);
    }
    // After that point, we then do a sliding window using shift operations, moving out the oldest map and appending the newest one
    else
    {
      for (int i = 0; i < (5-1); i++)
      {
        costmap_ptr->obs_map_vect[i] = costmap_ptr->obs_map_vect[i + 1];
        costmap_ptr->T_c_w_vect[i] = costmap_ptr->T_c_w_vect[i + 1];
      }
      costmap_ptr->obs_map_vect[4] = obs_map;
      costmap_ptr->T_c_w_vect[4] = costmap_ptr->T_c_w ;
    }


  //CLOG(DEBUG, "path_planning.teb") << "Now that we made the pointers, try to display them: " << costmap_ptr->obs_map_ptr;
  //CLOG(DEBUG, "path_planning.teb") << "Now that we made the pointers, try to display them: " << costmap_ptr->T_r_costmap_ptr;

  }
  

  // retrieve info from the localization chain
  const auto chain_info = getChainInfo(robot_state);
  auto [stamp, w_p_r_in_r, T_p_r, T_w_p, curr_sid] = chain_info;
  //CLOG(INFO, "path_planning.teb") << "stamp " << stamp;
  //CLOG(INFO, "path_planning.teb") << "w_p_r_in_r: " << w_p_r_in_r;
  //CLOG(INFO, "path_planning.teb") << "T_p_r: " << T_p_r;
  //CLOG(INFO, "path_planning.teb") << "T_w_p: " << T_w_p;
  //CLOG(INFO, "path_planning.cbit") << "The cbit stamp is:  " << stamp;
  //CLOG(INFO, "path_planning.cbit") << "The cbit current Time is: " << now();
  //CLOG(INFO, "path_planning.cbit") << "The cbit current time my way is: " << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  
  // Attempt a test rviz publish
  //CLOG(ERROR, "path_planning.cbit") << "The pointer size: " << (*cbit_path_ptr).size();
  //CLOG(ERROR, "path_planning.cbit") << "The pointer address: " << cbit_path_ptr;
  std::string test_string = "This is a test message!";
  // Dont visualize unless we are both localized and an initial solution is found
  if ((*cbit_path_ptr).size() != 0)
  {
    //CLOG(ERROR, "path_planning.cbit") << "Trying to visualize on the lidarcbit side";
    visualize(test_string, stamp, T_w_p, T_p_r);
    // Testing that we are receiving the most up to date output plans
    //CLOG(INFO, "path_planning.cbit") << "The first pose is x: " << (*cbit_path_ptr)[0].x << " y: " << (*cbit_path_ptr)[0].y << " z: " << (*cbit_path_ptr)[0].z;
  }






  //TODO: EXPERIMENTAL MPC IMPLEMENTATION, MOVE ALL THIS TO ITS OWN FILE
  // Conduct an MPC cycle

  // PSEUDOCODE

  // HARDCODED INITIALIZATIONS (TODO: Move these to configs)
  int K = 10;
  double DT = 0.2;
  double VF = 1.0;

  // Kinematic projection Matrix for Unicycle Model (note its -1's because our varpi is of a weird frame)

  Eigen::Matrix<double, 6, 2> P_tran;
  P_tran << -1, 0,
            0, 0,
            0, 0,
            0, 0,
            0, 0,
            0, -1;

  //CLOG(DEBUG, "mpc.cbit") << "Double Checking Projection Matrix: P_tran = " << P_tran;

  // Setup shared loss functions and noise models
  const auto sharedLossFunc = steam::L2LossFunc::MakeShared();

  // Pose Covariance Weights
  Eigen::Matrix<double, 6, 6> pose_noise_vect;
  pose_noise_vect << 1, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0,
                    0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 1;

  const auto sharedPoseNoiseModel =
      steam::StaticNoiseModel<6>::MakeShared(pose_noise_vect);

  // Disturbance Velocity Covariance
  Eigen::Matrix<double, 2, 2> vel_noise_vect;
  vel_noise_vect << 1000, 0,
                    0, 1000;

  const auto sharedVelNoiseModel =
      steam::StaticNoiseModel<2>::MakeShared(vel_noise_vect);

  // Kinematics Covariance Weights
  Eigen::Matrix<double, 6, 6> kin_noise_vect;
  kin_noise_vect << 0.001, 0, 0, 0, 0, 0,
                    0, 0.001, 0, 0, 0, 0,
                    0, 0, 0.001, 0, 0, 0,
                    0, 0, 0, 0.001, 0, 0,
                    0, 0, 0, 0, 0.001, 0,
                    0, 0, 0, 0, 0, 0.001;

  const auto sharedKinNoiseModel =
      steam::StaticNoiseModel<6>::MakeShared(kin_noise_vect);


  // Initialize the Current Robot State in the world frame and current velocity
  Eigen::Matrix4d T0;
  T0 << 1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1;
  lgmath::se3::Transformation T0_1 = lgmath::se3::Transformation(T0);
  lgmath::se3::Transformation T0_2 = lgmath::se3::Transformation(T_w_p * T_p_r);
  std::tuple<double, double, double, double, double, double> robot_pose = T2xyzrpy(T0_2);

  Eigen::Vector2d v0(0.0, 0.0);

  CLOG(DEBUG, "mpc.cbit") << "MPC TESTING:";
  CLOG(DEBUG, "mpc.cbit") << "The Spoofed Robot State is: " << T0_1;
  CLOG(DEBUG, "mpc.cbit") << "The Current Robot State Using Direct Robot Values is: : " << T0_2;
  // Need to also invert the robot state to make it T_vi instead of T_iv
  lgmath::se3::Transformation T0_2_inv = T0_2.inverse();
  CLOG(DEBUG, "mpc.cbit") << "The Inverted Current Robot State Using Direct Robot Values is: : " << T0_2_inv;



  // Generate STEAM States for the velocity vector and SE3 state transforms
  std::vector<lgmath::se3::Transformation> pose_states;
  std::vector<Eigen::Matrix<double,2,1>> vel_states;
  
  // Pushback the initial states (current robot state)
  pose_states.push_back(T0_2_inv); // Change this to T0_2 when implementing on robot, T0_1 for debug
  //vel_states.push_back(std::vector<double> {0.0, 0.0}); //I think a single line way t odo this is something like Eigen::Matrix<double, 2, 1>::Zero()
  vel_states.push_back(v0); 
 
  // Set the remaining states
  for (int i=0; i<K-1; i++)
  {
    pose_states.push_back(lgmath::se3::Transformation());
    vel_states.push_back(v0);
  }

  // Create Steam states
  std::vector<steam::se3::SE3StateVar::Ptr> pose_state_vars;
  std::vector<steam::vspace::VSpaceStateVar<2>::Ptr> vel_state_vars;
  
  for (int i = 0; i < K; i++)
  {
    pose_state_vars.push_back(steam::se3::SE3StateVar::MakeShared(pose_states[i]));
    vel_state_vars.push_back(steam::vspace::VSpaceStateVar<2>::MakeShared(vel_states[i])); 
  }

  // Lock the first (current robot) state from being able to be modified during the optimization
  pose_state_vars[0]->locked() = true;


  // Take in the current euclidean path solution from the cbit planner in the world frame, the current robot state, and determine
  // which measurements we wish to track to follow the path at the desired target velocity
  // The current path solution can be retrieved by accessing the cbit_path_ptr variable
  CLOG(DEBUG, "mpc.cbit") << "Reference Path Points:";
  std::vector<lgmath::se3::Transformation> measurements;
  double starting_dist = INFINITY;
  double new_dist;
  double dx;
  double dy;
  double delta_dist = 0;
  double index_counter = 0;

  // Find closest point on the path to the current state
  while (delta_dist >= 0)
  {
    CLOG(DEBUG, "mpc.cbit") << "x: " << (*cbit_path_ptr)[index_counter].x << "y: " << (*cbit_path_ptr)[index_counter].y << "z: " << (*cbit_path_ptr)[index_counter].z;
    dx = (*cbit_path_ptr)[index_counter].x - std::get<0>(robot_pose);
    dy = (*cbit_path_ptr)[index_counter].y - std::get<1>(robot_pose);
    new_dist = sqrt((dx * dx) + (dy * dy));
    delta_dist = starting_dist - new_dist;
    CLOG(DEBUG, "mpc.cbit") << "Dist to Pt: " << new_dist;
    CLOG(DEBUG, "mpc.cbit") << "Delta Dist: " << delta_dist;
    if (delta_dist >= 0)
    {
      starting_dist = new_dist;
      index_counter = index_counter + 1;
    }
    else
    {
      CLOG(DEBUG, "mpc.cbit") << "Delta Dist Negative, Return i = " << index_counter-1;
      index_counter = index_counter - 1;
    }
  }

  // Keep iterating through the rest of the path, storing points in the path as measurements if they maintain an approximate
  // forward path velocity of VF (//TODO, may need to also interpolate in some instances if we want to be very particular)
  for (int i = index_counter; i < (*cbit_path_ptr).size(); i++)
  //for (int i = index_counter; i < K+index_counter; i++) 
  {
    // Pseudocode:
    // 1. Loop through the entire path starting from the first index 0
    // 2. First we need to find the point on the path that is closest to the current robot pose
    //      - I think the easiest way to do this is compute the dist to the index 0 point, then compare to distance for index 1, etc
    //      - Keep doing this so long as the distance continues to decrease
    // 3. Then we keep iterating through the path, and select points (or interpolate some) which make forward progress along the path
    //    While also maintaining the desired forward path velocity given DT and VF. Note that we also need to approximate orientation
    //    based on the vector formed between successive points
    // 4. Break early once we have enough measurements for the horizon selected.
    

    // The first iteration we need to add the closest point to the initial position as a measurement
    // Subesequent iterations we want to select points on the path to track carefully based on the desired velocity we want to meet.
    
    // Reset the measurement distance
    double delta_dist = 0.0;
    if (index_counter != i)
    {
      // scan the path into the future until we proceed approximately VF*DT meters forward longitudinally long the path
      // The resulting indice of the path will be the one we use for the next measurement
      while (delta_dist <= (VF*DT))
      {
        
        double prev_x = (*cbit_path_ptr)[i-1].x;
        double prev_y = (*cbit_path_ptr)[i-1].y;
        double next_x = (*cbit_path_ptr)[i].x;
        double next_y = (*cbit_path_ptr)[i].y;
        delta_dist = delta_dist + sqrt(((next_x-prev_x) * (next_x-prev_x)) + ((next_y-prev_y) * (next_y-prev_y)));
        i = i + 1;
      }

      i = i-1; // TODO maybe cleanup this logic abit, Im not seeing another way to do this at the moment
      // With the above setup, pretty sure the resulting i will be 1 too far when we break the loop, so we need to decrement it once at the end
    }




    // Derive a yaw direction for each point based on the vector formed between the current point and subsequent point on the path
    double yaw = std::atan2(((*cbit_path_ptr)[i+1].y - (*cbit_path_ptr)[i].y), ((*cbit_path_ptr)[i+1].x - (*cbit_path_ptr)[i].x));
    CLOG(DEBUG, "mpc.cbit") << "The yaw of the path pt is: " << yaw;

    // Generate a reference Transformation matrix (ignores roll/pitch)
    Eigen::Matrix4d T_ref;
    T_ref << std::cos(yaw),-1*std::sin(yaw),0,(*cbit_path_ptr)[i].x,
             std::sin(yaw),   std::cos(yaw),0,(*cbit_path_ptr)[i].y,
             0,               0,            1,(*cbit_path_ptr)[i].z,
             0,               0,            0,                    1;
    T_ref = T_ref.inverse().eval();

    measurements.push_back(lgmath::se3::Transformation(T_ref));

    // Early break condition when the number of measurements we need is satisfied based on the horizon
    if (measurements.size() == K)
    {
      break;
    }
    // TODO handle end of path case => will want to repeat the final measurements and turn problem into a point stabilization MPC.

  }




  // Debug hardcoded reference measurements
  /*
  std::vector<lgmath::se3::Transformation> measurements;
  for (int i = 0; i < K; i++)
  {
    Eigen::Matrix4d T_ref;
    T_ref << 1,0,0,i,
             0,1,0,i,
             0,0,1,0,
             0,0,0,1;
    T_ref = T_ref.inverse().eval();
    
    
    measurements.push_back(lgmath::se3::Transformation(T_ref));
  }
  */
  // End of debug hardcoded reference measurements






  // Setup the optimization problem
  steam::OptimizationProblem opt_problem;
  for (int i=0; i<K; i++)
  {
    opt_problem.addStateVariable(pose_state_vars[i]);
    opt_problem.addStateVariable(vel_state_vars[i]);
  }

  // Generate the cost terms using combinations of the builtin steam evaluators
  for (int i = 0; i < K; i++)
  {
    // Pose Error
    const auto pose_error_func = steam::se3::SE3ErrorEvaluator::MakeShared(pose_state_vars[i], measurements[i]);
    const auto pose_cost_term = steam::WeightedLeastSqCostTerm<6>::MakeShared(pose_error_func, sharedPoseNoiseModel, sharedLossFunc);
    opt_problem.addCostTerm(pose_cost_term);

    // Non-Zero Velocity Penalty
    const auto vel_cost_term = steam::WeightedLeastSqCostTerm<2>::MakeShared(vel_state_vars[i], sharedVelNoiseModel, sharedLossFunc);
    opt_problem.addCostTerm(vel_cost_term);

    // TODO: Kinematics (Not quite working yet but its close)
    if (i < (K-1))
    {
      const auto lhs = steam::se3::ComposeInverseEvaluator::MakeShared(pose_state_vars[i+1], pose_state_vars[i]);
      
      //debug:
      //const auto vel_proj = steam::vspace::MatrixMultTest::MakeShared(vel_state_vars[i], P_tran); // TODO, I guess this version of steam doesnt have this one, will need to do it myself
      
      const auto vel_proj = steam::vspace::MatrixMultEvaluator<2>::MakeShared(vel_state_vars[i], P_tran); // TODO, I guess this version of steam doesnt have this one, will need to do it myself
      const auto scaled_vel_proj = steam::vspace::ScalarMultEvaluator<6>::MakeShared(vel_proj, DT);
      const auto rhs = steam::se3::ExpMapEvaluator::MakeShared(scaled_vel_proj);
      const auto kin_error_func = steam::se3::LogMapEvaluator::MakeShared(steam::se3::ComposeInverseEvaluator::MakeShared(lhs, rhs));
      const auto kin_cost_term = steam::WeightedLeastSqCostTerm<6>::MakeShared(kin_error_func, sharedKinNoiseModel, sharedLossFunc);
      opt_problem.addCostTerm(kin_cost_term);
    }


  }

  // Solve the optimization problem with GuassNewton solver
  using SolverType = steam::VanillaGaussNewtonSolver;
  // Initialize parameters (enable verbose mode)
  SolverType::Params params;
  params.verbose = true; // Makes the output display

  SolverType solver(&opt_problem, params);
  solver.optimize();

  CLOG(DEBUG, "mpc.cbit") << "Trying to Display the Optimization Results";
  CLOG(DEBUG, "mpc.cbit") << "The First State is: " << pose_state_vars[0]->value().inverse();
  CLOG(DEBUG, "mpc.cbit") << "The Second State is: " << pose_state_vars[1]->value().inverse();
  CLOG(DEBUG, "mpc.cbit") << "The Third State is: " << pose_state_vars[2]->value().inverse();
  CLOG(DEBUG, "mpc.cbit") << "The Forth State is: " << pose_state_vars[3]->value().inverse();
  CLOG(DEBUG, "mpc.cbit") << "The Fifth State is: " << pose_state_vars[4]->value().inverse();
  CLOG(DEBUG, "mpc.cbit") << "The Sixth State is: " << pose_state_vars[5]->value().inverse();
  CLOG(DEBUG, "mpc.cbit") << "The Seventh State is: " << pose_state_vars[6]->value().inverse();
  CLOG(DEBUG, "mpc.cbit") << "The Eighth State is: " << pose_state_vars[7]->value().inverse();
  CLOG(DEBUG, "mpc.cbit") << "The Ninth State is: " << pose_state_vars[8]->value().inverse();
  CLOG(DEBUG, "mpc.cbit") << "The Tenth State is: " << pose_state_vars[9]->value().inverse();
  CLOG(DEBUG, "mpc.cbit") << "The First Velocity is: " << vel_state_vars[0]->value();
  CLOG(DEBUG, "mpc.cbit") << "The Second Velocity is: " << vel_state_vars[1]->value();
  CLOG(DEBUG, "mpc.cbit") << "The Third Velocity is: " << vel_state_vars[2]->value();
  CLOG(DEBUG, "mpc.cbit") << "The Forth Velocity is: " << vel_state_vars[3]->value();
  CLOG(DEBUG, "mpc.cbit") << "The Fifth Velocity is: " << vel_state_vars[4]->value();
  CLOG(DEBUG, "mpc.cbit") << "The Sixth Velocity is: " << vel_state_vars[5]->value();
  CLOG(DEBUG, "mpc.cbit") << "The Seventh Velocity is: " << vel_state_vars[6]->value();
  CLOG(DEBUG, "mpc.cbit") << "The Eighth Velocity is: " << vel_state_vars[7]->value();
  CLOG(DEBUG, "mpc.cbit") << "The Ninth Velocity is: " << vel_state_vars[8]->value();
  CLOG(DEBUG, "mpc.cbit") << "The Tenth Velocity is: " << vel_state_vars[9]->value();


  CLOG(DEBUG, "mpc.cbit") << "Linear Component to Return is: " << (vel_state_vars[0]->value())[0];
  CLOG(DEBUG, "mpc.cbit") << "Angular Component to Return is: " << (vel_state_vars[0]->value())[1];

  //const auto test = pose_state_vars[0]->value();
  //Debug: try to display some of the optimizated states

  // return the computed velocity command for the first time step
  Command command;

  // Saturate the command if required to some configuration limits
  // DEBUG: TEMPORARY HARD CODED VELOCITY LIMITS
  double v_lim = 1.5;
  double w_lim = 0.5;
  if (((vel_state_vars[0]->value())[0]) >= v_lim)
  {
    command.linear.x = v_lim;
  }
  else
  {
    command.linear.x = ((vel_state_vars[0]->value())[0]);
  }

  if (((vel_state_vars[0]->value())[1]) >= w_lim)
  {
    command.angular.z = w_lim;
  }
  else
  {
    command.angular.z = ((vel_state_vars[0]->value())[1]);
  }

  return command;

  // END OF EXPERIMENTAL MPC CHANGES

  

  //return Command(); // This returns the stop command when called with no arguments
  
}



}  // namespace lidar
}  // namespace vtr