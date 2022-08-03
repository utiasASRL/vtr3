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
    CLOG(INFO, "path_planning.teb") << "The transform from costmap to robot is:" << T_start_vertex;
    
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
    costmap_ptr->obs_map = obs_map;
    // Store the transform T_c_w (from costmap to world)
    costmap_ptr->T_c_w = T_start_vertex.inverse(); // Note this probably isnt the right transform to assign, need to do some poking to figure out what is right
    // Store the grid resoltuion
    costmap_ptr->grid_resolution = change_detection_costmap->dl();
  


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
  return Command(); // This returns the stop command when called with no arguments
  
}



}  // namespace lidar
}  // namespace vtr