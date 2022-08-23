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
 * \file cbit.cpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_path_planning/cbit/cbit.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

namespace vtr {
namespace path_planning {

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
auto CBIT::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
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
CBIT::CBIT(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const Callback::Ptr& callback)
    : BasePathPlanner(config, robot_state, callback), config_(config) {
  CLOG(ERROR, "path_planning.cbit") << "Constructing the CBIT Class";
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
}

CBIT::~CBIT() { stop(); }


// Here is where we can do all the teach path pre-processing and then begin the anytime planner asychronously
void CBIT::initializeRoute(RobotState& robot_state) {

  CLOG(ERROR, "path_planning.cbit") << "DEBUG! TRYING TO INITIALIZEROUTE2";
  //initializeRoute2(robot_state);
  /// \todo reset any internal state
  CLOG(ERROR, "path_planning.cbit") << "DEBUG! ENTERED THE WRONG INITIALIZEROUTE";
  CLOG(INFO, "path_planning.cbit") << "Path Planner has been started, here is where we will begin teach path pre-processing and asychronous cbit";
  //CLOG(INFO, "path_planning.teb") << "The state_update_freq config param from ROS is: " << config_->state_update_freq; // This is how you would reference a param, need to use config_ not config!!!
  //CLOG(INFO, "path_planning.teb") << "The state_update_freq config param from ROS is: " << cbit_config.state_update_freq; 
  //CLOG(INFO, "path_planning.teb") << "The alpha config param from ROS is: " << cbit_config.alpha; 

  auto& chain = *robot_state.chain;

  // Initially when the initializeRoute is called in the base planner, the chain has not yet been localized, so we cannot see the frames yet.
  // We need to loop for awhile until the chain localizes doing nothing on this thread (just takes a second or two)
  while (!chain.isLocalized()) 
  {
    // Longer term maybe want a timeout counter here or something? The rest of the stack may well handle it though
  }
  

  //CLOG(INFO, "path_planning.cbit") << "Robot is now localized and we can start trying to pre-process the map";


  //CLOG(INFO, "path_planning.teb") << "Testing whether I can access the shared path memory pointer or not: " << cbit_path_ptr;


  //CLOG(INFO, "path_planning.teb") << "The size of the chain is: " << chain.size();
  //CLOG(INFO, "path_planning.teb") << "The first frame is " << chain.pose(0);
  //CLOG(INFO, "path_planning.teb") << "The second frame is " << chain.pose(1);

  // World frame: (I think the very first frame is identity)
  /*
  auto world_frame = chain.pose(0);
  auto next_frame = chain.pose(1);
  std::tuple<double, double, double, double, double, double> vector = T2xyrpy(next_frame);
  CLOG(INFO, "path_planning.teb") << "My se(3) vector conversion: x: " << std::get<0>(vector) << " y: " 
  << std::get<1>(vector) << " z: " << std::get<2>(vector) << " roll: " << std::get<3>(vector) << " pitch: " 
  << std::get<4>(vector) << " yaw: " << std::get<5>(vector);
  */

  //CLOG(INFO, "path_planning.teb") << "lgmath se(3) vector conversion: " << tran2vec(next_frame);
  
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

  CLOG(INFO, "path_planning.cbit") << "Path has been pre-processed, Attempting to instantiae the Planner";


  // instantiate the planner
  CBITPlanner cbit(cbit_config, global_path_ptr, robot_state, cbit_path_ptr, costmap_ptr);

  CLOG(INFO, "path_planning.cbit") << "Planner Successfully Created and resolved, end of initializeRoute function";

  // Here is an example for getting the teach path frames, use chain.pose(<frame_index>) where 0 is the first frame
  // I think there is a chain.size() function you can use to get the total number of frames in the teach path we are trying to repeat.
  //CLOG(INFO, "path_planning.teb") << "Key Frame 1 " << chain.pose(0);
  //CLOG(INFO, "path_planning.teb") << "Key Frame 2 " << chain.pose(1);

  /*
  const auto chain_info = getChainInfo(robot_state);
  auto [stamp, w_p_r_in_r, T_p_r, T_w_p, curr_sid] = chain_info;


  CLOG(INFO, "path_planning.teb") << "stamp " << stamp;
  CLOG(INFO, "path_planning.teb") << "w_p_r_in_r: " << w_p_r_in_r;
  CLOG(INFO, "path_planning.teb") << "T_p_r: " << T_p_r;
  CLOG(INFO, "path_planning.teb") << "T_w_p: " << T_w_p;

  CLOG(INFO, "path_planning.teb") << "Successfull Displayed Chain info: ";
  */



  // PseudoCode

  // Get the transformation chain, store to a vector of transforms

  // Iterate through the transform chain, convert to Euclidean points w.r.t the world frmae (Frame 1 of current teach path)
  // Also compute an associated p

  // Preprocess the path (spline, ROC) by piping this vector into the generate_pq path class

  // Create the main CBIT class object with the config initializations and path, which will kick off the asychronous planner
}


// Called at a the control rate in base_planner, eventually we need to do the mpc here using the output of cbit as constraints
// For now just using it for debug and sending an empty command

// TODO: longer term the mpc will be implemented in the mpc_path_planner.cpp file, which will then become the base class of this file
         // Actually now that I think of it, maybe it makes more sense for mpc to derive from cbit so it has access to the path variables
// For now though im going to implement the MPC here directly though
auto CBIT::computeCommand(RobotState& robot_state) -> Command {
  auto& chain = *robot_state.chain;
  if (!chain.isLocalized()) {
    CLOG(WARNING, "path_planning.cbit")
        << "Robot is not localized, command to stop the robot";
    return Command();
  }

  /*
  else {
    CLOG(INFO, "path_planning.cbit") << "Robot is now localized and we are trying to compute a command";
  }
  */


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
  std::string test_string = "This is a test message!";
  // Dont visualize unless we are both localized and an initial solution is found
  if ((*cbit_path_ptr).size() != 0)
  {
    visualize(test_string, stamp, T_w_p, T_p_r);
    // Testing that we are receiving the most up to date output plans
    //CLOG(INFO, "path_planning.cbit") << "The first pose is x: " << (*cbit_path_ptr)[0].x << " y: " << (*cbit_path_ptr)[0].y << " z: " << (*cbit_path_ptr)[0].z;
  }

  //TODO, MPC WITHOUT OBSTACLES
  /*

  // Conduct an MPC cycle

  // PSEUDOCODE

  // HARDCODED INITIALIZATIONS (TODO: Move these to configs)
  int K = 5;
  double DT = 0.2;
  double VF = 1.0;

  // Initialize the Current Robot State in the world frame
  Eigen::Matrix4d T0;
  T0 << 1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1;
  lgmath::se3::Transformation T0_ = lgmath::se3::Transformation(T0);

  CLOG(DEBUG, "mpc.cbit") << "MPC TESTING:";
  CLOG(DEBUG, "mpc.cbit") << "The Current Robot State is: " << T0_;

  // Generate STEAM States for the velocity vector and SE3 state transforms
  std::vector<lgmath::se3::Transformation> pose_states;
  std::vector<std::vector<double>> vel_states;
  
  //const Eigen::Matrix4d& T0 = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};
  //lgmath::se3::Transformation test_state = lgmath::se3::Transformation();
  for (int i=0; i<K; i++)
  {
    pose_states.push_back(lgmath::se3::Transformation());
    vel_states.push_back(std::vector<double> {0.0, 0.0});
  }

  // Initialize the first state to the current initial position
  //Eigen::Matrix4d T0;
  //T0 << 1,0,0,0,
  //      0,1,0,0,
  //      0,0,1,0,
  //      0,0,0,1;
  //pose_states[0].C_ba_ = Eigen::Matrix3d({{1,0,0}, {0,1,0}, {0,0,1}});

  // Take in the current euclidean path solution from the cbit planner in the world frame, the current robot state, and determine
  // which measurements we wish to track to follow the path at the desired target velocity

  // Setup the optimization problem

  // Generate the cost terms using combinations of the builtin steam evaluators

  // Solve the optimization problem with GuassNewton solver
  
  // return the computed velocity command for the first time step
  */

  





  return Command(); // This returns the stop command when called with no arguments
}


// Function for grabbing the robots velocity in planning frame, transform of robot into planning frame, and transform of planning frame to world frame
// Modified it to only give this info, I think the other things were only really teb relevant for now
auto CBIT::getChainInfo(RobotState& robot_state) -> ChainInfo {
  auto& chain = *robot_state.chain;
  auto lock = chain.guard();
  const auto stamp = chain.leaf_stamp();
  const auto w_p_r_in_r = chain.leaf_velocity();
  const auto T_p_r = chain.T_leaf_trunk().inverse();
  const auto T_w_p = chain.T_start_trunk();
  const auto curr_sid = chain.trunkSequenceId();
  return ChainInfo{stamp, w_p_r_in_r, T_p_r, T_w_p, curr_sid};
}


void CBIT::visualize(std::string text, const tactic::Timestamp& stamp, const tactic::EdgeTransform& T_w_p, const tactic::EdgeTransform& T_p_r)
{
  //CLOG(ERROR, "path_planning.cbit") << "TRYING TO VISUALIZE IN CBIT CLASS";
  // Test string message to make sure publisher is working
  std_msgs::msg::String string_msg;
  string_msg.data = text;
  test_pub_->publish(string_msg);

  /// Publish the current frame for planning
  {
    Eigen::Affine3d T(T_w_p.matrix());
    auto msg = tf2::eigenToTransform(T);
    msg.header.stamp = rclcpp::Time(stamp);
    msg.header.frame_id = "world";
    msg.child_frame_id = "planning frame";
    tf_bc_->sendTransform(msg);
  }

  /// Publish the current robot in the planning frame
  {
    Eigen::Affine3d T(T_p_r.matrix());
    auto msg = tf2::eigenToTransform(T);
    msg.header.frame_id = "planning frame";
    msg.header.stamp = rclcpp::Time(stamp);
    msg.child_frame_id = "robot planning";
    tf_bc_->sendTransform(msg);
  }

  // Attempting to publish the actual path which we are receiving from the shared pointer in the cbitplanner
  // The path is stored as a vector of se3 Pose objects from cbit/utils, need to iterate through and construct proper ros2 nav_msgs PoseStamped

  /// Publish the intermediate goals in the planning frame
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "world";
    path.header.stamp = rclcpp::Time(stamp);
    auto& poses = path.poses;

    // iterate through the path
    //CLOG(INFO, "path_planning.cbit") << "Trying to publish the path, the size is: " << (*cbit_path_ptr).size();
    geometry_msgs::msg::Pose test_pose;
    for (unsigned i = 0; i < (*cbit_path_ptr).size(); ++i) 
    {
      auto& pose = poses.emplace_back();
      // The teb planner used this toMsg function to convert transforms into the correct pose message type, but I dont see why we cant just directly use the geometry message
      //pose.pose = tf2::toMsg(Eigen::Affine3d(T_p_i_vec[i].matrix())); // oh in hindsight I think this is how to grab the transform from a transform with covariance
      test_pose.position.x = (*cbit_path_ptr)[i].x;
      test_pose.position.y = (*cbit_path_ptr)[i].y; 
      test_pose.position.z = (*cbit_path_ptr)[i].z; 
      test_pose.orientation.x = 0.0;
      test_pose.orientation.y = 0.0;
      test_pose.orientation.z = 0.0;
      test_pose.orientation.w = 1.0;
      pose.pose = test_pose;
    }

    path_pub_->publish(path);
  }
  return;
}


}  // namespace path_planning
}  // namespace vtr