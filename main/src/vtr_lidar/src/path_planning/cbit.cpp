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
#include "vtr_lidar/path_planning/cbit.hpp"
#include "vtr_lidar/path_planning/mpc.hpp"
#include "vtr_lidar/cache.hpp"


namespace vtr {
namespace lidar {
namespace {
// Function for converting Transformation matrices into se(2) [x, y, z, roll, pitch, yaw]
inline std::tuple<double, double, double, double, double, double> T2xyzrpy(
    const tactic::EdgeTransform& T) {
  const auto Tm = T.matrix();
  return std::make_tuple(Tm(0, 3), Tm(1, 3), Tm(2,3), std::atan2(Tm(2, 1), Tm(2, 2)), std::atan2(-1*Tm(2, 0), sqrt(pow(Tm(2, 1),2) + pow(Tm(2, 2),2))), std::atan2(Tm(1, 0), Tm(0, 0)));
}
}


// Configure the class as a ROS2 node, get configurations from the ros parameter server
auto LidarCBIT::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();

  // Base planner configs
  config->control_period = (unsigned int)node->declare_parameter<int>(prefix + ".control_period", config->control_period);

  // robot configuration
  config->robot_model = node->declare_parameter<std::string>(prefix + ".teb.robot_model", config->robot_model);
  config->robot_radius = node->declare_parameter<double>(prefix + ".teb.robot_radius", config->robot_radius);

  // CBIT Configs
  // ENVIRONMENT:
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


  // MPC Configs:
  // CONTROLLER PARAMS
  config->horizon_steps = node->declare_parameter<int>(prefix + ".mpc.horizon_steps", config->horizon_steps);
  config->horizon_step_size = node->declare_parameter<double>(prefix + ".mpc.horizon_step_size", config->horizon_step_size);
  config->forward_vel = node->declare_parameter<double>(prefix + ".mpc.forward_vel", config->forward_vel);
  config->max_lin_vel = node->declare_parameter<double>(prefix + ".mpc.max_lin_vel", config->max_lin_vel);
  config->max_ang_vel = node->declare_parameter<double>(prefix + ".mpc.max_ang_vel", config->max_ang_vel);

  // COST FUNCTION WEIGHTS
  const auto pose_error_diag = node->declare_parameter<std::vector<double>>(prefix + ".mpc.pose_error_cov", std::vector<double>());
  config->pose_error_cov.diagonal() << pose_error_diag[0], pose_error_diag[1], pose_error_diag[2], pose_error_diag[3], pose_error_diag[4], pose_error_diag[5];

  const auto vel_error_diag = node->declare_parameter<std::vector<double>>(prefix + ".mpc.vel_error_cov", std::vector<double>());
  config->vel_error_cov.diagonal() << vel_error_diag[0], vel_error_diag[1];

  const auto acc_error_diag = node->declare_parameter<std::vector<double>>(prefix + ".mpc.acc_error_cov", std::vector<double>());
  config->acc_error_cov.diagonal() << acc_error_diag[0], acc_error_diag[1];

  const auto kin_error_diag = node->declare_parameter<std::vector<double>>(prefix + ".mpc.kin_error_cov", std::vector<double>());
  config->kin_error_cov.diagonal() << kin_error_diag[0], kin_error_diag[1], kin_error_diag[2], kin_error_diag[3], kin_error_diag[4], kin_error_diag[5];

  // MISC
  config->command_history_length = node->declare_parameter<int>(prefix + ".mpc.command_history_length", config->command_history_length);

  // COSTMAP PARAMS
  config->costmap_filter_value = node->declare_parameter<double>(prefix + ".costmap.costmap_filter_value", config->costmap_filter_value);
  config->costmap_history = node->declare_parameter<int>(prefix + ".costmap.costmap_history", config->costmap_history);

  return config;
}

// Declare class as inherited from the BasePathPlanner
LidarCBIT::LidarCBIT(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const Callback::Ptr& callback)
    : vtr::path_planning::CBIT(config, robot_state, callback), config_(config) {
  CLOG(INFO, "path_planning.cbit") << "Constructing the LidarCBIT Class";
  
  // Initialize the current velocity state and a vector for storing a history of velocity commands applied
  applied_vel << 0,
                 0;
  vel_history.reserve(config_->command_history_length); 
  for (int i = 0; i < config_->command_history_length; i++) 
  {
    vel_history.push_back(applied_vel);
  }
}

LidarCBIT::~LidarCBIT() { stop(); }

// Given the current plan and obstacles, generate a twist command for the robot using tracking mpc
auto LidarCBIT::computeCommand(RobotState& robot_state0) -> Command {
  auto& robot_state = dynamic_cast<LidarOutputCache&>(robot_state0);
  auto& chain = *robot_state.chain;
  if (!chain.isLocalized()) {
    CLOG(WARNING, "path_planning.cbit") << "Robot is not localized, commanding the robot to stop";
    applied_vel << 0.0, 0.0;
    // Update history:
    vel_history.erase(vel_history.begin());
    vel_history.push_back(applied_vel);
    return Command();
  }

  // retrieve the transorm info from the localization chain
  const auto chain_info = getChainInfo(robot_state);
  auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] = chain_info;


  //START OF OBSTACLE PERCEPTION UPDATES

  // Generate an occupancy grid from the current LIDAR observation
  CLOG(INFO, "obstacle_detection.cbit") << "Trying to get the costmap";
  auto change_detection_costmap_ref = robot_state.change_detection_costmap.locked();
  auto& change_detection_costmap = change_detection_costmap_ref.get();

  if (change_detection_costmap.valid() && (prev_stamp != stamp)) 
  {
    CLOG(INFO, "obstacle_detection.cbit") << "Costmap received";
    // update change detection result
    const auto costmap_sid = change_detection_costmap->vertex_sid();
    const auto costmap_T_vertex_this =change_detection_costmap->T_vertex_this();
    const auto T_start_vertex = chain.pose(costmap_sid);
 
    // This is the important line, this is what is grabbing the data from the grid, and returning an unordered dicrete grid map which we can use for collision checking
    vtr::lidar::BaseCostMap::XY2ValueMap obs_map = change_detection_costmap->filter(config_->costmap_filter_value);
    CLOG(DEBUG, "obstacle_detection.cbit") << "the size of the map is: " << obs_map.size();

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
    CLOG(DEBUG, "obstacle_detection.cbit") << "Updating Costmap SID to: " << change_detection_costmap->vertex_sid();
    costmap_ptr->obs_map = obs_map;
    // Store the transform T_c_w (from costmap to world)
    costmap_ptr->T_c_w = T_start_vertex.inverse(); // note that T_start_vertex is T_w_c if we want to bring keypoints to the world frame
    // Store the grid resoltuion
    CLOG(DEBUG, "obstacle_detection.cbit") << "The costmap to world transform is: " << T_start_vertex.inverse();
    costmap_ptr->grid_resolution = change_detection_costmap->dl();

    // Experimental: Storing sequences of costmaps for temporal filtering purposes
    // For the first x iterations, fill the obstacle vector
    if (costmap_ptr->obs_map_vect.size() < config_->costmap_history)
    {
      costmap_ptr->obs_map_vect.push_back(obs_map);
      costmap_ptr->T_c_w_vect.push_back(costmap_ptr->T_c_w);
    }
    // After that point, we then do a sliding window using shift operations, moving out the oldest map and appending the newest one
    else
    {
      // Legacy code for when I was filtering inside cbit (made a quick and dirty work around temporarily)
      //for (int i = 0; i < (config_->costmap_history-1); i++)
      //{
      //  costmap_ptr->obs_map_vect[i] = costmap_ptr->obs_map_vect[i + 1];
      //  costmap_ptr->T_c_w_vect[i] = costmap_ptr->T_c_w_vect[i + 1];
      //}
      costmap_ptr->obs_map_vect[config_->costmap_history-1] = obs_map;
      costmap_ptr->T_c_w_vect[config_->costmap_history-1] = costmap_ptr->T_c_w ;
    }

    // //TODO: Experimental global costmap filtering idea
    // Pseudocode:
    // 1. Iterate through all the keys in the unordered map, generate a point from the centre of the cell, transform that point to the world frame.
    // 2. Repeat this for the 3 most recent costmaps
    // 3. In the global map, we purge it of all obstacle cells that are not within a particular radius of the current robot state (do this in a loop at the start)
    // 4. each transformed point can be assigned to a cell bin of size "grid_resolution" in the world frame. If all 3 of the recent scans (or whatever level of transient filter we want)
    //    agree on an obs cell, we create an unordered map key value pair in the global map pointer
    // 5. This will generate a global obstacle map with transients filtered, but will also remember obstacles it had seen previously but have since come to close
    // 6. Then we just pass the global map to cbit, which only needs to collision check one cost map and all the filtering is contained here

    // I think this would work well, only issue is if you have a tall dynamic object walking very close to the robot (in the permanence range)
    // In that case it will add those cells to the map and basically never remove them, but not much we can do about that without better sensor fov/predictions

    
  }
  
  // END OF OBSTACLE PERCEPTION UPDATES


  // Extrapolate the pose of the robot into the future based on the localization delay
  prev_stamp = stamp;
  const auto curr_time = now();  // always in nanoseconds
  const auto dt = static_cast<double>(curr_time - stamp) * 1e-9;

  // This code is for the old robot pose extrapolation using odometry. I found this to be very unstable and not very useful so it is no longer in use
  const auto T_p_r_extp = [&]() {
    // extrapolate based on current time
    Eigen::Matrix<double, 6, 1> xi_p_r_in_r(dt * w_p_r_in_r);
    CLOG(INFO, "mpc.cbit")
      << "Time difference b/t estimation and planning: " << dt;
    return T_p_r * tactic::EdgeTransform(xi_p_r_in_r).inverse();
  }();


  // START OF MPC CODE
  // Dont proceed to mpc control unless we have a valid plan to follow from BIT*, else return a 0 velocity command to stop and wait
  if ((*cbit_path_ptr).size() != 0)
  {

    // Initializations from config
    int K = config_->horizon_steps; // Horizon steps
    double DT = config_->horizon_step_size; // Horizon step size
    double VF = config_->forward_vel; // Desired Forward velocity set-point for the robot. MPC will try to maintain this rate while balancing other constraints

    // Pose Covariance Weights
    Eigen::Matrix<double, 6, 6> pose_noise_vect;
    pose_noise_vect = config_->pose_error_cov;

    // Disturbance Velocity Covariance
    Eigen::Matrix<double, 2, 2> vel_noise_vect;
    vel_noise_vect = config_->vel_error_cov;

    // Acceleration Tuning
    Eigen::Matrix<double, 2, 2> accel_noise_vect;
    accel_noise_vect = config_->acc_error_cov;

    // Kinematics Covariance Weights (should be weighted quite heavily (smaller is higher because its covariance))
    Eigen::Matrix<double, 6, 6> kin_noise_vect;
    kin_noise_vect = config_->kin_error_cov;


    
    // Extrapolating robot pose into the future by using the history of applied mpc velocity commands
    const auto curr_time = now();  // always in nanoseconds
    const auto dt = static_cast<double>(curr_time - stamp) * 1e-9;


    CLOG(INFO, "mpc_debug.cbit") << "History of the Robot Velocities:" << vel_history;
    
    // Check the time past since the last state update was received
    // Go back through the vel_history to approximately dt seconds in the past
    // Start applying each of the applied velocities sequentially 
    double control_period = config_->control_period / 1000.0; // control period is given by user in ms in the config
    auto T_p_r2 = T_p_r;
    for (int i=std::floor(dt / control_period); i > 0; i--)
    {
      CLOG(DEBUG, "mpc_debug.cbit") << "The iteration Index i is: " << i;
      w_p_r_in_r(0) = -1* vel_history[vel_history.size()-(i+1)][0];
      w_p_r_in_r(1) = 0.0;
      w_p_r_in_r(2) = 0.0;
      w_p_r_in_r(3) = 0.0;
      w_p_r_in_r(4) = 0.0;
      w_p_r_in_r(5) = -1* vel_history[vel_history.size()-(i+1)][1];
      CLOG(DEBUG, "mpc_debug.cbit") << "Robot velocity Used for Extrapolation: " << -w_p_r_in_r.transpose() << std::endl;

      Eigen::Matrix<double, 6, 1> xi_p_r_in_r(control_period * w_p_r_in_r);
      T_p_r2 = T_p_r2 * tactic::EdgeTransform(xi_p_r_in_r).inverse();
      CLOG(DEBUG, "mpc_debug.cbit") << "Make sure the lie algebra is changing right:" << T_p_r2;

    }
    // Apply the final partial period velocity
    w_p_r_in_r(0) = -1* vel_history.back()[0];
    w_p_r_in_r(1) = 0.0;
    w_p_r_in_r(2) = 0.0;
    w_p_r_in_r(3) = 0.0;
    w_p_r_in_r(4) = 0.0;
    w_p_r_in_r(5) = -1* vel_history.back()[1];
    CLOG(DEBUG, "mpc_debug.cbit") << "Robot velocity Used for Extrapolation: " << -w_p_r_in_r.transpose() << std::endl;
    Eigen::Matrix<double, 6, 1> xi_p_r_in_r((dt - (std::floor(dt / control_period) * control_period)) * w_p_r_in_r);
    T_p_r2 = T_p_r2 * tactic::EdgeTransform(xi_p_r_in_r).inverse();
    CLOG(DEBUG, "mpc_debug.cbit") << "The final time period is: "  << (dt - (std::floor(dt / control_period) * control_period));
    const auto T_p_r_extp2 = T_p_r2;

    CLOG(DEBUG, "mpc_debug.cbit") << "New extrapolated pose:"  << T_p_r_extp2;
    
    // Uncomment if we use the extrapolated robot pose for control (constant velocity model from odometry)
    //lgmath::se3::Transformation T0 = lgmath::se3::Transformation(T_w_p * T_p_r_extp);

    // Uncomment for using the mpc extrapolated robot pose for control
    lgmath::se3::Transformation T0 = lgmath::se3::Transformation(T_w_p * T_p_r_extp2);

    // no extrapolation (comment this out if we are not using extrapolation)
    //lgmath::se3::Transformation T0 = lgmath::se3::Transformation(T_w_p * T_p_r);

    // TODO: Set whether to use mpc extrapolation as a config param (though there is almost never a good reason not to use it)

    //Convert to x,y,z,roll, pitch, yaw
    std::tuple<double, double, double, double, double, double> robot_pose = T2xyzrpy(T0);
    CLOG(DEBUG, "mpc_debug.cbit") << "The Current Robot Pose (from planning) is - x: " << std::get<0>(robot_pose) << " y: " << std::get<1>(robot_pose) << " yaw: " << std::get<5>(robot_pose);

    CLOG(DEBUG, "mpc_debug.cbit") << "The Current Robot State Transform is: : " << T0;
    // Need to also invert the robot state to make it T_vi instead of T_iv as this is how the MPC problem is structured
    lgmath::se3::Transformation T0_inv = T0.inverse();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Inverted Current Robot State Using Direct Robot Values is: " << T0_inv;
    // End of pose extrapolation


    // Calculate which T_ref measurements to used based on the current path solution
    CLOG(INFO, "mpc.cbit") << "Attempting to generate T_ref measurements";
    auto meas_result = GenerateReferenceMeas(cbit_path_ptr, robot_pose, K,  DT, VF);
    auto measurements = meas_result.measurements;
    bool point_stabilization = meas_result.point_stabilization;


    // Create and solve the STEAM optimization problem
    std::vector<lgmath::se3::Transformation> mpc_poses;
    try
    {
      CLOG(INFO, "mpc.cbit") << "Attempting to solve the MPC problem";
      auto mpc_result = SolveMPC(applied_vel, T0, measurements, K, DT, VF, pose_noise_vect, vel_noise_vect, accel_noise_vect, kin_noise_vect, point_stabilization);
      applied_vel = mpc_result.applied_vel; // note dont re-declare applied vel here
      mpc_poses = mpc_result.mpc_poses;
      CLOG(INFO, "mpc.cbit") << "Successfully solved MPC problem";
    }
    catch(...)
    {
      CLOG(ERROR, "mpc.cbit") << "STEAM Optimization Failed; Commanding to Stop the Vehicle";
      applied_vel(0) = 0.0;
      applied_vel(1) = 0.0;
    }

    CLOG(INFO, "mpc.cbit") << "The linear velocity is:  " << applied_vel(0) << " The angular vel is: " << applied_vel(1);

 
    // If required, saturate the output velocity commands based on the configuration limits
    CLOG(INFO, "mpc.cbit") << "Saturating the velocity command if required";
    Eigen::Matrix<double, 2, 1> saturated_vel = SaturateVel(applied_vel, config_->max_lin_vel, config_->max_ang_vel);

    // Store the result in memory so we can use previous state values to re-initialize and extrapolate the robot pose in subsequent iterations
    vel_history.erase(vel_history.begin());
    vel_history.push_back(saturated_vel);
    
    // Store the current robot state in the robot state path so it can be visualized
    robot_poses.push_back(T_w_p * T_p_r);

    // Send the robot poses and mpc prediction to rviz
    visualize(stamp, T_w_p, T_p_r, T_p_r_extp, T_p_r_extp2, mpc_poses, robot_poses);

    // return the computed velocity command for the first time step
    Command command;
    command.linear.x = saturated_vel(0);
    command.angular.z = saturated_vel(1);

    CLOG(INFO, "mpc.cbit")
      << "Final control command: [" << command.linear.x << ", "
      << command.linear.y << ", " << command.linear.z << ", "
      << command.angular.x << ", " << command.angular.y << ", "
      << command.angular.z << "]";

    return command;
  }
  // Otherwise stop the robot
  else
  {
    CLOG(INFO, "mpc.cbit") << "There is not a valid plan yet, returning zero velocity commands";

    applied_vel << 0.0, 0.0;
    vel_history.erase(vel_history.begin());
    vel_history.push_back(applied_vel);
    return Command();
  }
}



}  // namespace lidar
}  // namespace vtr