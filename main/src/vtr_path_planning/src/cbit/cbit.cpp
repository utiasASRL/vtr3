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
#include <tf2_eigen/tf2_eigen.hpp>

namespace vtr {
namespace path_planning {

namespace {
// Function for converting Transformation matrices into se(2) [x, y, z, roll, pitch, yaw]
inline std::tuple<double, double, double, double, double, double> T2xyzrpy(const tactic::EdgeTransform& T) 
{
  const auto Tm = T.matrix();
  return std::make_tuple(Tm(0, 3), Tm(1, 3), Tm(2,3), std::atan2(Tm(2, 1), Tm(2, 2)), std::atan2(-1*Tm(2, 0), sqrt(pow(Tm(2, 1),2) + pow(Tm(2, 2),2))), std::atan2(Tm(1, 0), Tm(0, 0)));
}
}

// Configure the class as a ROS2 node, get configurations from the ros parameter server
auto CBIT::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();

  // Base planner configs
  config->control_period = (unsigned int)node->declare_parameter<int>(prefix + ".control_period", config->control_period);

  // CBIT Configs
  // ENVIRONMENT:
  config->obstacle_avoidance = node->declare_parameter<bool>(prefix + ".cbit.obstacle_avoidance", config->obstacle_avoidance);
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
  config->eta = node->declare_parameter<double>(prefix + ".cbit.eta", config->eta);
  config->rad_m_exhange = node->declare_parameter<double>(prefix + ".rad_m_exhange", config->rad_m_exhange);
  config->initial_exp_rad = node->declare_parameter<double>(prefix + ".cbit.initial_exp_rad", config->initial_exp_rad);
  config->extrapolation = node->declare_parameter<bool>(prefix + ".cbit.extrapolation", config->extrapolation);


  // MPC Configs:
  // SPEED SCHEDULER PARAMETERS
  config->planar_curv_weight = node->declare_parameter<double>(prefix + ".speed_scheduler.planar_curv_weight", config->planar_curv_weight);
  config->profile_curv_weight = node->declare_parameter<double>(prefix + ".speed_scheduler.profile_curv_weight", config->profile_curv_weight);
  config->eop_weight = node->declare_parameter<double>(prefix + ".speed_scheduler.eop_weight", config->eop_weight);
  config->min_vel = node->declare_parameter<double>(prefix + ".speed_scheduler.min_vel", config->min_vel);

  // CONTROLLER PARAMS
  config->kinematic_model = node->declare_parameter<std::string>(prefix + ".mpc.vehicle_model", config->kinematic_model);
  config->extrapolate_robot_pose = node->declare_parameter<bool>(prefix + ".mpc.extrapolate_robot_pose", config->extrapolate_robot_pose);
  config->mpc_verbosity = node->declare_parameter<bool>(prefix + ".mpc.mpc_verbosity", config->mpc_verbosity);
  config->forward_vel = node->declare_parameter<double>(prefix + ".mpc.forward_vel", config->forward_vel);
  config->max_lin_vel = node->declare_parameter<double>(prefix + ".mpc.max_lin_vel", config->max_lin_vel);
  config->max_ang_vel = node->declare_parameter<double>(prefix + ".mpc.max_ang_vel", config->max_ang_vel);
  config->max_lin_acc = node->declare_parameter<double>(prefix + ".mpc.max_lin_acc", config->max_lin_acc);
  config->max_ang_acc = node->declare_parameter<double>(prefix + ".mpc.max_ang_acc", config->max_ang_acc);
  config->robot_linear_velocity_scale = node->declare_parameter<double>(prefix + ".mpc.robot_linear_velocity_scale", config->robot_linear_velocity_scale);
  config->robot_angular_velocity_scale = node->declare_parameter<double>(prefix + ".mpc.robot_angular_velocity_scale", config->robot_angular_velocity_scale);
  config->turning_radius = node->declare_parameter<double>(prefix + ".mpc.turning_radius", config->turning_radius);

  // MISC
  config->command_history_length = node->declare_parameter<int>(prefix + ".mpc.command_history_length", config->command_history_length);

  // COSTMAP PARAMS
  config->costmap_filter_value = node->declare_parameter<double>(prefix + ".costmap.costmap_filter_value", config->costmap_filter_value);
  config->costmap_history = node->declare_parameter<int>(prefix + ".costmap.costmap_history", config->costmap_history);

  return config;
}


// Declare class as inherited from the BasePathPlanner
CBIT::CBIT(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const Callback::Ptr& callback)
    : BasePathPlanner(config, robot_state, callback), config_(config) {
  CLOG(INFO, "cbit.path_planning") << "Constructing the CBIT Class";
  robot_state_ = robot_state;
  const auto node = robot_state->node.ptr();

  if(config->kinematic_model == "unicycle")
    solver_ = std::make_shared<CasadiUnicycleMPC>(config->mpc_verbosity);
  else if (config->kinematic_model == "ackermann")
    solver_ = std::make_shared<CasadiAckermannMPC>(config->mpc_verbosity);
  else{
    CLOG(ERROR, "cbit") << "Config parameter vehicle_model must be one of 'unicycle' or 'ackermann'";
    throw std::invalid_argument("Config parameter vehicle_model must be one of 'unicycle' or 'ackermann'");
  }

  // Create visualizer and its corresponding pointer:
  VisualizationUtils visualization_utils(node);
  visualization_ptr = std::make_shared<VisualizationUtils>(visualization_utils);

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
  cbit_config.eta = config->eta;
  cbit_config.rad_m_exhange = config->rad_m_exhange;
  cbit_config.initial_exp_rad = config->initial_exp_rad;
  cbit_config.extrapolation = config->extrapolation;

  // Misc
  cbit_config.incremental_plotting = config->incremental_plotting;
  cbit_config.plotting = config->plotting;
  CLOG(INFO, "cbit.path_planning") << "Successfully Constructed the CBIT Class";
}

void CBIT::setRunning(const bool running) {
  //Starting up
  if (running_ == false && running == true) {
    CLOG(INFO, "cbit.path_planning") << "Initializing CBIT Route";
    initializeRoute(*robot_state_);
    CLOG(INFO, "cbit.path_planning") << "CBIT Plan Completed";
  } else if (running_ == true && running == false) {
    CLOG(INFO, "cbit.path_planning") << "Stopping CBIT Planning";
    planner_ptr_->stopPlanning();
    if (process_thread_cbit_.joinable()) process_thread_cbit_.join();
    thread_count_--;
    planner_ptr_->resetPlanner();
    // planner_ptr_.reset();
    CLOG(INFO, "cbit.path_planning") << "Stopped CBIT Planning";
  }
  BasePathPlanner::setRunning(running);
}

CBIT::~CBIT() { stop_cbit(); }

void CBIT::stop_cbit() {
  UniqueLock lock(mutex_);
  terminate_ = true;
  cv_terminate_or_state_changed_.notify_all();
  cv_thread_finish_.wait(lock, [this] { return thread_count_ == 0; });
  if (process_thread_cbit_.joinable()) process_thread_cbit_.join();
}

void CBIT::process_cbit() {
  el::Helpers::setThreadName("cbit_path_planning");
  CLOG(INFO, "cbit.path_planning") << "Starting the CBIT Planning Thread";
  while (true) {
    UniqueLock lock(mutex_);
    cv_terminate_or_state_changed_.wait(lock, [this] {
      waiting_ = true;
      cv_waiting_.notify_all();
      return terminate_ || running_;
    });
    waiting_ = false;

    if (terminate_) {
        waiting_ = true;
        cv_waiting_.notify_all();
        --thread_count_;
        CLOG(INFO, "cbit.path_planning") << "Stopping the CBIT Thread.";
        cv_thread_finish_.notify_all();
        return;
    }

    // Planner should not require the thread lock to execute
    lock.unlock();

    // Note we need to run the above first so that the lidarcbit class can be constructed before calling initializeroute (so it can be overrided correctly)

  }
}




// Here is where we do all the teach path pre-processing and then begin the anytime planner asychronously
void CBIT::initializeRoute(RobotState& robot_state) {
  auto& chain = *robot_state.chain;


  // Initialize the shared pointer to the output of the planner
  cbit_path_ptr = std::make_shared<std::vector<Pose>> (cbit_path);
  valid_solution_ptr = std::make_shared<bool> (false);
  q_max_ptr = std::make_shared<double> (config_->q_max);

    // Initialize the current velocity state and a vector for storing a history of velocity commands applied
  applied_vel_ << 0,
                 0;
  vel_history.reserve(config_->command_history_length);
  for (int i = 0; i < config_->command_history_length; i++)
  {
    vel_history.push_back(applied_vel_);
  }
  robot_poses.clear();

  lgmath::se3::TransformationWithCovariance teach_frame;
  std::tuple<double, double, double, double, double, double> se3_vector;
  Pose se3_pose;
  std::vector<Pose> euclid_path_vec; // Store the se3 frames w.r.t the initial world frame into a path vector
  euclid_path_vec.reserve(chain.size());
  // Loop through all frames in the teach path, convert to euclidean coords w.r.t the first frame and store it in a cbit Path class (vector of se(3) poses)
  for (size_t i = 0; i < chain.size(); i++)
  {
    teach_frame = chain.pose(i);
    se3_vector = T2xyzrpy(teach_frame);
    se3_pose = Pose(std::get<0>(se3_vector), std::get<1>(se3_vector), std::get<2>(se3_vector), std::get<3>(se3_vector), std::get<4>(se3_vector), std::get<5>(se3_vector));
    euclid_path_vec.push_back(se3_pose);
  }


  CLOG(INFO, "cbit.path_planning") << "Trying to create global path";
  // Create the path class object (Path preprocessing)
  global_path_ptr = std::make_shared<CBITPath>(cbit_config, euclid_path_vec);
  CLOG(INFO, "cbit.path_planning") << "Teach Path has been pre-processed. Attempting to initialize the dynamic corridor";


  // Initialize the dynamic corridor
  corridor_ptr = std::make_shared<CBITCorridor>(cbit_config, global_path_ptr);
  CLOG(INFO, "cbit.path_planning") << "Corridor generated successfully. Attempting to instantiate the planner";

  // Instantiate the planner
  planner_ptr_ = std::make_shared<CBITPlanner>(cbit_config, global_path_ptr, robot_state, cbit_path_ptr, costmap_ptr, corridor_ptr, valid_solution_ptr, q_max_ptr);
  CLOG(INFO, "cbit.path_planning") << "Planner successfully created and resolved";
  thread_count_++;
  process_thread_cbit_ = std::thread(&CBITPlanner::plan, planner_ptr_);

}

auto CBIT::computeCommand(RobotState& robot_state) -> Command {
  auto raw_command = computeCommand_(robot_state);
  
  Eigen::Vector2d output_vel = {raw_command.linear.x, raw_command.angular.z};

  // Apply robot motor controller calibration scaling factors if applicable
  output_vel(0) = output_vel(0) * config_->robot_linear_velocity_scale;
  output_vel(1) = output_vel(1) * config_->robot_angular_velocity_scale;

  // If required, saturate the output velocity commands based on the configuration limits
  CLOG(DEBUG, "cbit.control") << "Saturating the velocity command if required";
  Eigen::Vector2d saturated_vel = saturateVel(output_vel, config_->max_lin_vel, config_->max_ang_vel);
  CLOG(INFO, "cbit.control") << "The Saturated linear velocity is:  " << saturated_vel(0) << " The angular vel is: " << saturated_vel(1);
  
  Command command;
  command.linear.x = saturated_vel(0);
  command.angular.z = saturated_vel(1);
  prev_vel_stamp_ = now();
  applied_vel_ = saturated_vel;

  // Store the result in memory so we can use previous state values to re-initialize and extrapolate the robot pose in subsequent iterations
  vel_history.erase(vel_history.begin());
  vel_history.push_back(applied_vel_);

  CLOG(INFO, "cbit.control")
    << "Final control command: [" << command.linear.x << ", "
    << command.linear.y << ", " << command.linear.z << ", "
    << command.angular.x << ", " << command.angular.y << ", "
    << command.angular.z << "] for timestamp: " << prev_cost_stamp_;
  
  return command;
}


// Generate twist commands to track the planned local path (function is called at the control rate)
auto CBIT::computeCommand_(RobotState& robot_state) -> Command {
  auto& chain = robot_state.chain.ptr();
  if (!chain->isLocalized()) {
    CLOG(WARNING, "cbit.control") << "Robot is not localized, commanding the robot to stop";
    return Command();
  }

  // retrieve the transform info from the localization chain for the current robot state
  const auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] = getChainInfo(*chain);

  // Store the current robot state in the robot state path so it can be visualized
  auto T_w_r = T_w_p * T_p_r;
  robot_poses.push_back(T_w_r);

  // Handling Dynamic Corridor Widths:
  // Retrieve the terrain type (corresponds to maximum planning width)
  if (curr_sid <= chain->size() - 2)
  {
    CLOG(DEBUG, "cbit.control") << "Trying to query the graph vertex type for SID: " << curr_sid;
    int vertex_type = chain->query_terrain_type(curr_sid);

    // Calculate q_max width for planner
    *q_max_ptr = pose_graph::BasicPathBase::terrian_type_corridor_width(vertex_type);
    CLOG(DEBUG, "cbit.control") << "Vertex Corridor Width is: " << *q_max_ptr; // we may want to look ahead a few frames instead

  }

  // Retrieve the latest obstacle costmap
  if ((prev_cost_stamp_ != stamp) && (config_->obstacle_avoidance == true))
  {
    
    std::lock_guard<std::mutex> lock(robot_state.obsMapMutex);
    // Read or use output.obs_map safely
    auto obs_map = robot_state.obs_map;
    const auto costmap_sid = robot_state.costmap_sid;
    const auto T_start_vertex = chain->pose(costmap_sid);
    costmap_ptr->grid_resolution = robot_state.grid_resolution;
  
    CLOG(DEBUG, "cbit.obstacle_filtering") << "The size of the map is: " << obs_map.size();

    // Updating the costmap pointer
    CLOG(DEBUG, "cbit.obstacle_filtering") << "Updating Costmap SID to: " << costmap_sid;
    costmap_ptr->obs_map = obs_map;
    // Store the transform T_c_w (from costmap to world)
    costmap_ptr->T_c_w = T_start_vertex.inverse(); // note that T_start_vertex is T_w_c if we want to bring keypoints to the world frame
    // Store the grid resolution
    CLOG(DEBUG, "cbit.obstacle_filtering") << "The costmap to world transform is: " << T_start_vertex.inverse();

    // Storing sequences of costmaps for temporal filtering purposes
    // For the first x iterations, fill the obstacle vector
    if (costmap_ptr->obs_map_vect.size() < (unsigned) config_->costmap_history)
    {
      costmap_ptr->obs_map_vect.push_back(obs_map);
      costmap_ptr->T_c_w_vect.push_back(costmap_ptr->T_c_w);
    }
    // After that point, we then do a sliding window using shift operations, moving out the oldest map and appending the newest one
    else
    {
      //TODO looks like this shold use a list, it looks like it's just overwriting the last one.
      costmap_ptr->obs_map_vect[config_->costmap_history-1] = obs_map;
      costmap_ptr->T_c_w_vect[config_->costmap_history-1] = costmap_ptr->T_c_w ;
    }
  }
  prev_cost_stamp_ = stamp;
  // END OF OBSTACLE PERCEPTION UPDATES

  // Make sure there is a valid solution, else stop the robot
  if (*valid_solution_ptr == false)
  {
    CLOG(INFO, "cbit.control") << "There is Currently No Valid Solution, Disabling MPC";
    return Command();
  }

  // START OF MPC CODE
  // Dont proceed to mpc control unless we have a valid plan to follow from BIT*, else return a 0 velocity command to stop and wait
  if (cbit_path_ptr->size() != 0)
  {
    // EXTRAPOLATING ROBOT POSE INTO THE FUTURE TO COMPENSATE FOR SYSTEM DELAYS
    // Removing for now. I'm not sure this is a good idea with noisy velocity estimates
    auto T_p_r_extp = T_p_r;
    if (config_->extrapolate_robot_pose) {
      const auto curr_time = now();  // always in nanoseconds
      auto dt = static_cast<double>(curr_time - stamp) * 1e-9 - 0.05;
      if (fabs(dt) > 0.25) { 
        CLOG(WARNING, "cbit") << "Pose extrapolation was requested but the time delta is " << dt << "s.\n"
              << "Ignoring extrapolation requestion. Check your time sync!";
        dt = 0;
      }

      CLOG(DEBUG, "cbit.debug") << "Robot velocity Used for Extrapolation: " << -w_p_r_in_r.transpose() << " dt: " << dt << std::endl;
      Eigen::Matrix<double, 6, 1> xi_p_r_in_r(-dt * w_p_r_in_r);
      T_p_r_extp = T_p_r * tactic::EdgeTransform(xi_p_r_in_r);

      CLOG(DEBUG, "cbit.debug") << "New extrapolated pose:"  << T_p_r_extp;
    }
    CLOG(DEBUG, "cbit.control") << "Last velocity " << w_p_r_in_r << " with stamp " << stamp;

    double state_p = findRobotP(T_w_p * T_p_r_extp, chain);
    std::shared_ptr<PoseResultHomotopy> referenceInfo;

    CasadiMPC::Config::Ptr baseMpcConfig;

    if (config_->kinematic_model == "unicycle"){
      CasadiUnicycleMPC::Config mpcConfig;
      mpcConfig.vel_max = {config_->max_lin_vel, config_->max_ang_vel};

      // Initializations from config
      
      // Schedule speed based on path curvatures + other factors
      // TODO refactor to accept the chain and use the curvature of the links
      mpcConfig.VF = ScheduleSpeed(chain, {config_->forward_vel, config_->min_vel, config_->planar_curv_weight, config_->profile_curv_weight, config_->eop_weight, 7});

      lgmath::se3::Transformation T0 = T_p_r_extp;
      mpcConfig.T0 = tf_to_global(T0);

      std::vector<double> p_rollout;
      for(int j = 1; j < mpcConfig.N+1; j++){
        p_rollout.push_back(state_p + j*mpcConfig.VF*mpcConfig.DT);
      }

      referenceInfo = std::make_shared<PoseResultHomotopy>(generateHomotopyReference(p_rollout, chain));

      mpcConfig.reference_poses.clear();

      for(const auto& Tf : referenceInfo->poses) {
        mpcConfig.reference_poses.push_back(tf_to_global(T_w_p.inverse() *  Tf));
        CLOG(DEBUG, "test") << "Target " << tf_to_global(T_w_p.inverse() *  Tf);
      }

      mpcConfig.up_barrier_q = referenceInfo->barrier_q_max;
      mpcConfig.low_barrier_q = referenceInfo->barrier_q_min;
      
      mpcConfig.previous_vel = {-w_p_r_in_r(0, 0), -w_p_r_in_r(5, 0)};
      baseMpcConfig = std::make_shared<CasadiMPC::Config>(mpcConfig);
    } else if (config_->kinematic_model == "ackermann") {
      CasadiAckermannMPC::Config mpcConfig;
      mpcConfig.vel_max = {config_->max_lin_vel, config_->max_ang_vel};
      mpcConfig.turning_radius = config_->turning_radius;

      // Initializations from config
      
      // Schedule speed based on path curvatures + other factors
      // TODO refactor to accept the chain and use the curvature of the links
      mpcConfig.VF = ScheduleSpeed(chain, {config_->forward_vel, config_->min_vel, config_->planar_curv_weight, config_->profile_curv_weight, config_->eop_weight, 7});

      lgmath::se3::Transformation T0 = T_p_r_extp;
      mpcConfig.T0 = tf_to_global(T0);

      std::vector<double> p_rollout;
      for(int j = 1; j < mpcConfig.N+1; j++){
        p_rollout.push_back(state_p + j*mpcConfig.VF*mpcConfig.DT);
      }

      referenceInfo = std::make_shared<PoseResultHomotopy>(generateHomotopyReference(p_rollout, chain));
      mpcConfig.reference_poses.clear();
      for(const auto& Tf : referenceInfo->poses) {
        mpcConfig.reference_poses.push_back(tf_to_global(T_w_p.inverse() *  Tf));
        CLOG(DEBUG, "test") << "Target " << tf_to_global(T_w_p.inverse() *  Tf);
      }
      
      mpcConfig.previous_vel = {-w_p_r_in_r(0, 0), -w_p_r_in_r(5, 0)};
    
      baseMpcConfig = std::make_shared<CasadiMPC::Config>(mpcConfig);
    }
   

    // Create and solve the casadi optimization problem
    std::vector<lgmath::se3::Transformation> mpc_poses;
    // return the computed velocity command for the first time step
    Command command;
    std::vector<Eigen::Vector2d> mpc_velocities;
    try {
      CLOG(INFO, "cbit.control") << "Attempting to solve the MPC problem";
      auto mpc_res = solver_->solve(*baseMpcConfig);
      
      for(int i = 0; i < mpc_res["pose"].columns(); i++) {
        const auto& pose_i = mpc_res["pose"](casadi::Slice(), i).get_elements();
        mpc_poses.push_back(T_w_p * tf_from_global(pose_i[0], pose_i[1], pose_i[2]));
      }

      CLOG(INFO, "cbit.control") << "Successfully solved MPC problem";
      const auto& mpc_vel_vec = mpc_res["vel"](casadi::Slice(), 0).get_elements();

      command.linear.x = mpc_vel_vec[0];
      command.angular.z = mpc_vel_vec[1];

      // Get all the mpc velocities 
      for (int i = 0; i < mpc_res["vel"].columns(); i++) {
        const auto& vel_i = mpc_res["vel"](casadi::Slice(), i).get_elements();
        mpc_velocities.emplace_back(vel_i[0], vel_i[1]);
      }

    } catch(std::exception &e) {
      CLOG(WARNING, "cbit.control") << "casadi failed! " << e.what() << " Commanding to Stop the Vehicle";
      return Command();
    }


    CLOG(INFO, "cbit.control") << "The linear velocity is:  " << command.linear.x << " The angular vel is: " << command.angular.z;

    // grab elements for visualization
    lgmath::se3::Transformation T_w_p_interpolated_closest_to_robot = interpolatedPose(state_p, chain);

    // visualize the outputs
    visualization_ptr->visualize(stamp, T_w_p, T_p_r, T_p_r_extp, T_w_r, mpc_poses, mpc_velocities, robot_poses, referenceInfo->poses, referenceInfo->poses, cbit_path_ptr, corridor_ptr, T_w_p_interpolated_closest_to_robot, state_p, global_path_ptr, curr_sid);

    return command;
  }
  // Otherwise stop the robot
  else  {
    CLOG(INFO, "cbit.control") << "There is not a valid plan yet, returning zero velocity commands";

    return Command();
  }
}


// Simple function for checking that the current output velocity command is saturated between our mechanical velocity limits
Eigen::Vector2d saturateVel(const Eigen::Vector2d& applied_vel, double v_lim, double w_lim) {
  return {std::clamp(applied_vel(0), -v_lim, v_lim), std::clamp(applied_vel(1), -w_lim, w_lim)};
}

}  // namespace path_planning
}  // namespace vtr