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
#include "vtr_path_planning/mpc/mpc_path_planner.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

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
  config->frame_interval = node->declare_parameter<int>(prefix + ".cbit.frame_interval", config->frame_interval); // going to get rid of this
  config->iter_max = node->declare_parameter<int>(prefix + ".cbit.iter_max", config->iter_max); // going to get rid of this
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
  config->extrapolate_robot_pose = node->declare_parameter<bool>(prefix + ".mpc.extrapolate_robot_pose", config->extrapolate_robot_pose);
  config->mpc_verbosity = node->declare_parameter<bool>(prefix + ".mpc.mpc_verbosity", config->mpc_verbosity);
  config->homotopy_guided_mpc = node->declare_parameter<bool>(prefix + ".mpc.homotopy_guided_mpc", config->homotopy_guided_mpc);
  config->horizon_steps = node->declare_parameter<int>(prefix + ".mpc.horizon_steps", config->horizon_steps);
  config->horizon_step_size = node->declare_parameter<double>(prefix + ".mpc.horizon_step_size", config->horizon_step_size);
  config->forward_vel = node->declare_parameter<double>(prefix + ".mpc.forward_vel", config->forward_vel);
  config->max_lin_vel = node->declare_parameter<double>(prefix + ".mpc.max_lin_vel", config->max_lin_vel);
  config->max_ang_vel = node->declare_parameter<double>(prefix + ".mpc.max_ang_vel", config->max_ang_vel);
  config->robot_linear_velocity_scale = node->declare_parameter<double>(prefix + ".mpc.robot_linear_velocity_scale", config->robot_linear_velocity_scale);
  config->robot_angular_velocity_scale = node->declare_parameter<double>(prefix + ".mpc.robot_angular_velocity_scale", config->robot_angular_velocity_scale);

  // MPC COST FUNCTION COVARIANCE
  const auto pose_error_diag = node->declare_parameter<std::vector<double>>(prefix + ".mpc.pose_error_cov", std::vector<double>());
  config->pose_error_cov.diagonal() << pose_error_diag[0], pose_error_diag[1], pose_error_diag[2], pose_error_diag[3], pose_error_diag[4], pose_error_diag[5];
  const auto vel_error_diag = node->declare_parameter<std::vector<double>>(prefix + ".mpc.vel_error_cov", std::vector<double>());
  config->vel_error_cov.diagonal() << vel_error_diag[0], vel_error_diag[1];
  const auto acc_error_diag = node->declare_parameter<std::vector<double>>(prefix + ".mpc.acc_error_cov", std::vector<double>());
  config->acc_error_cov.diagonal() << acc_error_diag[0], acc_error_diag[1];
  const auto kin_error_diag = node->declare_parameter<std::vector<double>>(prefix + ".mpc.kin_error_cov", std::vector<double>());
  config->kin_error_cov.diagonal() << kin_error_diag[0], kin_error_diag[1], kin_error_diag[2], kin_error_diag[3], kin_error_diag[4], kin_error_diag[5];
  const auto lat_error_diag = node->declare_parameter<std::vector<double>>(prefix + ".mpc.lat_error_cov", std::vector<double>());
  config->lat_error_cov.diagonal() << lat_error_diag[0];

  // MPC COST FUNCTION WEIGHTS
  config->pose_error_weight = node->declare_parameter<double>(prefix + ".mpc.pose_error_weight", config->pose_error_weight);
  config->vel_error_weight = node->declare_parameter<double>(prefix + ".mpc.vel_error_weight", config->vel_error_weight);
  config->acc_error_weight = node->declare_parameter<double>(prefix + ".mpc.acc_error_weight", config->acc_error_weight);
  config->kin_error_weight = node->declare_parameter<double>(prefix + ".mpc.kin_error_weight", config->kin_error_weight);
  config->lat_error_weight = node->declare_parameter<double>(prefix + ".mpc.lat_error_weight", config->lat_error_weight);

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
  // Initialize the shared pointer to the output of the planner
  cbit_path_ptr = std::make_shared<std::vector<Pose>> (cbit_path);
  valid_solution_ptr = std::make_shared<bool> (false);
  q_max_ptr = std::make_shared<double> (config->q_max);

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
  cbit_config.frame_interval = config->frame_interval;
  cbit_config.iter_max = config->iter_max;
  cbit_config.eta = config->eta;
  cbit_config.rad_m_exhange = config->rad_m_exhange;
  cbit_config.initial_exp_rad = config->initial_exp_rad;
  cbit_config.extrapolation = config->extrapolation;

  // Misc
  cbit_config.incremental_plotting = config->incremental_plotting;
  cbit_config.plotting = config->plotting;
  CLOG(INFO, "cbit.path_planning") << "Successfully Constructed the CBIT Class";


  // Initialize the current velocity state and a vector for storing a history of velocity commands applied
  applied_vel << 0,
                 0;
  vel_history.reserve(config_->command_history_length);
  for (int i = 0; i < config_->command_history_length; i++)
  {
    vel_history.push_back(applied_vel);
  }

  thread_count_ = 2;
  process_thread_cbit_ = std::thread(&CBIT::process_cbit, this);
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
    CLOG(INFO, "cbit.path_planning") << "Initializing CBIT Route";
    initializeRoute(*robot_state_);
    CLOG(INFO, "cbit.path_planning") << "CBIT Plan Completed";
  }
}




// Here is where we do all the teach path pre-processing and then begin the anytime planner asychronously
void CBIT::initializeRoute(RobotState& robot_state) {
  auto& chain = *robot_state.chain;

  // Wait until the chain becomes localized
  while (!chain.isLocalized())
  {
  }

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


  // Using two consecutive poses on the path, we need to try to determine which direction the repeat is going before we generate the pq space:
  const auto chain_info = getChainInfo(robot_state);
  auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] = chain_info;
  auto world_frame_pose = T2xyzrpy(T_w_p * T_p_r);
  auto first_pose = euclid_path_vec[0];
  auto second_pose = euclid_path_vec[1];
  auto path_yaw = std::atan2((second_pose.y-first_pose.y),(second_pose.x-first_pose.x));
  auto pose_graph_yaw = std::get<5>(world_frame_pose);

  // Logic for determining the forward/reverse sign:
  PathDirection path_direction; //1.0 = forward planning, -1.0 = reverse planning
  if (abs((abs(path_yaw) - abs(pose_graph_yaw))) > 1.57075)
  {
    path_direction = PATH_DIRECTION_REVERSE;
  }
  else
  {
    path_direction = PATH_DIRECTION_FORWARD;
  }

  CLOG(INFO, "cbit.path_planning") << "The path repeat direction is:" << path_direction;
  CLOG(INFO, "cbit.path_planning") << "Trying to create global path";
  // Create the path class object (Path preprocessing)
  CBITPath global_path(cbit_config, euclid_path_vec);
  // Make a pointer to this path
  global_path_ptr = std::make_shared<CBITPath>(global_path);
  CLOG(INFO, "cbit.path_planning") << "Teach Path has been pre-processed. Attempting to initialize the dynamic corridor";


  // Initialize the dynamic corridor
  CBITCorridor corridor(cbit_config, global_path_ptr);
  // Make a pointer to the corridor
  corridor_ptr = std::make_shared<CBITCorridor>(corridor);
  CLOG(INFO, "cbit.path_planning") << "Corridor generated successfully. Attempting to instantiate the planner";

  // Instantiate the planner
  CBITPlanner cbit(cbit_config, global_path_ptr, robot_state, cbit_path_ptr, costmap_ptr, corridor_ptr, valid_solution_ptr, q_max_ptr, path_direction);
  CLOG(INFO, "cbit.path_planning") << "Planner successfully created and resolved";
}





// Generate twist commands to track the planned local path (function is called at the control rate)
auto CBIT::computeCommand(RobotState& robot_state) -> Command {
  auto& chain = *robot_state.chain;
  if (!chain.isLocalized()) {
    CLOG(WARNING, "cbit.control") << "Robot is not localized, commanding the robot to stop";
    applied_vel << 0.0, 0.0;
    // Update history:
    vel_history.erase(vel_history.begin());
    vel_history.push_back(applied_vel);
    return Command();
  }

  // retrieve the transorm info from the localization chain for the current robot state
  const auto chain_info = getChainInfo(robot_state);
  auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] = chain_info;
  if (curr_sid == (chain.size()-2))
  {
    CLOG(INFO, "cbit.control") << "Reaching End of Path, Disabling MPC";
    return Command();
  }


  // Handling Dynamic Corridor Widths:
  // Retrieve the terrain type (corresponds to maximum planning width)
  if (curr_sid <= chain.size()-2)
  {
    CLOG(DEBUG, "cbit.control") << "Trying to query the graph vertex type for SID: " << curr_sid;
    int vertex_type = chain.query_terrain_type(curr_sid);
    CLOG(DEBUG, "cbit.control") << "GUI Vertex Type is: " << (vertex_type + 1); // we may want to look ahead a few frames instead
    
    // Type 1 in GUI is default and returns the value 0. This should correspond to 2.5m maximum lateral deviation.
    // Each subsequent type maps to Type # - 1, i.e. Type 2 path = 1, Type 3 = 2, and so on.

    // Calculate q_max width for planner
    *q_max_ptr = 2.5 - (0.5 * vertex_type);
    // Note: Minimum should never go to exactly zero or this could cause issues in the planner (0.01 is fine)
    // TODO: Update the type values in the GUI to reflect this change
  }

  // Retrieve the latest obstacle costmap
  bool obstacle_avoidance = config_->obstacle_avoidance;
  if ((prev_stamp != stamp) && (obstacle_avoidance == true))
  {
    
    std::lock_guard<std::mutex> lock(robot_state.obsMapMutex);
    // Read or use output.obs_map safely
    auto obs_map = robot_state.obs_map;
    const auto costmap_sid = robot_state.costmap_sid;
    const auto T_start_vertex = chain.pose(costmap_sid);
    costmap_ptr->grid_resolution = robot_state.grid_resolution;
  
    CLOG(DEBUG, "cbit.obstacle_filtering") << "The size of the map is: " << obs_map.size();

    // Updating the costmap pointer
    CLOG(DEBUG, "cbit.obstacle_filtering") << "Updating Costmap SID to: " << costmap_sid;
    costmap_ptr->obs_map = obs_map;
    // Store the transform T_c_w (from costmap to world)
    costmap_ptr->T_c_w = T_start_vertex.inverse(); // note that T_start_vertex is T_w_c if we want to bring keypoints to the world frame
    // Store the grid resoltuion
    CLOG(DEBUG, "cbit.obstacle_filtering") << "The costmap to world transform is: " << T_start_vertex.inverse();

    // Storing sequences of costmaps for temporal filtering purposes
    // For the first x iterations, fill the obstacle vector
    if (costmap_ptr->obs_map_vect.size() < config_->costmap_history)
    {
      costmap_ptr->obs_map_vect.push_back(obs_map);
      costmap_ptr->T_c_w_vect.push_back(costmap_ptr->T_c_w);
    }
    // After that point, we then do a sliding window using shift operations, moving out the oldest map and appending the newest one
    else
    {
      costmap_ptr->obs_map_vect[config_->costmap_history-1] = obs_map;
      costmap_ptr->T_c_w_vect[config_->costmap_history-1] = costmap_ptr->T_c_w ;
    }
  }
  prev_stamp = stamp;
  // END OF OBSTACLE PERCEPTION UPDATES

  // Make sure there is a valid solution, else stop the robot
  if (*valid_solution_ptr == false)
  {
    CLOG(INFO, "cbit.control") << "There is Currently No Valid Solution, Disabling MPC";
    return Command();
  }

  // START OF MPC CODE
  // Dont proceed to mpc control unless we have a valid plan to follow from BIT*, else return a 0 velocity command to stop and wait
  if ((*cbit_path_ptr).size() != 0)
  {
    CLOG(DEBUG, "cbit.debug") << "History of the Robot Velocities:" << vel_history;

    // Initializations from config
    bool extrapolate_robot_pose = config_->extrapolate_robot_pose;
    bool mpc_verbosity = config_->mpc_verbosity;
    bool homotopy_guided_mpc = config_->homotopy_guided_mpc;
    int K = config_->horizon_steps; // Horizon steps
    double DT = config_->horizon_step_size; // Horizon step size
    double VF = config_->forward_vel; // Desired Forward velocity set-point for the robot. MPC will try to maintain this rate while balancing other constraints
    
    // Schedule speed based on path curvatures + other factors
    VF = ScheduleSpeed(global_path_ptr->disc_path_curvature_xy, global_path_ptr->disc_path_curvature_xz_yz, VF, curr_sid, config_->planar_curv_weight, config_->profile_curv_weight, config_->eop_weight, config_->horizon_step_size, config_->min_vel);

    // Grab the current MPC configurations
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

    // Lateral Covariance Weights (should be weighted quite heavily (smaller is higher because its covariance))
    Eigen::Matrix<double, 1, 1> lat_noise_vect;
    lat_noise_vect = config_->lat_error_cov;

    // Cost term weights
    double pose_error_weight = config_->pose_error_weight;
    double vel_error_weight = config_->vel_error_weight;
    double acc_error_weight = config_->acc_error_weight;
    double kin_error_weight = config_->kin_error_weight;
    double lat_error_weight = config_->lat_error_weight;
  


    // EXTRAPOLATING ROBOT POSE INTO THE FUTURE TO COMPENSATE FOR SYSTEM DELAYS
    auto T_p_r_extp = T_p_r;
    auto T_p_r_prop = T_p_r;
    if (extrapolate_robot_pose == true)
    {
      const auto curr_time = now();  // always in nanoseconds
      const auto dt = static_cast<double>(curr_time - stamp) * 1e-9;

      // Check the time past since the last state update was received
      // Go back through the vel_history to approximately dt seconds in the past
      // Start applying each of the applied velocities sequentially
      double control_period = config_->control_period / 1000.0; // control period is given by user in ms in the config
      for (int i=std::floor(dt / control_period); i > 0; i--)
      {
        w_p_r_in_r(0) = -1* vel_history[vel_history.size()-(i+1)][0];
        w_p_r_in_r(1) = 0.0;
        w_p_r_in_r(2) = 0.0;
        w_p_r_in_r(3) = 0.0;
        w_p_r_in_r(4) = 0.0;
        w_p_r_in_r(5) = -1* vel_history[vel_history.size()-(i+1)][1];
        CLOG(DEBUG, "cbit.debug") << "Robot velocity Used for Extrapolation: " << -w_p_r_in_r.transpose() << std::endl;

        Eigen::Matrix<double, 6, 1> xi_p_r_in_r(control_period * w_p_r_in_r);
        T_p_r_prop = T_p_r_prop * tactic::EdgeTransform(xi_p_r_in_r).inverse();

      }
      // Apply the final partial period velocity
      w_p_r_in_r(0) = -1* vel_history.back()[0];
      w_p_r_in_r(1) = 0.0;
      w_p_r_in_r(2) = 0.0;
      w_p_r_in_r(3) = 0.0;
      w_p_r_in_r(4) = 0.0;
      w_p_r_in_r(5) = -1* vel_history.back()[1];
      CLOG(DEBUG, "cbit.debug") << "Robot velocity Used for Extrapolation: " << -w_p_r_in_r.transpose() << std::endl;
      Eigen::Matrix<double, 6, 1> xi_p_r_in_r((dt - (std::floor(dt / control_period) * control_period)) * w_p_r_in_r);
      T_p_r_prop = T_p_r_prop * tactic::EdgeTransform(xi_p_r_in_r).inverse();
      CLOG(DEBUG, "cbit.debug") << "The final time period is: "  << (dt - (std::floor(dt / control_period) * control_period));
      T_p_r_extp = T_p_r_prop;

      CLOG(DEBUG, "cbit.debug") << "New extrapolated pose:"  << T_p_r_extp;
    }

    lgmath::se3::Transformation T0 = lgmath::se3::Transformation(T_w_p * T_p_r_extp);

    //Convert to x,y,z,roll, pitch, yaw
    std::tuple<double, double, double, double, double, double> robot_pose = T2xyzrpy(T0);
    CLOG(DEBUG, "cbit.control") << "The Current Robot Pose (from planning) is - x: " << std::get<0>(robot_pose) << " y: " << std::get<1>(robot_pose) << " yaw: " << std::get<5>(robot_pose);

    CLOG(DEBUG, "cbit.control") << "The Current Robot State Transform is: : " << T0;
    // Need to also invert the robot state to make it T_vi instead of T_iv as this is how the MPC problem is structured
    lgmath::se3::Transformation T0_inv = T0.inverse();
    CLOG(DEBUG, "cbit.control") << "The Inverted Current Robot State Using Direct Robot Values is: " << T0_inv;
    // END OF ROBOT POSE EXTRAPOLATION



    // Calculate which T_ref poses to used based on the current path solution
    CLOG(INFO, "cbit.control") << "Attempting to generate T_ref poses";
    auto ref_tracking_result = GenerateTrackingReference(cbit_path_ptr, robot_pose, K,  DT, VF);
    auto tracking_poses = ref_tracking_result.poses;
    //bool point_stabilization = ref_tracking_result.point_stabilization; // No need to do both tracking and homotopy point stabilization
    std::vector<double> p_interp_vec = ref_tracking_result.p_interp_vec;
    std::vector<double> q_interp_vec = ref_tracking_result.q_interp_vec;

    // Synchronized Tracking/Teach Reference Poses For Homotopy Guided MPC:
    auto ref_homotopy_result = GenerateHomotopyReference(global_path_ptr, corridor_ptr, robot_pose, K,  DT, VF, curr_sid, p_interp_vec);
    auto homotopy_poses = ref_homotopy_result.poses;
    bool point_stabilization = ref_homotopy_result.point_stabilization;
    std::vector<double> barrier_q_left = ref_homotopy_result.barrier_q_left;
    std::vector<double> barrier_q_right = ref_homotopy_result.barrier_q_right;
    std::vector<lgmath::se3::Transformation> tracking_pose_vec;
    for (int i = 0; i<tracking_poses.size(); i++)
    {
      tracking_pose_vec.push_back(tracking_poses[i].inverse());
    }
    std::vector<lgmath::se3::Transformation> homotopy_pose_vec;
    for (int i = 0; i<homotopy_poses.size(); i++)
    {
      homotopy_pose_vec.push_back(homotopy_poses[i].inverse());
    }

    // Generate the mpc configuration structure:
    MPCConfig mpc_config;
    mpc_config.previous_vel = applied_vel;
    mpc_config.T0 = T0;
    mpc_config.tracking_reference_poses = tracking_poses;
    if (homotopy_guided_mpc == true)
    {
      mpc_config.homotopy_reference_poses = homotopy_poses;
    }
    else
    {
      mpc_config.homotopy_reference_poses = tracking_poses; // if not using homotopy guided mpc, set the homotopy reference poses to be the tracking poses
    }
    mpc_config.barrier_q_left = barrier_q_left;
    mpc_config.barrier_q_right = barrier_q_right;
    mpc_config.K = K;
    mpc_config.DT = DT;
    mpc_config.VF = VF;
    mpc_config.lat_noise_vect = lat_noise_vect;
    mpc_config.pose_noise_vect = pose_noise_vect;
    mpc_config.vel_noise_vect = vel_noise_vect;
    mpc_config.accel_noise_vect = accel_noise_vect;
    mpc_config.kin_noise_vect = kin_noise_vect;
    mpc_config.point_stabilization = point_stabilization;
    mpc_config.pose_error_weight = pose_error_weight;
    mpc_config.vel_error_weight = vel_error_weight;
    mpc_config.acc_error_weight = acc_error_weight;
    mpc_config.kin_error_weight = kin_error_weight;
    mpc_config.lat_error_weight = lat_error_weight;
    mpc_config.verbosity = mpc_verbosity;
    mpc_config.homotopy_mode = homotopy_guided_mpc;

    // Create and solve the STEAM optimization problem
    std::vector<lgmath::se3::Transformation> mpc_poses;
    try
    {
      CLOG(INFO, "cbit.control") << "Attempting to solve the MPC problem";
      auto MPCResult = SolveMPC(mpc_config);
      applied_vel = MPCResult.applied_vel; // note dont re-declare applied vel here
      mpc_poses = MPCResult.mpc_poses;
      CLOG(INFO, "cbit.control") << "Successfully solved MPC problem";
    }
    catch(...)
    {
      CLOG(WARNING, "cbit.control") << "STEAM Optimization Failed; Commanding to Stop the Vehicle";
      applied_vel(0) = 0.0;
      applied_vel(1) = 0.0;
    }

    CLOG(INFO, "cbit.control") << "The linear velocity is:  " << applied_vel(0) << " The angular vel is: " << applied_vel(1);

    // Apply robot motor controller calibration scaling factors if applicable
    applied_vel(0) = applied_vel(0) * config_->robot_linear_velocity_scale;
    applied_vel(1) = applied_vel(1) * config_->robot_angular_velocity_scale;

    // If required, saturate the output velocity commands based on the configuration limits
    CLOG(INFO, "cbit.control") << "Saturating the velocity command if required";
    Eigen::Matrix<double, 2, 1> saturated_vel = SaturateVel(applied_vel, config_->max_lin_vel, config_->max_ang_vel);
    CLOG(INFO, "cbit.control") << "The Saturated linear velocity is:  " << saturated_vel(0) << " The angular vel is: " << saturated_vel(1);
    
    // Store the result in memory so we can use previous state values to re-initialize and extrapolate the robot pose in subsequent iterations
    vel_history.erase(vel_history.begin());
    vel_history.push_back(saturated_vel);

    // Store the current robot state in the robot state path so it can be visualized
    robot_poses.push_back(T_w_p * T_p_r);

    // visualize the outputs
    visualization_ptr->visualize(stamp, T_w_p, T_p_r, T_p_r_extp, mpc_poses, robot_poses, tracking_pose_vec, homotopy_pose_vec, cbit_path_ptr, corridor_ptr);

    // return the computed velocity command for the first time step
    Command command;
    command.linear.x = saturated_vel(0);
    command.angular.z = saturated_vel(1);
    
    // Flagged log messages to test calibration of the grizzly controller (or any future robot)
    CLOG(DEBUG, "grizzly_controller_tests.cbit") << "Twist Linear Velocity: " << saturated_vel(0);
    CLOG(DEBUG, "grizzly_controller_tests.cbit") << "Twist Angular Velocity: " << saturated_vel(1);

    
    CLOG(INFO, "cbit.control")
      << "Final control command: [" << command.linear.x << ", "
      << command.linear.y << ", " << command.linear.z << ", "
      << command.angular.x << ", " << command.angular.y << ", "
      << command.angular.z << "]";

    return command;
  }
  // Otherwise stop the robot
  else
  {
    CLOG(INFO, "cbit.control") << "There is not a valid plan yet, returning zero velocity commands";

    applied_vel << 0.0, 0.0;
    vel_history.erase(vel_history.begin());
    vel_history.push_back(applied_vel);
    return Command();
  }
}


// Function for grabbing the robots velocity in planning frame, transform of robot into planning frame, and transform of planning frame to world frame
auto CBIT::getChainInfo(RobotState& robot_state) -> ChainInfo {
  auto& chain = *robot_state.chain;
  auto lock = chain.guard();
  const auto stamp = chain.leaf_stamp();
  const auto w_p_r_in_r = chain.leaf_velocity();
  const auto T_p_r = chain.T_leaf_trunk().inverse();
  const auto T_w_p = chain.T_start_trunk();
  const auto T_w_v_odo = chain.T_start_petiole();
  const auto T_r_v_odo = chain.T_leaf_petiole();
  const auto curr_sid = chain.trunkSequenceId();
  return ChainInfo{stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid};
}


}  // namespace path_planning
}  // namespace vtr