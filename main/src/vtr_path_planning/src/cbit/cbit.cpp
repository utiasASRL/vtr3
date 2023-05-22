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
//#include "vtr_path_planning/mpc/mpc_path_planner.hpp"
#include "vtr_path_planning/mpc/mpc_path_planner2.hpp"


#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

namespace vtr {
namespace path_planning {

namespace {
// Function for converting Transformation matrices into se(2) [x, y, z, roll, pitch, yaw]
inline std::tuple<double, double, double, double, double, double> T2xyzrpy(
    const tactic::EdgeTransform& T) {
  const auto Tm = T.matrix();
  return std::make_tuple(Tm(0, 3), Tm(1, 3), Tm(2,3), std::atan2(Tm(2, 1), Tm(2, 2)), std::atan2(-1*Tm(2, 0), sqrt(pow(Tm(2, 1),2) + pow(Tm(2, 2),2))), std::atan2(Tm(1, 0), Tm(0, 0)));
}
}

// Configure the class as a ROS2 node, get configurations from the ros parameter server
auto CBIT::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
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
  config->robot_linear_velocity_scale = node->declare_parameter<double>(prefix + ".robot_linear_velocity_scale", config->robot_linear_velocity_scale);
  config->robot_angular_velocity_scale = node->declare_parameter<double>(prefix + ".robot_angular_velocity_scale", config->robot_angular_velocity_scale);


  // COST FUNCTION COVARIANCE
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

  // COST FUNCTION WEIGHTS
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
  CLOG(INFO, "path_planning.cbit") << "Constructing the CBIT Class";
  robot_state_ = robot_state;
  const auto node = robot_state->node.ptr();
  // Initialize the shared pointer to the output of the planner
  cbit_path_ptr = std::make_shared<std::vector<Pose>> (cbit_path);

  // Create publishers
  tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  mpc_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("mpc_prediction", 10);
  robot_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("robot_path", 10);
  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("planning_path", 10);
  corridor_pub_l_ = node->create_publisher<nav_msgs::msg::Path>("corridor_path_left", 10);
  corridor_pub_r_ = node->create_publisher<nav_msgs::msg::Path>("corridor_path_right", 10);
  ref_pose_pub1_ = node->create_publisher<geometry_msgs::msg::PoseArray>("mpc_ref_pose_array1", 10);
  ref_pose_pub2_ = node->create_publisher<geometry_msgs::msg::PoseArray>("mpc_ref_pose_array2", 10);

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
  CLOG(INFO, "path_planning.cbit") << "Successfully Constructed the CBIT Class";


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
  CLOG(INFO, "path_planning.cbit") << "Starting the CBIT Planning Thread";
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
        CLOG(INFO, "path_planning.cbit") << "Stopping the CBIT Thread.";
        cv_thread_finish_.notify_all();
        return;
    }


    // Planner should not require the thread lock to execute
    lock.unlock();

    // Note we need to run the above first so that the lidarcbit class can be constructed before calling initializeroute (so it can be overrided correctly)
    CLOG(INFO, "path_planning.cbit") << "Initializing CBIT Route";
    initializeRoute(*robot_state_);
    CLOG(INFO, "path_planning.cbit") << "CBIT Plan Completed";
  }
}

// Here is where we can do all the teach path pre-processing and then begin the anytime planner asychronously
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
    //CLOG(ERROR, "path_planning.cbit") << "Global Path Yaw values: " << i << ": " << std::get<5>(se3_vector);
  }


  // experimental, trying to determine sign for path following direction
  // Using two consecutive poses on the path, we need to try to determine which direction the repeat is going:
  const auto chain_info = getChainInfo(robot_state);
  auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] = chain_info;
  auto world_frame_pose = T2xyzrpy(T_w_p * T_p_r);
  auto test1 = euclid_path_vec[0];
  auto test2 = euclid_path_vec[1];
  auto path_yaw = std::atan2((test2.y-test1.y),(test2.x-test1.x));
  auto pose_graph_yaw = std::get<5>(world_frame_pose);
  CLOG(INFO, "path_planning.cbit") << "The path_yaw is: " << path_yaw;
  CLOG(INFO, "path_planning.cbit") << "The pose_graph yaw is: " << pose_graph_yaw;
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
  CLOG(INFO, "path_planning.cbit") << "The path repeat direction is:" << path_direction;


  CLOG(INFO, "path_planning.cbit") << "Trying to create global path";
  // Create the path class object (Path preprocessing)
  CBITPath global_path(cbit_config, euclid_path_vec);
  // Make a pointer to this path
  global_path_ptr = std::make_shared<CBITPath>(global_path);
  CLOG(INFO, "path_planning.cbit") << "Teach Path has been pre-processed. Attempting to initialize the dynamic corridor";


  // Initialize the dynamic corridor
  CBITCorridor corridor(cbit_config, global_path_ptr);
  // Make a pointer to the corridor
  corridor_ptr = std::make_shared<CBITCorridor>(corridor);
  CLOG(INFO, "path_planning.cbit") << "Corridor generated successfully. Attempting to instantiate the planner";


  // Instantiate the planner
  CBITPlanner cbit(cbit_config, global_path_ptr, robot_state, cbit_path_ptr, costmap_ptr, corridor_ptr, path_direction);
  CLOG(INFO, "path_planning.cbit") << "Planner successfully created and resolved";

}

// Generate twist commands to track the planned local path (obstacle free)
auto CBIT::computeCommand(RobotState& robot_state) -> Command {
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

  CLOG(INFO, "path_planning.cbit") << "The T_r_v_odo is: " << T_r_v_odo;
  CLOG(INFO, "path_planning.cbit") << "The T_p_r is: " << T_p_r;
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
    

    
    // Experimental Speed Scheduler: (TODO: in progress - move to separate file longer term)
    // Takes in the desired forward_velocity and the pre-processed global path and reduces the set speed based on a range of tunable factors:
    // 1. XY curvature (implemneted)
    // 2. YZ curvature (TODO)
    // 3. XZ curvature (TODO)
    // 4. Corridor Width (TODO)
    // 5. Obstacle Presence (TODO)

    // Pseudocode:
    // - Estimate the current p value of the vehicle (doesnt need to be super precise so here we can imply opt to use the sid value)
    // - Avergage the radius of curvature in the upcoming segments of the path
    // - TODO: generate other scaling factors
    // - Scale the forward velocity

    // Basic implementation - weights hardcoded for now
    CLOG(ERROR, "mpc_debug.cbit") << "TRYING TO SCHEDULE SPEED:";
    CLOG(ERROR, "mpc_debug.cbit") << "CURRENT SID IS:" << curr_sid;
    double VF_EOP;
    double VF_XY;
    double VF_XZ_YZ;
    double avg_curvature_xy = 0.0;
    double avg_curvature_xz_yz = 0.0;
    double end_of_path = 0.0;
    for (int i = curr_sid; i < curr_sid + 10; i++) // Lookahead hardcoded for now, todo, make this a distance based correlating value
    {
      // Handle end of path case
      if (i == (global_path_ptr->p.size()-1))
      {
        end_of_path = 1.0;
        break;
      }
      avg_curvature_xy = avg_curvature_xy + global_path_ptr->disc_path_curvature_xy[i];
      avg_curvature_xz_yz = avg_curvature_xz_yz + global_path_ptr->disc_path_curvature_xz_yz[i];

    }
    avg_curvature_xy = avg_curvature_xy / 10;
    avg_curvature_xz_yz = avg_curvature_xz_yz / 10;
    CLOG(ERROR, "mpc_debug.cbit") << "THE AVERAGE XY CURVATURE IS:  " << avg_curvature_xy;
    CLOG(ERROR, "mpc_debug.cbit") << "THE AVERAGE XZ CURVATURE IS:  " << avg_curvature_xz_yz;
    //CLOG(ERROR, "mpc_debug.cbit") << "THE AVERAGE YZ CURVATURE IS:  " << avg_curvature_yz;
    double xy_curv_weight = 5.0; // hardocded for now, make a param
    double xz_yz_curv_weight = 0.5; // hardocded for now, make a param
    double end_of_path_weight = 1.0; // hardocded for now, make a param

    // handle forward/referse case and calculate a candidate VF speed for each of our scheduler modules (XY curvature, XZ curvature, End of Path etc)

    VF_EOP = std::max(0.5, VF / (1 + (end_of_path * end_of_path * end_of_path_weight)));
    VF_XY = std::max(0.5, VF / (1 + (avg_curvature_xy * avg_curvature_xy * xy_curv_weight)));
    VF_XZ_YZ = std::max(0.5, VF / (1 + (avg_curvature_xz_yz * avg_curvature_xz_yz * xz_yz_curv_weight)));
    
    // Take the minimum of all candidate (positive) scheduled speeds
    VF = std::min({VF_EOP, VF_XY, VF_XZ_YZ});
    CLOG(ERROR, "mpc_debug.cbit") << "THE VF_EOP SPEED IS:  " << VF_EOP;
    CLOG(ERROR, "mpc_debug.cbit") << "THE VF_XY SPEED IS:  " << VF_XY;
    CLOG(ERROR, "mpc_debug.cbit") << "THE VF_XZ SPEED IS:  " << VF_XZ_YZ;

    // Take the minimum of all candidate scheduled speeds
    CLOG(ERROR, "mpc_debug.cbit") << "THE SPEED SCHEDULED SPEED IS:  " << VF;
    // End of speed scheduler code




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

    /*
    // Calculate which T_ref measurements to used based on the current path solution
    CLOG(INFO, "mpc.cbit") << "Attempting to generate T_ref measurements";
    auto meas_result = GenerateReferenceMeas2(cbit_path_ptr, robot_pose, K,  DT, VF);
    auto measurements = meas_result.measurements;
    bool point_stabilization = meas_result.point_stabilization;


    // Experimental, corridor MPC reference measurement generation:
    CLOG(WARNING, "mpc.cbit") << "Attempting to generate T_ref measurements";
    auto meas_result3 = GenerateReferenceMeas3(global_path_ptr, corridor_ptr, robot_pose, K,  DT, VF, curr_sid);
    auto measurements3 = meas_result3.measurements;
    bool point_stabilization3 = meas_result3.point_stabilization;
    std::vector<double> barrier_q_left = meas_result3.barrier_q_left;
    std::vector<double> barrier_q_right = meas_result3.barrier_q_right;
    // END of experimental code
    */



    // Calculate which T_ref measurements to used based on the current path solution
    CLOG(INFO, "mpc.cbit") << "Attempting to generate T_ref measurements";
    auto meas_result = GenerateReferenceMeas2(cbit_path_ptr, robot_pose, K,  DT, VF);
    auto measurements = meas_result.measurements;
    bool point_stabilization = meas_result.point_stabilization;

    std::vector<double> p_interp_vec = meas_result.p_interp_vec;
    std::vector<double> q_interp_vec = meas_result.q_interp_vec;

    // Experimental Synchronized Tracking/Teach Reference Poses:
    auto meas_result4 = GenerateReferenceMeas4(global_path_ptr, corridor_ptr, robot_pose, K,  DT, VF, curr_sid, p_interp_vec);
    auto measurements4 = meas_result4.measurements;
    bool point_stabilization4 = meas_result4.point_stabilization;
    std::vector<double> barrier_q_left = meas_result4.barrier_q_left;
    std::vector<double> barrier_q_right = meas_result4.barrier_q_right;
    //CLOG(ERROR, "mpc_debug.cbit") << "The New Reference Measurements are: " << measurements4;




    std::vector<lgmath::se3::Transformation> ref_pose_vec1;
    for (int i = 0; i<measurements.size(); i++)
    {
      ref_pose_vec1.push_back(measurements[i].inverse());
    }
    std::vector<lgmath::se3::Transformation> ref_pose_vec2;
    for (int i = 0; i<measurements4.size(); i++)
    {
      ref_pose_vec2.push_back(measurements4[i].inverse());
    }




    // Create and solve the STEAM optimization problem
    std::vector<lgmath::se3::Transformation> mpc_poses;
    try
    {
      CLOG(INFO, "mpc.cbit") << "Attempting to solve the MPC problem";
      // Solve using corridor mpc
      auto mpc_result = SolveMPC2(applied_vel, T0, measurements4, measurements, barrier_q_left, barrier_q_right, K, DT, VF, lat_noise_vect, pose_noise_vect, vel_noise_vect, accel_noise_vect, kin_noise_vect, point_stabilization, pose_error_weight, vel_error_weight, acc_error_weight, kin_error_weight, lat_error_weight);
      // Solve using tracking mpc
      //auto mpc_result = SolveMPC2(applied_vel, T0, measurements, measurements, barrier_q_left, barrier_q_right, K, DT, VF, lat_noise_vect, pose_noise_vect, vel_noise_vect, accel_noise_vect, kin_noise_vect, point_stabilization3, pose_error_weight, acc_error_weight, kin_error_weight, lat_error_weight);
      //auto mpc_result = SolveMPC(applied_vel, T0, measurements, K, DT, VF, pose_noise_vect, vel_noise_vect, accel_noise_vect, kin_noise_vect, point_stabilization); // Tracking controller version
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
    Eigen::Matrix<double, 2, 1> saturated_vel = SaturateVel2(applied_vel, config_->max_lin_vel, config_->max_ang_vel);
    CLOG(INFO, "mpc.cbit") << "The Saturated linear velocity is:  " << saturated_vel(0) << " The angular vel is: " << saturated_vel(1);
    
    // Store the result in memory so we can use previous state values to re-initialize and extrapolate the robot pose in subsequent iterations
    vel_history.erase(vel_history.begin());
    vel_history.push_back(saturated_vel);

    // Store the current robot state in the robot state path so it can be visualized
    robot_poses.push_back(T_w_p * T_p_r);

    // Send the robot poses and mpc prediction to rviz
    visualize(stamp, T_w_p, T_p_r, T_p_r_extp, T_p_r_extp2, mpc_poses, robot_poses, ref_pose_vec1, ref_pose_vec2);

    // return the computed velocity command for the first time step
    Command command;
    command.linear.x = saturated_vel(0) * config_->robot_linear_velocity_scale;
    command.angular.z = saturated_vel(1) * config_->robot_angular_velocity_scale;
    // Temporary modification by Jordy to test calibration of hte grizzly controller
    CLOG(DEBUG, "grizzly_controller_tests.cbit") << "Twist Linear Velocity: " << saturated_vel(0);
    CLOG(DEBUG, "grizzly_controller_tests.cbit") << "Twist Angular Velocity: " << saturated_vel(1);
    // End of modification
    
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

// Visualizing robot pose and the planned paths in rviz
void CBIT::visualize(const tactic::Timestamp& stamp, const tactic::EdgeTransform& T_w_p, const tactic::EdgeTransform& T_p_r, const tactic::EdgeTransform& T_p_r_extp, const tactic::EdgeTransform& T_p_r_extp_mpc, std::vector<lgmath::se3::Transformation> mpc_prediction, std::vector<lgmath::se3::Transformation> robot_prediction, std::vector<lgmath::se3::Transformation> ref_pose_vec1, std::vector<lgmath::se3::Transformation> ref_pose_vec2)
{
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

  // Publish robot pose extrapolated using odometry
  {
    Eigen::Affine3d T(T_p_r_extp.matrix());
    auto msg = tf2::eigenToTransform(T);
    msg.header.frame_id = "planning frame";
    msg.header.stamp = rclcpp::Time(stamp);
    msg.child_frame_id = "robot planning (extrapolated)";
    tf_bc_->sendTransform(msg);
  }

  // Publish MPC extrapolated current robot pose
  {
    Eigen::Affine3d T(T_p_r_extp_mpc.matrix());
    auto msg = tf2::eigenToTransform(T);
    msg.header.frame_id = "planning frame";
    msg.header.stamp = rclcpp::Time(stamp);
    msg.child_frame_id = "robot planning (extrapolated) mpc";
    tf_bc_->sendTransform(msg);
  }

  /// Publishing the MPC horizon prediction
  {
    nav_msgs::msg::Path mpc_path;
    mpc_path.header.frame_id = "world";
    mpc_path.header.stamp = rclcpp::Time(stamp);
    auto& poses = mpc_path.poses;

    // intermediate states
    for (unsigned i = 0; i < mpc_prediction.size(); ++i) {
      auto& pose = poses.emplace_back();
      pose.pose = tf2::toMsg(Eigen::Affine3d(mpc_prediction[i].matrix()));
    }
    mpc_path_pub_->publish(mpc_path);
  }

  /// Publishing the history of the robots actual pose
  {
    nav_msgs::msg::Path robot_path;
    robot_path.header.frame_id = "world";
    robot_path.header.stamp = rclcpp::Time(stamp);
    auto& poses = robot_path.poses;

    // intermediate states
    for (unsigned i = 0; i < robot_prediction.size(); ++i) {
      auto& pose = poses.emplace_back();
      pose.pose = tf2::toMsg(Eigen::Affine3d(robot_prediction[i].matrix()));
    }
    robot_path_pub_->publish(robot_path);
  }


  // Attempting to publish the actual path which we are receiving from the shared pointer in the cbitplanner
  // The path is stored as a vector of se3 Pose objects from cbit/utils, need to iterate through and construct proper ros2 nav_msgs PoseStamped

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
      //pose.pose = tf2::toMsg(Eigen::Affine3d(T_p_i_vec[i].matrix())); // Example for how to grab the transform from a transform with covariance data type
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



  // Attempting to Publish the left and right dynamic corridor for the current path homotopy class
  {
    nav_msgs::msg::Path corridor_left;
    corridor_left.header.frame_id = "world";
    corridor_left.header.stamp = rclcpp::Time(stamp);
    auto& poses_l = corridor_left.poses;

    nav_msgs::msg::Path corridor_right;
    corridor_right.header.frame_id = "world";
    corridor_right.header.stamp = rclcpp::Time(stamp);
    auto& poses_r = corridor_right.poses;

    // iterate through the corridor paths
    geometry_msgs::msg::Pose test_pose_l;
    geometry_msgs::msg::Pose test_pose_r;
    for (unsigned i = 0; i < corridor_ptr->x_left.size(); i++) 
    {
      //lhs
      auto& pose_l = poses_l.emplace_back();
      test_pose_l.position.x = corridor_ptr->x_left[i];
      test_pose_l.position.y = corridor_ptr->y_left[i];
      test_pose_l.position.z = 0.0; // setting this 0.0 for now for flat world assumption, but long term we might want to add a z component
      test_pose_l.orientation.x = 0.0;
      test_pose_l.orientation.y = 0.0;
      test_pose_l.orientation.z = 0.0;
      test_pose_l.orientation.w = 1.0;
      pose_l.pose = test_pose_l;

      // rhs
      auto& pose_r = poses_r.emplace_back();
      test_pose_r.position.x = corridor_ptr->x_right[i];
      test_pose_r.position.y = corridor_ptr->y_right[i];
      test_pose_r.position.z = 0.0; // setting this 0.0 for now for flat world assumption, but long term we might want to add a z component
      test_pose_r.orientation.x = 0.0;
      test_pose_r.orientation.y = 0.0;
      test_pose_r.orientation.z = 0.0;
      test_pose_r.orientation.w = 1.0;
      pose_r.pose = test_pose_r;
    }

    corridor_pub_l_->publish(corridor_left);
    corridor_pub_r_->publish(corridor_right);
  }

  // Attempting to Publish the reference poses used in the mpc optimization as a pose array
  {
    // create a PoseArray message
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header.frame_id = "world";

    // fill the PoseArray with some sample poses
    for (int i = 0; i < ref_pose_vec1.size(); i++) {
        geometry_msgs::msg::Pose pose;
        auto T1 = ref_pose_vec1[i].matrix();
        pose.position.x = T1(0,3);
        pose.position.y = T1(1,3);;
        pose.position.z = T1(2,3);;
        pose.orientation.w = 1.0;
        pose_array_msg.poses.push_back(pose);
    }
    ref_pose_pub1_->publish(pose_array_msg);
  }

    // Attempting to Publish the reference poses used in the mpc optimization as a pose array
  {
    // create a PoseArray message
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header.frame_id = "world";

    // fill the PoseArray with some sample poses
    for (int i = 0; i < ref_pose_vec2.size(); i++) {
        geometry_msgs::msg::Pose pose;
        auto T2 = ref_pose_vec2[i].matrix();
        pose.position.x = T2(0,3);
        pose.position.y = T2(1,3);;
        pose.position.z = T2(2,3);;
        pose.orientation.w = 1.0;
        pose_array_msg.poses.push_back(pose);
    }
    ref_pose_pub2_->publish(pose_array_msg);
  }
  return;
}


// Function for converting a p,q coordinate value into a euclidean coordinate using the pre-processed path to follow
Node CBIT::curve_to_euclid(Node node)
{
  double p_val = node.p;
  double q_val = node.q;
  int p_ind =  bisection(global_path_ptr->p, p_val);

  // Linearly interpolate a Euclidean Pose using the euclidean path and the relative p_val,q_val
  // TODO: need to use steam or lgmath se(3) classes for these poses, for now just using a vector
  Pose pose_c = lin_interpolate(p_ind, p_val);

  double x_i = pose_c.x - sin(pose_c.yaw)*q_val;
  double y_i = pose_c.y + cos(pose_c.yaw)*q_val;

  // Experimental, also interpolate the z value
  double z_i = pose_c.z; //+ cos(pose_c.yaw)*q_val; // I think here we just want to set the z to whatever the pose_c.z value

  return Node(x_i,y_i,z_i);
}

Pose CBIT::lin_interpolate(int p_ind, double p_val)
{
  double p_max = global_path_ptr->p[(global_path_ptr->p.size() - 1)]; //TODO: Replace this with se(3)
  double p_lower;
  double p_upper;
  if (p_val >= p_max) // if p_val is exactly the max (goal p) then return the final euclid pose
  {
    return Pose(global_path_ptr->path[(global_path_ptr->path.size() - 1)]);
  }

  else
  {
    p_upper = global_path_ptr->p[p_ind + 1];
    p_lower = global_path_ptr->p[p_ind];
  }

  Pose start_pose = global_path_ptr->path[p_ind];
  Pose end_pose = global_path_ptr->path[p_ind + 1];

  double x_c = start_pose.x + ((p_val - p_lower) / (p_upper - p_lower)) * (end_pose.x - start_pose.x);
  double y_c = start_pose.y + ((p_val - p_lower) / (p_upper - p_lower)) * (end_pose.y - start_pose.y);
  double z_c = start_pose.z + ((p_val - p_lower) / (p_upper - p_lower)) * (end_pose.z - start_pose.z);

  // For angles, we dont really care about roll and pitch, these can be left 0 (atleast for now)
  // For yaw need to be very careful of angle wrap around problem:
  double angle_difference = std::fmod((std::fmod((end_pose.yaw - start_pose.yaw),(2.0*M_PI)) + (3.0*M_PI)),(2.0*M_PI)) - M_PI; // CHECK THIS!
  double yaw_c = start_pose.yaw + ((p_val - p_lower) / (p_upper - p_lower)) * angle_difference;

  return Pose({x_c, y_c, z_c, 0.0, 0.0, yaw_c});

}


struct CBIT::collision_result CBIT::discrete_collision_v2(double discretization, Node start, Node end)
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

        if (costmap_col_tight(euclid_pt))
        {
          return {true, curv_pt};
        }
        

    }

    return {false, curv_pt};
}


// This collision check is only used at the end of each batch and determines whether the path should be rewired using the bare minimum obstacle distance
// Under normal operation we plan paths around a slightly more conservative buffer around each obstacle (equal to influence dist + min dist)
bool CBIT::costmap_col_tight(Node node)
{
  //CLOG(DEBUG, "path_planning.cbit_planner") << "Original Node: x: " << node.p << " y: " << node.q << " z: " << node.z;
  Eigen::Matrix<double, 4, 1> test_pt({node.p, node.q, node.z, 1});

  // I am no longer temporally filtering here, instead this takes place in the costmap itself in the change detection module
  // This means the costmap and transform size should never be more than 1
  //for (int i = 0; i < cbit_costmap_ptr->T_c_w_vect.size(); i++)
  //{


  auto collision_pt = costmap_ptr->T_c_w * test_pt;

  // Round the collision point x and y values down to the nearest grid resolution so that it can be found in the obstacle unordered_map
  float x_key = floor(collision_pt[0] / costmap_ptr->grid_resolution) * costmap_ptr->grid_resolution;
  float y_key = floor(collision_pt[1] / costmap_ptr->grid_resolution) * costmap_ptr->grid_resolution;

  //CLOG(DEBUG, "path_planning.cbit_planner") << "X_key:  " << x_key;
  //CLOG(DEBUG, "path_planning.cbit_planner") << "Y_key:  " << y_key;

  float grid_value;

  // Check to see if the point is in the obstacle map
  // We need to use a try/catch in this metod as if the key value pair doesnt exist (it usually wont) we catch an out of range error
  try 
  {
  // Block of code to try
    grid_value = costmap_ptr->obs_map.at(std::pair<float, float> (x_key, y_key));
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

}  // namespace path_planning
}  // namespace vtr