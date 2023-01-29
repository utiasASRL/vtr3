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
  }

  CLOG(INFO, "path_planning.cbit") << "Trying to create global path";
  // Create the path class object (Path preprocessing)
  CBITPath global_path(cbit_config, euclid_path_vec);

  // Make a pointer to this path
  std::shared_ptr<CBITPath> global_path_ptr = std::make_shared<CBITPath>(global_path);

  CLOG(INFO, "path_planning.cbit") << "Teach Path has been pre-processed. Attempting to instantiate the planner";


  // Instantiate the planner
  CBITPlanner cbit(cbit_config, global_path_ptr, robot_state, cbit_path_ptr, costmap_ptr);

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
    CLOG(INFO, "mpc.cbit") << "Attempting to solve the MPC problem";
    auto mpc_result = SolveMPC(applied_vel, T0, measurements, K, DT, VF, pose_noise_vect, vel_noise_vect, accel_noise_vect, kin_noise_vect, point_stabilization);
    applied_vel = mpc_result.applied_vel; // note dont re-declare applied vel here
    auto mpc_poses = mpc_result.mpc_poses;
    CLOG(INFO, "mpc.cbit") << "Successfully solved MPC problem";

    CLOG(INFO, "mpc.cbit") << "The linear velocity is:  " << applied_vel(0) << " The angular vel is: " << applied_vel(1);


    // If required, saturate the output velocity commands based on the configuration limits
    CLOG(INFO, "mpc.cbit") << "Saturating the velocity command if required";
    Eigen::Matrix<double, 2, 1> saturated_vel = SaturateVel(applied_vel, config_->max_lin_vel, config_->max_ang_vel);

    // Store the result in memory so we can use previous state values to re-initialize and extrapolate the robot pose in subsequent iterations
    vel_history.erase(vel_history.begin());
    vel_history.push_back(saturated_vel);

    // Store the current robot state in the robot state path so it can be visualized
    robot_poses.push_back(T0);

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
void CBIT::visualize(const tactic::Timestamp& stamp, const tactic::EdgeTransform& T_w_p, const tactic::EdgeTransform& T_p_r, const tactic::EdgeTransform& T_p_r_extp, const tactic::EdgeTransform& T_p_r_extp_mpc, std::vector<lgmath::se3::Transformation> mpc_prediction, std::vector<lgmath::se3::Transformation> robot_prediction)
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
  return;
}

}  // namespace path_planning
}  // namespace vtr