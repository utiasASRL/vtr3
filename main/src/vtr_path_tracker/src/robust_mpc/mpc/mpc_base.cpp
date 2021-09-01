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
 * \file mpc_base.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_path_tracker/robust_mpc/mpc/mpc_base.hpp>
#include <vtr_path_tracker/robust_mpc/mpc/mpc_types.hpp>

namespace vtr {
namespace path_tracker {

std::shared_ptr<Base> PathTrackerMPC::Create(
    const std::shared_ptr<Graph> graph,
    const std::shared_ptr<rclcpp::Node> node) {
  return std::make_shared<PathTrackerMPC>(graph, node, "path_tracker");
}

PathTrackerMPC::PathTrackerMPC(const std::shared_ptr<Graph> &graph,
                               const std::shared_ptr<rclcpp::Node> &node,
                               const std::string &param_prefix)
    : Base(graph, node, param_prefix),
      rc_experience_management_(graph, *node->get_clock()) {
  path_ = std::make_shared<MpcPath>(node_, param_prefix_);

  // Load the parameters for speed scheduling, optimization, and MPC.
  // Note: VTR2 had these in loadConfigs()
  CLOG(INFO, "path_tracker")
      << "Fetching MPC, MPC path and solver configuration parameters";
  // path configuration
  path_->loadPathParams();
  path_->loadGainScheduleConfigFile();
  // solver and MPC configuration
  loadSolverParams();
  loadMpcParams();
}

void PathTrackerMPC::loadSolverParams() {
  MpcSolverXUopt::opt_params_t opt_params;
  // clang-format off
  opt_params.weight_lat = node_->declare_parameter<float>(param_prefix_ + ".weight_lateral_error_mpc", 5.0);
  opt_params.weight_head = node_->declare_parameter<float>(param_prefix_ + ".weight_heading_error_mpc", 10.0);
  opt_params.weight_lat_final = node_->declare_parameter<float>(param_prefix_ + ".weight_lateral_error_final_mpc", 0.0);
  opt_params.weight_head_final = node_->declare_parameter<float>(param_prefix_ + ".weight_heading_error_final_mpc", 0.0);
  opt_params.weight_u = node_->declare_parameter<float>(param_prefix_ + ".weight_control_input_mpc", 3.0);
  opt_params.weight_v = node_->declare_parameter<float>(param_prefix_ + ".weight_speed_input_mpc", 10.0);
  opt_params.weight_du = node_->declare_parameter<float>(param_prefix_ + ".weight_control_input_derivative_mpc", 10.0);
  opt_params.weight_dv = node_->declare_parameter<float>(param_prefix_ + ".weight_speed_input_derivative_mpc", 50.0);
  opt_params.barrier_norm = node_->declare_parameter<float>(param_prefix_ + ".weight_barrier_norm_mpc", 0.3);
  opt_params.flg_en_mpcConstraints = node_->declare_parameter<bool>(param_prefix_ + ".enable_constrained_mpc", false);
  opt_params.flg_en_robustMpcConstraints = node_->declare_parameter<bool>(param_prefix_ + ".enable_robust_constrained_mpc", false);
  opt_params.w_max = node_->get_parameter(param_prefix_ + ".max_allowable_angular_speed").as_double();
  // clang-format on

  solver_.set_sizes(3, 1, 1);
  solver_.set_weights(opt_params);

  std::stringstream str_out;
  if (opt_params.flg_en_robustMpcConstraints &&
      opt_params.flg_en_mpcConstraints) {
    str_out << "ROBUST CONSTRAINED";
  } else if (opt_params.flg_en_mpcConstraints) {
    str_out << "CONSTRAINED";
  } else {
    str_out << "STANDARD";
  }
  str_out << " Optimization selected.";
  CLOG(INFO, "path_tracker") << str_out.str();
}

void PathTrackerMPC::loadMpcParams() {
  // clang-format off
  /// \note some of these params will already have been declared in loadPathParams() so we "get" instead
  // Controller flags
  mpc_params_.flg_en_time_delay_compensation = node_->declare_parameter<bool>(param_prefix_ + ".enable_time_delay_compensation", false);
  mpc_params_.flg_allow_ctrl_tos = node_->get_parameter(param_prefix_ + ".enable_turn_on_spot").as_bool();
  mpc_params_.flg_allow_ctrl_to_end = node_->declare_parameter<bool>(param_prefix_ + ".enable_ctrlToEnd", false);
  mpc_params_.flg_allow_ctrl_to_dir_sw = node_->declare_parameter<bool>(param_prefix_ + ".enable_ctrlToDirSw", false);
  mpc_params_.flg_use_steam_velocity = node_->declare_parameter<bool>(param_prefix_ + ".use_steam_velocity", false);
  mpc_params_.flg_use_vtr2_covariance = node_->declare_parameter<bool>(param_prefix_ + ".use_cov_from_vtr2", false);
  mpc_params_.flg_enable_fudge_block = node_->declare_parameter<bool>(param_prefix_ + ".enable_fudge_block", false);
  mpc_params_.flg_enable_varied_pred_step = node_->declare_parameter<bool>(param_prefix_ + ".enable_varied_pred_step", false);
  // Controller parameters
  mpc_params_.robust_control_sigma = node_->declare_parameter<double>(param_prefix_ + ".robust_control_sigma", 0.0);
  mpc_params_.default_xy_disturbance_uncertainty = node_->declare_parameter<double>(param_prefix_ + ".default_xy_disturbance_uncertainty", 0.035); // m
  mpc_params_.default_theta_disturbance_uncertainty = node_->declare_parameter<double>(param_prefix_ + ".default_theta_disturbance_uncertainty", 0.035); //rad
  mpc_params_.max_solver_iterations = node_->declare_parameter<int>(param_prefix_ + ".max_solver_iterations", 30);
  mpc_params_.max_lookahead = node_->declare_parameter<int>(param_prefix_ + ".count_mpc_size", 5);
  mpc_params_.init_step_size = node_->declare_parameter<double>(param_prefix_ + ".init_step_size", NAN);
  mpc_params_.local_path_poses_forward = node_->declare_parameter<int>(param_prefix_ + ".local_path_poses_forward", 25);
  mpc_params_.local_path_poses_back = node_->declare_parameter<int>(param_prefix_ + ".local_path_poses_back", 15);
  mpc_params_.look_ahead_step_ms = node_->declare_parameter<double>(param_prefix_ + ".look_ahead_step_ms", 150.0);
  mpc_params_.control_delay_ms = node_->declare_parameter<double>(param_prefix_ + ".control_delay_ms", 100.0);
  // Artificial disturbance parameters
  mpc_params_.Kv_artificial = node_->declare_parameter<double>(param_prefix_ + ".artificial_disturbance_Kv", 1.0);
  mpc_params_.Kw_artificial = node_->declare_parameter<double>(param_prefix_ + ".artificial_disturbance_Kw", 1.0);
  mpc_params_.num_poses_end_check = (unsigned)node_->declare_parameter<int>(param_prefix_ + ".num_poses_end_check", 3);
  // Trajectory timeout (previously in vtr_navigation)
  mpc_params_.extrapolate_timeout = node_->declare_parameter<double>(param_prefix_ + ".extrapolate_timeout", 1.5);

  CLOG(DEBUG, "path_tracker") << "Loaded MPC Parameters: ";
  CLOG(DEBUG, "path_tracker") << "flg_en_timeDelayCompensation " << mpc_params_.flg_en_time_delay_compensation;
  CLOG(DEBUG, "path_tracker") << "robust_control_sigma " << mpc_params_.robust_control_sigma;
  CLOG(DEBUG, "path_tracker") << "default_xy_disturbance_uncertainty " << mpc_params_.default_xy_disturbance_uncertainty;
  CLOG(DEBUG, "path_tracker") << "default_theta_disturbance_uncertainty " << mpc_params_.default_theta_disturbance_uncertainty;
  CLOG(DEBUG, "path_tracker") << "max_solver_iterations " << mpc_params_.max_solver_iterations;
  CLOG(DEBUG, "path_tracker") << "max_lookahead " << mpc_params_.max_lookahead;
  CLOG(DEBUG, "path_tracker") << "init_step_size " << mpc_params_.init_step_size;
  // clang-format on
}

void PathTrackerMPC::notifyNewLeaf(const Chain::ConstPtr &chain,
                                   const Stamp leaf_stamp, const Vid live_vid) {
  vision_pose_.updateLeaf(chain, leaf_stamp, live_vid);
}

void PathTrackerMPC::notifyNewLeaf(
    const Chain::ConstPtr &chain,
    const steam::se3::SteamTrajInterface &trajectory, const Vid live_vid,
    const uint64_t image_stamp) {
  const double time_now_ns = node_->now().nanoseconds();
  const bool trajectory_timed_out =
      (time_now_ns - image_stamp) > mpc_params_.extrapolate_timeout * 1e9;
  if (trajectory_timed_out) {
    CLOG(WARNING, "path_tracker") << "The trajectory timed out after "
                                  << (time_now_ns - image_stamp) * 1e-9
                                  << " s while updating the path tracker.";

    vision_pose_.updateLeaf(chain, vtr::common::timing::toChrono(image_stamp),
                            live_vid);
  } else {
    vision_pose_.updateLeaf(chain, trajectory, live_vid, image_stamp);
  }
}

Command PathTrackerMPC::controlStep() {
  if (!vision_pose_.isUpdated()) {
    CLOG_EVERY_N(30, WARNING, "path_tracker")
        << "Controller hasn't received a pose update yet. Commanding vehicle "
           "to stop.";
    setLatestCommand(0., 0.);
    return latest_command_;
  }

  // Extrapolate the pose to the time the control is published.
  CLOG(DEBUG, "path_tracker") << "=> Updating vision pose";
  if (mpc_params_.flg_en_time_delay_compensation) {
    /// \note assuming use_fixed_ctrl_rate
    common::timing::milliseconds time_to_control(
        static_cast<long>(control_period_ms_ + mpc_params_.control_delay_ms -
                          step_timer_.elapsedMs()));
    Stamp ctrl_time = common::timing::clock::now() + time_to_control;
    vision_pose_.updateFixedPose(ctrl_time);
  } else {
    vision_pose_.updateFixedPose(vision_pose_.voLeafStamp());
  }

  // Check the state and reset optimization hot-start if previously paused
  if (resetIfPreviouslyPaused()) {
    time_delay_comp2_.clear_hist();
    CLOG(INFO, "path_tracker") << "Path tracker re-starting from pause";
  }

  // check if path is complete
  if (checkPathComplete()) {
    CLOG(INFO, "path_tracker")
        << "Path tracker has reached the end of the path. Stopping vehicle.";
    setLatestCommand(0., 0.);
    publisher_->publish(latest_command_.twist);
    state_ = State::STOP;
    return latest_command_;
  }

  // Update time-delay compensation
  /// \todo: (old) Make sure this is safe for the first time-step before
  /// experience_management is properly initialized with measurements
  CLOG(DEBUG, "path_tracker") << "=> Updating experience management";
  rclcpp::Duration transform_delta_t =
      common::timing::toRosTime(vision_pose_.leafStamp()) -
      rc_experience_management_.experience_k_.transform_time;
  if (transform_delta_t.seconds() > 0.01) {
    // New localization received
    rclcpp::Time t_1 =
        rc_experience_management_.experience_km1_
            .transform_time;  // todo (Ben) this time doesn't seem to update, is
                              // it supposed to?
    rclcpp::Time t_2 = common::timing::toRosTime(vision_pose_.leafStamp());
    float v_cmd_avg, w_cmd_avg;
    time_delay_comp2_.get_avg_cmd(t_1, t_2, v_cmd_avg, w_cmd_avg,
                                  *node_->get_clock());

    rc_experience_management_.experience_km1_.x_k.command_k[0] = v_cmd_avg;
    rc_experience_management_.experience_km1_.x_k.command_k[1] = w_cmd_avg;

    time_delay_comp2_.del_hist_older_than(t_1);
  }

  // Project the path (locally) to 2D
  CLOG(DEBUG, "path_tracker") << "=> Construct local path (2D)";
  local_path_t local_path;
  int num_tos_poses_ahead;  // the number of TURN_ON_SPOT vertices coming up.
                            // (only count for one TOS maneuver)
  unsigned radius_forwards = 7, radius_backwards = 0;
  locateNearestPose(local_path,
                    std::max(closest_sid_guess_, vision_pose_.trunkSeqId()),
                    radius_forwards, radius_backwards);
  closest_sid_guess_ = local_path.current_pose_num;

  flattenDesiredPathAndGet2DRobotPose(local_path, num_tos_poses_ahead);

  // Set the current gain schedule
  CLOG(DEBUG, "path_tracker")
      << "=> Switch gain schedule to pose num: " << local_path.current_pose_num;
  path_->setCurrentGainSchedule(local_path.current_pose_num);

  // check control mode - feedback linearization or MPC?
  int pose_n = local_path.current_pose_num;
  bool use_dir_sw_ctrl = checkDirSw(pose_n);
  bool use_tos_ctrl = checkTOS(pose_n);
  bool use_end_ctrl = checkEndCtrl(pose_n);
  CLOG(DEBUG, "path_tracker") << "=> Checking control mode - "
                              << "direction switch: " << use_dir_sw_ctrl
                              << ", turn on spot: " << use_tos_ctrl
                              << ", end control: " << use_end_ctrl;

  // compute control
  bool flg_mpc_valid = false;
  float linear_speed_cmd = 0;
  float angular_speed_cmd = 0;

  // Use the feedback controller if required by the control mode
  // (TURN_ON_SPOT/DIR_SW/DIR_SW_REGION/END). Otherwise, use MPC.
  if (use_tos_ctrl || use_end_ctrl || use_dir_sw_ctrl) {
    CLOG(DEBUG, "path_tracker")
        << "=> Compute command from feedback linearization";
    computeCommandFdbk(linear_speed_cmd, angular_speed_cmd, use_tos_ctrl,
                       use_end_ctrl, use_dir_sw_ctrl,
                       path_->current_gain_schedule_, local_path,
                       num_tos_poses_ahead);
  } else {
    CLOG(DEBUG, "path_tracker") << "=> Compute command from mpc";
    /////////////////////////////////////////////////////////////////////////////
    // COMPUTE PARTS OF THE EXPERIENCE THAT ARE RELATED TO INFORMATION AVAILABLE
    // BEFORE THE CONTROL INPUT IS COMPUTED
    // Compute the latest experience
    // clang-format off
    rc_experience_management_.experience_k_.x_k.x_k = local_path.x_act;
    rc_experience_management_.experience_k_.T_0_v = local_path.T_0_v;
    rc_experience_management_.experience_k_.at_vertex_id = local_path.current_vertex_id;
    rc_experience_management_.experience_k_.to_vertex_id = local_path.next_vertex_id;
    rc_experience_management_.experience_k_.transform_time = common::timing::toRosTime(vision_pose_.leafStamp());
    rc_experience_management_.experience_k_.x_k.command_km1 = rc_experience_management_.experience_km1_.x_k.command_k;
    rc_experience_management_.experience_k_.path_curvature = 0.;
    rc_experience_management_.experience_k_.dist_from_vertex = local_path.x_act[0];
    rc_experience_management_.experience_k_.x_k.g_a_k.setZero();
    rc_experience_management_.experience_k_.x_k.dist_along_path_k = path_->dist_from_start_[local_path.current_pose_num];
    // clang-format on
    // Local error
    CLOG(DEBUG, "path_tracker") << "==> Compute local errors";
    /// \note: yuchen: tos_look_ahead_poses uninitialized, this function only
    /// needs current error not look ahead error, so this function call may not
    /// be needed at all.
    float heading_err, look_ahead_heading_err, lateral_err, longitudinal_err,
        look_ahead_long_err;
    int tos_look_ahead_poses;
    getLocalPathErrors(local_path, heading_err, look_ahead_heading_err,
                       lateral_err, longitudinal_err, look_ahead_long_err,
                       tos_look_ahead_poses);
    // clang-format off
    rc_experience_management_.experience_k_.x_k.tracking_error_k(0) = longitudinal_err;
    rc_experience_management_.experience_k_.x_k.tracking_error_k(1) = lateral_err;
    rc_experience_management_.experience_k_.x_k.tracking_error_k(2) = heading_err;
    // clang-format on

    // quantities related to velocity that need to be computed using
    // measurements from multiple time-steps
    MpcNominalModel nominal_model;
    if (mpc_params_.flg_use_steam_velocity) {
      // NOTE: Could update the velocity for experience_k, but that would be a
      // mess so doing it like this for now.
      // clang-format off
      rc_experience_management_.experience_km1_.velocity_k(0) = vision_pose_.velocity()(0);
      rc_experience_management_.experience_km1_.velocity_k(1) = vision_pose_.velocity()(5);
      rc_experience_management_.experience_km1_.velocity_is_valid = true;
      rc_experience_management_.experience_k_.x_k.velocity_km1 = rc_experience_management_.experience_km1_.velocity_k;
      rc_experience_management_.experience_k_.full_velocity_k = vision_pose_.velocity();
      // clang-format on
      nominal_model.computeDisturbancesForExperienceKm2SteamVel(
          rc_experience_management_.experience_km2_,
          rc_experience_management_.experience_km1_);
    } else {
      nominal_model.computeVelocitiesForExperienceKm1(
          rc_experience_management_.experience_km2_,
          rc_experience_management_.experience_km1_,
          rc_experience_management_.experience_k_);
      nominal_model.computeDisturbancesForExperienceKm2(
          rc_experience_management_.experience_km2_,
          rc_experience_management_.experience_km1_);
    }

    // DONE COMPUTING PARTS OF THE EXPERIENCE THAT ARE RELATED TO NON-CONTROL
    // INFO
    /////////////////////////////////////////////////////////////////////////////
    CLOG(DEBUG, "path_tracker") << "==> Computing MPC command";
    flg_mpc_valid =
        computeCommandMPC(local_path, linear_speed_cmd, angular_speed_cmd,
                          use_tos_ctrl, use_end_ctrl, use_dir_sw_ctrl);

    float linearSpeed = path_->scheduled_speed_[local_path.current_pose_num];
    float angularSpeed = angular_speed_cmd;
    if (solver_.result_flgs.flg_x_opt_and_pred_dont_match) {
      solver_.result_flgs.num_failed_opt_results =
          solver_.result_flgs.num_failed_opt_results + 1;

      if (solver_.result_flgs.num_failed_opt_results < 2) {
        CLOG(WARNING, "path_tracker") << "Using scaled down cmd from time k-1.";
        flg_mpc_valid = false;
        linearSpeed = solver_.v_km1 * 0.9;
        angularSpeed = solver_.u_km1 * 0.9;
      } else {
        CLOG(WARNING, "path_tracker")
            << "Too many failed optimizations in a row.";
        flg_mpc_valid = false;
        linearSpeed = utils::getSign(linearSpeed) * 0.3;
        path_->current_gain_schedule_.lateral_error_gain = 0.4;
        path_->current_gain_schedule_.heading_error_gain = 0.8;
      }
    } else {
      solver_.result_flgs.num_failed_opt_results = 0;
      if (solver_.opt_params.flg_en_mpcConstraints) {
        if (solver_.result_flgs.flg_nominal_pose_grossly_fails_constraints) {
          // Reschedule the default FL controller to aggressively come back to
          // the path
          CLOG(WARNING, "path_tracker")
              << "Nominal pose grossly not meeting constraints.  Disregarding "
                 "computed MPC inputs.";
          flg_mpc_valid = false;
          linearSpeed = utils::getSign(linearSpeed) * 0.3;
          path_->current_gain_schedule_.lateral_error_gain = 0.3;
          path_->current_gain_schedule_.heading_error_gain = 0.8;

        } else if (solver_.result_flgs.flg_nominal_pose_fails_constraints) {
          CLOG(WARNING, "path_tracker")
              << "Nominal pose not meeting constraints.";
        } else if (solver_.result_flgs.flg_uncertain_pose_fails_constraints) {
          CLOG(WARNING, "path_tracker")
              << "Uncertain pose not meeting constraints.";
        }
      }
      if (flg_mpc_valid) {
        if (utils::getSign(linearSpeed * linear_speed_cmd) < 0) {
          flg_mpc_valid = false;
          CLOG(WARNING, "path_tracker")
              << "Solver requested direction switch when none was planned.";

        } else if (std::isnan(angular_speed_cmd) ||
                   std::isnan(linear_speed_cmd)) {
          flg_mpc_valid = false;
          CLOG(WARNING, "path_tracker") << "Solver returned NAN.";

        } else {
          linearSpeed = utils::getSign(linearSpeed) *
                        std::max(0.0f, std::min(std::abs(linearSpeed),
                                                std::abs(linear_speed_cmd)));
          // convert_model_trajectory_to_poses(StatusOutMsg_, XuSolver_.x_opt);
        }
      }
    }
    // over-write commands with post-processed commands.
    linear_speed_cmd = linearSpeed;
    angular_speed_cmd = angularSpeed;

    if (!flg_mpc_valid) {
      // computeCommandMPC can change these flags if they are true within the
      // MPC look ahead window
      if (!(use_tos_ctrl || use_end_ctrl || use_dir_sw_ctrl)) {
        CLOG(WARNING, "path_tracker")
            << "MPC computation returned an error! Using feedback linearized "
               "control instead. This may be okay if near end of path.";
      }
      CLOG(DEBUG, "path_tracker")
          << "==> MPC not valid, fallback to feedback linearization.";
      computeCommandFdbk(linear_speed_cmd, angular_speed_cmd, use_tos_ctrl,
                         use_end_ctrl, use_dir_sw_ctrl,
                         path_->current_gain_schedule_, local_path,
                         num_tos_poses_ahead);
    }
  }  // Done computing MPC command.

  // Do other conditioning on the control outputs.
  if (mpc_params_.flg_enable_fudge_block) {
    float d_t = 0.1;
    rateLimitOutputs(linear_speed_cmd, angular_speed_cmd, solver_.v_km1,
                     path_->params_, d_t);
    CLOG_N_TIMES(1, WARNING, "path_tracker")
        << "Path tracker fudge block enabled!";
  }

  // Saturation (If the angular speed is too large)
  // bool angularSpeedSaturated = false;
  if (angular_speed_cmd > path_->current_gain_schedule_.saturation_limit) {
    CLOG(WARNING, "path_tracker")
        << "Angular Speed Saturation.  Desired: " << angular_speed_cmd
        << " Allowed: " << path_->current_gain_schedule_.saturation_limit;
    angular_speed_cmd = path_->current_gain_schedule_.saturation_limit;
  }
  if (angular_speed_cmd < -path_->current_gain_schedule_.saturation_limit) {
    CLOG(WARNING, "path_tracker")
        << "Angular Speed Saturation.  Desired: " << angular_speed_cmd
        << " Allowed: " << -path_->current_gain_schedule_.saturation_limit;
    angular_speed_cmd = -path_->current_gain_schedule_.saturation_limit;
  }

  // Saturate if the velocity or acceleration is too high
  if (linear_speed_cmd > path_->params_.v_max) {
    CLOG(WARNING, "path_tracker")
        << "Linear Speed Saturation.  Desired: " << linear_speed_cmd
        << " Allowed: " << path_->params_.v_max;
    linear_speed_cmd = path_->params_.v_max;
  }
  if (linear_speed_cmd < -path_->params_.v_max) {
    CLOG(WARNING, "path_tracker")
        << "Linear Speed Saturation.  Desired: " << linear_speed_cmd
        << " Allowed: " << -path_->params_.v_max;
    linear_speed_cmd = -path_->params_.v_max;
  }

  // record input for time-delay compensation
  rclcpp::Time current_time = node_->now();
  time_delay_comp2_.add_hist_entry(linear_speed_cmd, angular_speed_cmd,
                                   current_time, *node_->get_clock());
  solver_.set_cmd_km1(angular_speed_cmd, linear_speed_cmd);

  CLOG(DEBUG, "path_tracker") << "=> Final linear speed: " << linear_speed_cmd
                              << ", angular speed: " << angular_speed_cmd;

  // set latest_command
  setLatestCommand(linear_speed_cmd, angular_speed_cmd);
  // Finished saving experience
  return latest_command_;
}

void PathTrackerMPC::loadConfigs() {
  CLOG(INFO, "path_tracker")
      << "Loading configuration parameters and pre-processing the path.";

  // Next, load the desired path way-points from the localization chain into the
  // path object
  CLOG(DEBUG, "path_tracker") << "[ChainLock Requested] loadConfigs";
  chain_->lock();
  CLOG(DEBUG, "path_tracker") << "[ChainLock Acquired] loadConfigs";
  path_->extractPathInformation(chain_);
  chain_->unlock();
  CLOG(DEBUG, "path_tracker") << "[ChainLock Released] loadConfigs";

  // Set the control mode and the desired speed at each vertex
  path_->getSpeedProfile();

  path_->printPath();

  // Set up old experience management.
  CLOG(INFO, "path_tracker") << "Setting up path tracker experience management";
  initializeExperienceManagement();

  // Initialize closest sequence id to the start of the path sequence
  closest_sid_guess_ = 0;

  CLOG(INFO, "path_tracker") << "Finished setup for path tracker.";
}

void PathTrackerMPC::initializeExperienceManagement() {
  uint64_t curr_vid = path_->vertexID(0);
  uint64_t next_vid = path_->vertexID(1);
  MpcNominalModel nominal_model;

  // Set up RCExperienceManagement
  rc_experience_management_.initialize_running_experiences(
      nominal_model, curr_vid, next_vid, path_->turn_radius_[0]);
}

void PathTrackerMPC::publishCommand(Command &command) {
  command.twist.linear.x *= mpc_params_.Kv_artificial;
  command.twist.angular.z *= mpc_params_.Kw_artificial;
  Base::publishCommand(command);
}

void PathTrackerMPC::finishControlLoop() {
  // Send a stop command to the vehicle.
  CLOG(INFO, "path_tracker")
      << "Path tracker finished controlLoop; stopping the vehicle";
  auto command = Command();
  publishCommand(command);
}

bool PathTrackerMPC::checkDirSw(const int pose_n) {
  return (
      (path_->scheduled_ctrl_mode_[pose_n] == VertexCtrlType::DIR_SW_POSE or
       path_->scheduled_ctrl_mode_[pose_n] == VertexCtrlType::DIR_SW_REGION) and
      mpc_params_.flg_allow_ctrl_to_dir_sw);
}

bool PathTrackerMPC::checkTOS(const int pose_n) {
  return (path_->scheduled_ctrl_mode_[pose_n] ==
          VertexCtrlType::TURN_ON_SPOT) and
         mpc_params_.flg_allow_ctrl_tos;
}

bool PathTrackerMPC::checkEndCtrl(const int pose_n) {
  return (path_->scheduled_ctrl_mode_[pose_n] == VertexCtrlType::END) and
         mpc_params_.flg_allow_ctrl_to_end;
}

void PathTrackerMPC::setLatestCommand(const double linear_speed_cmd,
                                      const double angular_speed_cmd) {
  latest_command_.header.stamp = node_->now();
  latest_command_.twist.linear.x = linear_speed_cmd;
  latest_command_.twist.angular.z = angular_speed_cmd;
}

bool PathTrackerMPC::resetIfPreviouslyPaused() {
  if (state_ == State::PAUSE) {
    previously_paused_ = true;
    return false;

  } else if ((state_ == State::RUN) && previously_paused_) {
    CLOG(WARNING, "path_tracker")
        << "Path tracker resuming the current path from PAUSE";

    // Re-set and taper up the speed profile.
    path_->scheduled_speed_ = path_->original_scheduled_speed_;
    int start_region = vision_pose_.trunkSeqId();
    path_->adjustSpeedProfileHoldSpeed(
        start_region,
        path_->params_.reset_from_pause_slow_speed_zone_length_vertices,
        path_->params_.reset_from_pause_slow_speed);
    CLOG(INFO, "path_tracker")
        << "Tapering the speed profile up to resume from a PAUSE";

    previously_paused_ = false;
    solver_.reset_cmd_km1();
    solver_.result_flgs.num_failed_opt_results = 0;
    return true;
  } else {
    return false;
  }
}

bool PathTrackerMPC::checkPathComplete() {
  // Check if the trunk is close to the end. Only then, check distance
  if (vision_pose_.trunkSeqId() >=
      path_->numPoses() - mpc_params_.num_poses_end_check) {
    // get the error to the end of the path
    double linear_distance;
    double angular_distance;
    getErrorToEnd(linear_distance, angular_distance);

    // Check against thresholds
    bool within_end_offset =
        (std::abs(linear_distance) <= path_->params_.path_end_x_thresh);
    bool within_end_rotation =
        (std::abs(angular_distance) <= path_->params_.path_end_heading_thresh);
    return (within_end_offset and within_end_rotation);
  } else {
    return false;
  }
}

void PathTrackerMPC::getErrorToEnd(double &linear_distance,
                                   double &angular_distance) {
  CLOG(DEBUG, "path_tracker") << "[ChainLock Requested] getErrorToEnd";
  chain_->lock();
  CLOG(DEBUG, "path_tracker") << "[ChainLock Acquired] getErrorToEnd";
  Transformation T_0_v = chain_->pose(vision_pose_.trunkSeqId()) *
                         vision_pose_.T_leaf_trunk().inverse();
  Transformation T_0_end = chain_->pose(path_->num_poses_ - 1);
  chain_->unlock();
  CLOG(DEBUG, "path_tracker") << "[ChainLock Released] getErrorToEnd";
  Eigen::Matrix<double, 6, 1> se3_end_v = (T_0_end.inverse() * T_0_v).vec();

  // linear_distance  = se3_end_v.head<3>().norm();  // this is the sqrt( ...^2)
  // angular_distance = se3_end_v.tail<3>().norm();
  linear_distance = se3_end_v[0];
  angular_distance = se3_end_v[5];
}

void PathTrackerMPC::flattenDesiredPathAndGet2DRobotPose(
    local_path_t &local_path, int &tos_look_ahead_pose) {
  int state_size = 3;
  int num_poses = path_->numPoses();
  int poses_fwd = std::min(mpc_params_.local_path_poses_forward,
                           num_poses - (int)local_path.current_pose_num - 1);
  int poses_back = std::min(mpc_params_.local_path_poses_back,
                            (int)local_path.current_pose_num);

  // Initialize fields of local_path to the correct size
  local_path.x_des_fwd = Eigen::MatrixXf::Zero(state_size, poses_fwd + 1);
  local_path.x_ub = Eigen::MatrixXf::Zero(2, poses_fwd + 1);
  local_path.x_lb = Eigen::MatrixXf::Zero(2, poses_fwd + 1);
  local_path.x_des_bck = Eigen::MatrixXf::Zero(state_size, poses_back);

  // Initialize variables - note:  kpi = k plus i
  tf2::Vector3 p_0_k_0(0, 0, 0), p_0_kpi_0(0, 0, 0), p_k_kpi_0(0, 0, 0),
      p_k_kpi_k(0, 0, 0);
  tf2::Quaternion q_0_k_0(0, 0, 0, 0), q_0_kpi_0(0, 0, 0, 0);
  tf2::Vector3 x_hat(1, 0, 0);

  // Get the next path pose
  common::rosutils::getTfPoint(path_->poses_[local_path.current_pose_num],
                               p_0_k_0);
  common::rosutils::getTfQuaternion(path_->poses_[local_path.current_pose_num],
                                    q_0_k_0);
  tf2::Transform C_0_k(q_0_k_0);  // The transform from vertex K to the root.

  bool flg_counting_TOS = false;
  bool flg_done_counting = false;
  int poses_to_skip = 0;

  // Flatten the desired path to 2D (locally)
  for (int i = -poses_back; i < poses_fwd + 1; i++) {
    // Include one waypoint behind the current to make sure x_des_fwd always
    // brackets the robot
    int pose_i = std::max(0, int(local_path.current_pose_num) + i);

    // check if we need to re-size local_path.x_des_fwd
    if (pose_i == num_poses) {
      CLOG(WARNING, "path_tracker") << "Needed to re-size x_des_fwd. Make sure "
                                       "it is initialized correctly!";
      Eigen::MatrixXf x_des_fwd_temp = local_path.x_des_fwd;
      if (i > 0) {
        local_path.x_des_fwd = x_des_fwd_temp.block(0, 0, state_size, i);
      }
      break;
    }

    // Check if there are TURN_ON_SPOT vertices coming up. If so, keep track of
    // how many.
    if (i >= 0) {
      if (path_->scheduled_ctrl_mode_[pose_i] == VertexCtrlType::TURN_ON_SPOT &&
          !flg_done_counting) {
        flg_counting_TOS = true;
        poses_to_skip++;
      } else if (flg_counting_TOS && path_->scheduled_ctrl_mode_[pose_i] !=
                                         VertexCtrlType::TURN_ON_SPOT) {
        poses_to_skip++;
        flg_counting_TOS = false;
        flg_done_counting = true;
      }
    }

    common::rosutils::getTfPoint(path_->poses_[pose_i], p_0_kpi_0);
    common::rosutils::getTfQuaternion(path_->poses_[pose_i], q_0_kpi_0);

    // Find theta desired kpi
    tf2::Transform C_0_kpi(q_0_kpi_0);
    tf2::Vector3 vec_th_kpi = C_0_k.inverse() * C_0_kpi * x_hat;

    float th_des = atan2(vec_th_kpi.getY(), vec_th_kpi.getX());
    th_des = utils::thetaWrap(th_des);

    if (i < 0) {
      local_path.x_des_bck(2, i + poses_back) = th_des;
    } else {
      local_path.x_des_fwd(2, i) = th_des;
    }

    // Find x,y desired kpi
    p_k_kpi_0 = p_0_kpi_0 - p_0_k_0;
    p_k_kpi_k = C_0_k.inverse() * p_k_kpi_0;
    p_k_kpi_k.setZ(0.0);

    if (i < 0) {
      local_path.x_des_bck(0, i + poses_back) = p_k_kpi_k.getX();
      local_path.x_des_bck(1, i + poses_back) = p_k_kpi_k.getY();
    } else {
      local_path.x_des_fwd(0, i) = p_k_kpi_k.getX();
      local_path.x_des_fwd(1, i) = p_k_kpi_k.getY();
    }

    // Copy over the tracking limits (lateral/heading)
    if (i >= 0) {
      local_path.x_ub(0, i) = path_->poses_tol_positive_[pose_i];
      local_path.x_ub(1, i) = path_->poses_heading_constraint_pos_[pose_i];
      local_path.x_lb(0, i) = path_->poses_tol_negative_[pose_i];
      local_path.x_lb(1, i) = path_->poses_heading_constraint_neg_[pose_i];
    }
  }

  tos_look_ahead_pose =
      std::min((int)local_path.current_pose_num + poses_to_skip, num_poses - 1);

  // Get the current state (i.e. to be used in f(x_k, u_k))
  local_path.x_act = Eigen::VectorXf::Zero(state_size);

  // Note:  kpi = k plus i
  tf2::Vector3 p_v_k_v(0, 0, 0), p_k_v_k(0, 0, 0);

  // Transform the current robot pose
  // T_0_v is the transform from the vehicle frame to the root
  // p_0_k_0 is the position of frame k (the current frame) wrt the root
  //   expressesd in the root frame
  // p_v_k_v is the position of frame k wrt the vehicle expressed in the vehicle
  //   frame
  // -p_v_k_v is the position of the vehicle wrt frame k expressed in the
  //   vehicle frame
  // p_k_v_k is the position of the vehicle wrt frame k expressed
  //   in frame k
  p_v_k_v = local_path.T_0_v.inverse() * p_0_k_0;
  tf2::Transform C_0_v(local_path.T_0_v.getRotation());
  p_k_v_k = C_0_k.inverse() * C_0_v * (-p_v_k_v);
  tf2::Vector3 th_err_vec = C_0_v.inverse() * C_0_k * x_hat;
  float th_k = -atan2(th_err_vec.getY(), th_err_vec.getX());

  // get the uncertainty in the current pose.
  Eigen::Matrix<float, 6, 6> sigma_full;
  sigma_full =
      vision_pose_.T_leaf_trunk().cov().cast<float>();  // 6x6 uncertainty.
  // TODO: Make the size of the matrix configurable. Dynamic model
  // needs a state_size parameter.
  Eigen::MatrixXf sigma_act_xyth = Eigen::MatrixXf::Zero(3, 3);
  sigma_act_xyth.block<2, 2>(0, 0) =
      sigma_full.block<2, 2>(0, 0);  // x,y uncertainty
  sigma_act_xyth(2, 2) = sigma_full(5, 5);
  sigma_act_xyth.block<2, 1>(0, 2) = sigma_full.block<2, 1>(0, 5);
  sigma_act_xyth.block<1, 2>(2, 0) = sigma_full.block<1, 2>(5, 0);

  // Copy to local_path.x_act and x_act_cov
  local_path.x_act[0] = p_k_v_k.getX();
  local_path.x_act[1] = p_k_v_k.getY();
  local_path.x_act[2] = th_k;
  local_path.x_act_cov = sigma_act_xyth;

  CLOG(DEBUG, "path_tracker")
      << "[local_path] 2D projection result:" << std::endl
      << "x_des_bck:\n"
      << local_path.x_des_bck << std::endl
      << "x_des_fwd:\n"
      << local_path.x_des_fwd << std::endl
      << "x_ub:\n"
      << local_path.x_ub << std::endl
      << "x_lb:\n"
      << local_path.x_lb << std::endl
      << "x_act (p_k_vk): " << local_path.x_act.transpose();
}

void PathTrackerMPC::getLocalPathErrors(const local_path_t local_path,
                                        float &heading_error,
                                        float &look_ahead_heading_error,
                                        float &lateral_error,
                                        float &longitudinal_error,
                                        float &look_ahead_longitudinal_error,
                                        const int &tos_look_ahead_poses) {
  // set up some temporary convenience variables
  int current_pose_num = local_path.current_pose_num;
  auto x_act = local_path.x_act;
  auto x_des_fwd = local_path.x_des_fwd;
  int look_ahead_pose = 0;          // this will be set in getWindow.
  bool get_window_forwards = true;  // Option for getWindow function call

  // getWindow takes a distance and rotation then searches forwards through
  // the path to find the look-ahead pose which meets either the rotation
  // or distance requirement.  The index is returned in windowEnd.
  path_->getWindow(path_->dist_from_start_, path_->turn_angle_,
                   path_->current_gain_schedule_.look_ahead_distance,
                   path_->current_gain_schedule_.angular_look_ahead,
                   current_pose_num,  // start
                   look_ahead_pose,   // end
                   get_window_forwards);

  // compute the number of poses until we get to the end of the path or the end
  // of a turn on the spot
  int num_poses_ahead = (int)(look_ahead_pose - current_pose_num);
  int tos_num_poses_ahead = (int)(tos_look_ahead_poses - current_pose_num);

  tos_num_poses_ahead =
      std::min(tos_num_poses_ahead, (int)x_des_fwd.cols() - 1);
  num_poses_ahead = std::min(num_poses_ahead, (int)x_des_fwd.cols() - 1);

  // Compute tracking errors depending on whether or not the current vertex is a
  // turn on the spot
  if (path_->scheduled_ctrl_mode_[current_pose_num] ==
      VertexCtrlType::TURN_ON_SPOT) {
    look_ahead_heading_error = x_des_fwd(2, tos_num_poses_ahead) - x_act[2];
    look_ahead_longitudinal_error =
        x_des_fwd(0, tos_num_poses_ahead) - x_act[0];
  } else {
    look_ahead_heading_error = x_des_fwd(2, num_poses_ahead) - x_act[2];
    look_ahead_longitudinal_error = x_des_fwd(0, num_poses_ahead) - x_act[0];
  }

  // the coordinates of the vehicle are expressed in the desired frame, so the
  // error is -ve pose
  heading_error = -x_act[2];
  lateral_error = -x_act[1];
  longitudinal_error = -x_act[0];
}

void PathTrackerMPC::computeCommandFdbk(
    float &linear_speed_cmd, float &angular_speed_cmd, const bool use_tos_ctrl,
    const bool use_end_ctrl, const bool use_dir_sw_ctrl,
    GainSchedule &gain_schedule, const local_path_t local_path,
    const int num_tos_poses_ahead) {
  // get local path errors (current minus desired)
  float heading_error, look_ahead_heading_error, lateral_error,
      longitudinal_error, look_ahead_longitudinal_error;
  getLocalPathErrors(local_path, heading_error, look_ahead_heading_error,
                     lateral_error, longitudinal_error,
                     look_ahead_longitudinal_error, num_tos_poses_ahead);

  // Compute linear speed
  float target_linear_speed = gain_schedule.target_linear_speed;
  if (use_tos_ctrl) {
    // Turn on the spot
    CLOG(DEBUG, "path_tracker") << "Using turn on spot ctrl";
    if (fabs(gain_schedule.tos_x_error_gain * longitudinal_error) <
        fabs(target_linear_speed)) {
      target_linear_speed =
          gain_schedule.tos_x_error_gain * look_ahead_longitudinal_error;
    }
  } else if (use_end_ctrl) {
    // Control to end point
    CLOG(DEBUG, "path_tracker") << "Using end ctrl";
    double linear_distance, angular_distance;
    getErrorToEnd(linear_distance, angular_distance);
    double target_lin_speed_tmp =
        utils::getSign(gain_schedule.target_linear_speed) * 0.3;
    target_linear_speed = -1 * gain_schedule.end_x_error_gain * linear_distance;
    target_linear_speed = std::min((double)fabs(target_linear_speed),
                                   fabs(target_lin_speed_tmp)) *
                          utils::getSign(target_lin_speed_tmp);
  } else if (use_dir_sw_ctrl) {
    // Control to direction switch
    CLOG(DEBUG, "path_tracker") << "Using dir switch ctrl";
    /// \note used to use longitudinal error instead of look head longitudinal
    /// error
    if (fabs(gain_schedule.dir_sw_x_error_gain *
             look_ahead_longitudinal_error) < fabs(target_linear_speed)) {
      target_linear_speed =
          gain_schedule.dir_sw_x_error_gain * look_ahead_longitudinal_error;
    }
  } else {
    // Whenever MPC fails hard code linear speed to 0.3
    CLOG(WARNING, "path_tracker")
        << "Using feekback linearization, this very likely won't give the "
           "desired control behavior.";
    target_linear_speed =
        utils::getSign(gain_schedule.target_linear_speed) * 0.3;

    int pose_i = 0;
    MpcNominalModel NominalModel;
    while (NominalModel.robot_has_passed_desired_poseNew(
        path_->scheduled_speed_[vision_pose_.trunkSeqId()], local_path.x_act,
        local_path.x_des_fwd.block<3, 1>(0, pose_i))) {
      if (pose_i == (int)local_path.x_des_fwd.cols() - 1) break;
      pose_i++;
    }

    look_ahead_heading_error =
        local_path.x_des_fwd(2, pose_i) - local_path.x_act[2];
    lateral_error = local_path.x_des_fwd(1, pose_i) - local_path.x_act[1];
  }
  // Set the linear speed
  linear_speed_cmd = target_linear_speed;

  // Compute angular speed
  if (use_tos_ctrl) {
    // Turn on spot
    angular_speed_cmd = gain_schedule.tos_angular_speed *
                        utils::getSign(look_ahead_heading_error);
  } else if (use_end_ctrl) {
    // End of path
    angular_speed_cmd =
        gain_schedule.end_heading_error_gain * look_ahead_heading_error;
  } else if (use_dir_sw_ctrl) {
    // Direction switch
    angular_speed_cmd =
        gain_schedule.dir_sw_heading_error_gain * look_ahead_heading_error;
  } else {
    // Whenever MPC fails
    if (fabs(look_ahead_heading_error) >= (0.4 * M_PI)) {
      // If heading error ~ pi/2
      // Command saturated angular speed, rare. Set
      CLOG(WARNING, "path_tracker")
          << "Saturated commands. Using max allowed ctrl.";
      if (look_ahead_heading_error < 0)
        angular_speed_cmd = -gain_schedule.saturation_limit;
      else
        angular_speed_cmd = gain_schedule.saturation_limit;
    } else {
      const float k1 =
          gain_schedule.lateral_error_gain;  // changed when mpc fails
      const float el = lateral_error;
      const float k2 =
          gain_schedule.heading_error_gain;  // changed when mpc fails
      const float v = linear_speed_cmd;
      const float eh = look_ahead_heading_error;
      angular_speed_cmd = (k1 * el + k2 * v * sin(eh)) / (v * cos(eh));
    }
  }
}

bool PathTrackerMPC::computeCommandMPC(local_path_t &local_path, float &v_cmd,
                                       float &w_cmd, bool &use_tos_ctrl,
                                       bool &use_end_ctrl,
                                       bool &use_dir_sw_ctrl) {
  const int mpc_size = mpc_params_.max_lookahead;

  for (int opt_attempts = 0; opt_attempts < 2; opt_attempts++) {
    solver_.set_lookahead(mpc_size);
    local_path.x_des_interp = Eigen::MatrixXf::Zero(3, mpc_size + 1);
    local_path.x_lb_interp = Eigen::MatrixXf::Zero(2, mpc_size + 1);
    local_path.x_ub_interp = Eigen::MatrixXf::Zero(2, mpc_size + 1);
    local_path.x_lb_interp.block<2, 1>(0, 0) =
        local_path.x_lb.block<2, 1>(0, 0);
    local_path.x_ub_interp.block<2, 1>(0, 0) =
        local_path.x_ub.block<2, 1>(0, 0);

    // Reset solver
    solver_.reset_solver();

    // Prepare state sequences
    MpcNominalModel nominal_model;

    // Set x_pred and x_opt to zero, except the first element which contains the
    // current state.
    initializeModelTrajectory(mpc_size, nominal_model, solver_,
                              rc_experience_management_, local_path);
    MpcNominalModel::model_state_t *x_opt_curr_index;

    // Prepare time-delay compensation
    bool flg_time_delay_comp_possible = false;
    std::vector<float> v_cmd_vec, w_cmd_vec, dt_time_vec;
    unsigned time_delay_index = 0;
    if (mpc_params_.flg_en_time_delay_compensation) {
      long transform_time =
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              vision_pose_.leafStamp().time_since_epoch())
              .count();
      rclcpp::Time t_1(static_cast<double>(transform_time));
      rclcpp::Time t_2 = node_->now() + rclcpp::Duration(0.05 * 1.e9);

      time_delay_comp2_.get_cmd_list(t_1, t_2, v_cmd_vec, w_cmd_vec,
                                     dt_time_vec, *node_->get_clock());

      if (!v_cmd_vec.empty()) {
        flg_time_delay_comp_possible = true;
      }
    }

    /// \note assuming use_fixed_ctrl_rate
    float d_t = control_period_ms_ / 1000.;

    // Optimization outer loop
    for (int opt_iter = 0; opt_iter < mpc_params_.max_solver_iterations;
         ++opt_iter) {
      // Initialize indexers, updated at end of loop
      int pose_i = 0;
      int pose_im1 = 0;

      // At each iteration of the solver, predict the state sequence considering
      // x_0 and the current u_opt
      for (int pred_index = 0; pred_index < mpc_size; ++pred_index) {
        // Get pointer to variable containing information for
        // state/control/disturbance/etc at time "index1"
        x_opt_curr_index = solver_.select_x_pred(pred_index);

        // Get the desired velocity based on the a priori scheduled speed
        float v_des =
            path_->scheduled_speed_[local_path.current_pose_num + pose_im1];

        // Set the control input for time "index1"
        if (flg_time_delay_comp_possible) {
          // ... based on historic control inputs
          x_opt_curr_index->command_k[0] = v_cmd_vec[time_delay_index];
          x_opt_curr_index->command_k[1] = w_cmd_vec[time_delay_index];
          d_t = dt_time_vec[time_delay_index];
        } else {
          // Initialize with control sequence based on the current optimization
          // solution
          solver_.set_desired_speed(
              pred_index, v_des);  // set v_desired(???) and x_opt.command_k[0]
          solver_.get_desired_ctrl(pred_index);  // set x_opt.command_k[1]
          /// \note assuming use_fixed_ctrl_rate
          d_t = control_period_ms_ / 1000.;
        }

        // change the time-step to extend the look-ahead horizon if enabled
        // This should only happen for future time-steps. Not time-delay or the
        // first time-step.
        if (mpc_params_.flg_enable_varied_pred_step and pred_index > 0) {
          d_t = mpc_params_.look_ahead_step_ms / 1000.;
        }

        // Estimate disturbance for time "index1"
        nominal_model.set_disturbance_model_zero(*x_opt_curr_index);
        x_opt_curr_index->var_g_a_k = 0.001 * Eigen::MatrixXf::Identity(3, 3);
        x_opt_curr_index->var_g_a_k(0, 0) =
            std::pow(mpc_params_.default_xy_disturbance_uncertainty, 2);
        x_opt_curr_index->var_g_a_k(1, 1) =
            std::pow(mpc_params_.default_xy_disturbance_uncertainty, 2);
        x_opt_curr_index->var_g_a_k(2, 2) =
            std::pow(mpc_params_.default_theta_disturbance_uncertainty, 2);
        rotateDisturbanceIntoPoseNumFrame(*x_opt_curr_index);
        // Compute gradients (gdot), Hessians (Jdot_gdot) for time "index1", and
        // state for time "index1+1"
        nominal_model.get_gdot(*x_opt_curr_index,
                               d_t);  // set the jacobians for the optimiser
        nominal_model.get_Jdot_gdot(*x_opt_curr_index,
                                    d_t);  // set the hessians for the optimiser

        // set x_k, var_x_k, command_km1, velocity_km1, of the second argument
        nominal_model.f_x_unscentedUncertainty(
            *x_opt_curr_index, solver_.x_pred[pred_index + 1], d_t);

        // Post-process depending on what kind of solver you're using
        solver_.x_opt[pred_index + 1].var_x_k =
            solver_.x_pred[pred_index + 1].var_x_k;

        // Check if the state estimate has progressed past a desired pose
        // This check is very problematic for noisy paths, direction switches,
        // turn-on-spots
        while (nominal_model.robot_has_passed_desired_poseNew(
            v_des, solver_.x_opt[pred_index + 1].x_k,
            local_path.x_des_fwd.block<3, 1>(0, pose_i))) {
          pose_i++;
          pose_im1 = std::max(0, pose_i - 1);
          if (pose_i >= (int)local_path.x_des_fwd.cols()) break;
        }

        if (local_path.current_pose_num + pose_im1 >=
            (int)path_->scheduled_ctrl_mode_.size()) {
          /// \todo verify that this never happens, currently used to give
          /// informative error message in case.
          std::string err{"query pose exceeds size of the path"};
          CLOG(ERROR, "path_tracker") << err;
          throw std::runtime_error{err};
        }

        const auto &curr_ctrl_mode =
            path_->scheduled_ctrl_mode_[local_path.current_pose_num + pose_im1];
        if (curr_ctrl_mode == VertexCtrlType::END) {
          use_end_ctrl = true;
          CLOG(WARNING, "path_tracker") << "Look ahead pose enters END region";
          return false;
        } else if (curr_ctrl_mode == VertexCtrlType::TURN_ON_SPOT) {
          use_tos_ctrl = true;
          CLOG(WARNING, "path_tracker")
              << "Look ahead pose enters TURN_ON_SPOT region";
          return false;
        } else if (curr_ctrl_mode == VertexCtrlType::DIR_SW_REGION ||
                   curr_ctrl_mode == VertexCtrlType::DIR_SW_POSE) {
          use_dir_sw_ctrl = true;
          CLOG(WARNING, "path_tracker")
              << "Look ahead pose enters DIR_SW region or pose";
          return false;
        }

        if (pose_i >= (int)local_path.x_des_fwd.cols()) {
          CLOG(WARNING, "path_tracker")
              << "pose_i exceeds size of x_des_fwd, consider resetting path "
                 "window size";
          return false;
        }

        // Compute an interpolated desired pose
        // This interpolation is very problematic for noisy paths, direction
        // switches, turn-on-spots
        Eigen::MatrixXf temp_x_des_interp;
        nominal_model.computeInterpolatedDesiredPoseNew(
            local_path.x_des_fwd.block<3, 1>(0, pose_im1),
            local_path.x_des_fwd.block<3, 1>(0, pose_i),
            solver_.x_opt[pred_index + 1], temp_x_des_interp);
        local_path.x_des_interp.block<3, 1>(0, pred_index + 1) =
            temp_x_des_interp;
        local_path.x_lb_interp.block<2, 1>(0, pred_index + 1) =
            local_path.x_lb.block<2, 1>(0, pose_i);
        local_path.x_ub_interp.block<2, 1>(0, pred_index + 1) =
            local_path.x_ub.block<2, 1>(0, pose_i);

        nominal_model.computeDisturbanceDependancy(
            solver_.x_opt[pred_index + 1], solver_.x_opt[pred_index],
            local_path.x_des_interp.block<3, 1>(0, pred_index + 1),
            (float)
                path_->dist_from_start_[local_path.current_pose_num + pose_i],
            d_t);

        // Check if there are any more historic control inputs to process
        if (flg_time_delay_comp_possible) {
          *x_opt_curr_index = solver_.x_pred[pred_index + 1];
          x_opt_curr_index->dist_along_path_k =
              path_->dist_from_start_[local_path.current_pose_num +
                                      pose_i];  // Approx
          pred_index = pred_index - 1;

          time_delay_index = time_delay_index + 1;
          if (time_delay_index >= v_cmd_vec.size()) {
            flg_time_delay_comp_possible = false;
          }
        }
      }  // Done computing nominal prediction (incl. first and second
         // derivatives)

      // Generate worst-case scenarios given the predicted state and uncertainty
      if (solver_.opt_params.flg_en_robustMpcConstraints) {
        nominal_model.generateWorstCaseTrajectories(
            solver_.x_opt, mpc_params_.robust_control_sigma);
      }
      // Given the prediction (desired states, predicted
      // mean/uncertainty/gradients/hessians), compute updated control input
      // sequence
      solver_.compute_solver_update(local_path, opt_iter);
    }

    Eigen::MatrixXf u_vec, v_vec;
    solver_.get_u(u_vec);
    solver_.get_v(v_vec);

    w_cmd = (float)u_vec(0, 0);
    v_cmd = (float)v_vec(0, 0);

    CLOG(DEBUG, "path_tracker")
        << "Computed command in MPC compute loop - v: " << v_cmd
        << ", w: " << w_cmd;

    // Diagnostic outputs
    if (solver_.result_flgs.delta_x_pred_opt > 0.009 || opt_attempts > 0) {
      if (solver_.result_flgs.delta_x_pred_opt > 0.3) {
        CLOG(WARNING, "path_tracker")
            << "Loop " << opt_attempts
            << " delta_x_pred: " << solver_.result_flgs.delta_x_pred_opt;
      }
    } else {
      break;
    }

  }  // loop
  // Use the feedback linearized control instead of driving slowly when MPC
  // randomly suggests direction switches
  if (solver_.result_flgs.flg_des_vel_one_point_turn) {
    return false;
  }
  return true;
}

int PathTrackerMPC::computeLookahead(
    const std::vector<VertexCtrlType> &scheduled_ctrl_mode,
    const int &current_pose_num, const int &max_lookahead) {
  // get the number of poses in the path
  int num_poses = scheduled_ctrl_mode.size();

  // compute the end of the longest possible look-ahead window
  int look_ahead_pose = std::min(current_pose_num + max_lookahead, num_poses);

  // Count the number of poses from the current pose until the next pose that is
  // not NORMAL or a DIR_SW_POSE/REGION
  int mpc_size = 1;
  if (look_ahead_pose > current_pose_num) {
    while ((current_pose_num + mpc_size < look_ahead_pose) &&
           (scheduled_ctrl_mode[current_pose_num + mpc_size] ==
                VertexCtrlType::NORMAL or
            scheduled_ctrl_mode[current_pose_num + mpc_size] ==
                VertexCtrlType::START or
            scheduled_ctrl_mode[current_pose_num + mpc_size] ==
                VertexCtrlType::DIR_SW_POSE or
            scheduled_ctrl_mode[current_pose_num + mpc_size] ==
                VertexCtrlType::DIR_SW_REGION)) {
      mpc_size++;
    }
  }
  return mpc_size;
}

bool PathTrackerMPC::rateLimitOutputs(float &v_cmd, float &w_cmd,
                                      const float &v_cmd_km1,
                                      const PathParams &params, float d_t) {
  if (fabs(v_cmd) > MAX_TOS_SPEED) {
    // Compute max velocity assuming max allowed acceleration
    float v_accel_max;
    if (v_cmd_km1 * v_cmd > 0.0) {
      // v_cmd_km1 and v_cmd have same sign
      v_accel_max = fabs(v_cmd_km1) + d_t * params.max_accel;
    } else {
      // v_cmd_km1 and v_cmd change sign
      v_accel_max = d_t * params.max_accel;
    }

    // Compute max velocity assuming max allowed wheel speed
    float des_curvature = fabs(w_cmd / v_cmd);
    float alpha = 1 / (params.w_max / params.v_max + des_curvature);
    float v_wheel_max = alpha * params.w_max;

    // Limit commands (if necessary)
    bool v_cmd_lt_accel = fabs(v_cmd) < v_accel_max;
    bool v_cmd_lt_wheel = fabs(v_cmd) < v_wheel_max;

    if (v_cmd_lt_accel && v_cmd_lt_wheel) {
      // Do nothing, commands ok TODO: Restructure if (...)
    } else {
      if (v_accel_max < v_wheel_max) {
        // CLOG(INFO, "path_tracker") << "Path tracker: Computed commands exceed
        // maximum acceleration.  Applying accel limits.";
        v_cmd = utils::getSign(v_cmd) * v_accel_max;
        w_cmd = utils::getSign(w_cmd) * fabs(des_curvature * v_cmd);
      } else {
        v_cmd = utils::getSign(v_cmd) * v_wheel_max;
        w_cmd = utils::getSign(w_cmd) * fabs(des_curvature * v_cmd);
      }
    }
  } else {
    // Treat as Turn-on-spot
    if (fabs(w_cmd) > params.w_max) {
      w_cmd = utils::getSign(w_cmd) * params.w_max;
    }
  }
  return true;
}

void PathTrackerMPC::initializeModelTrajectory(
    const int &mpcSize, MpcNominalModel &NominalModel, MpcSolverBase &Solver,
    const RCExperienceManagement &experience_management,
    local_path_t local_path) {
  // Create x_pred[0]
  Solver.x_pred.clear();
  Solver.x_pred.resize(mpcSize + 1);
  Solver.x_pred[0] = experience_management.experience_k_.x_k;

  if (mpc_params_.flg_use_vtr2_covariance) {
    Solver.x_pred[0].var_x_k = local_path.x_act_cov;
  } else {
    CLOG_N_TIMES(1, WARNING, "path_tracker")
        << "Path tracker not using covariance estimate from vtr2.";
    Solver.x_pred[0].var_x_k = 0.0001 * Eigen::MatrixXf::Identity(3, 3);
  }

  // Initialize x_pred vector
  for (int i = 1; i < mpcSize + 1; i++) {
    NominalModel.initialize_state(Solver.x_pred[i]);
  }
  Solver.x_opt = Solver.x_pred;
}

void PathTrackerMPC::rotateDisturbanceIntoPoseNumFrame(
    MpcNominalModel::model_state_t &x_input) {
  x_input.g_a_k_des_frame = x_input.g_a_k;
  x_input.var_g_a_k_des_frame = x_input.var_g_a_k;

  float th_k = x_input.x_k[2];
  Eigen::MatrixXf C_k_r(2, 2);
  C_k_r << cos(th_k), -sin(th_k), sin(th_k), cos(th_k);

  x_input.g_a_k_des_frame.block<2, 1>(0, 0) =
      C_k_r * x_input.g_a_k.block<2, 1>(0, 0);
  x_input.var_g_a_k_des_frame.block<2, 2>(0, 0) =
      C_k_r * x_input.var_g_a_k.block<2, 2>(0, 0) * C_k_r.transpose();
}

void PathTrackerMPC::locateNearestPose(local_path_t &local_path,
                                       unsigned initialGuess,
                                       unsigned radiusForwards,
                                       unsigned radiusBackwards) {
  CLOG(DEBUG, "path_tracker") << "[ChainLock Requested] locateNearestPose";
  chain_->lock();
  CLOG(DEBUG, "path_tracker") << "[ChainLock Acquired] locateNearestPose";
  tf2::Transform T_0_v = common::rosutils::toTfTransformMsg(
      chain_->pose(vision_pose_.trunkSeqId()) *
      vision_pose_.T_leaf_trunk().inverse());
  chain_->unlock();
  CLOG(DEBUG, "path_tracker") << "[ChainLock Released] locateNearestPose";

  unsigned bestGuess = initialGuess;
  bool forwardPoseSearch;
  if (radiusBackwards == 0) {
    forwardPoseSearch = true;
  } else {
    forwardPoseSearch = false;
  }

  tf2::Vector3 p_0_v_0(T_0_v.getOrigin());
  tf2::Quaternion q_0_v_0(T_0_v.getRotation());

  geometry_msgs::msg::Vector3 rpy_0_v_0 = common::rosutils::quat2rpy(q_0_v_0);
  double k_omega = 0;
  if (path_->scheduled_ctrl_mode_[bestGuess] == VertexCtrlType::TURN_ON_SPOT) {
    k_omega = 4;
  }

  double bestDistance = 1000;
  unsigned searchEnd;

  // Search forwards:
  int numPoses = path_->numPoses();
  if ((initialGuess + radiusForwards) < unsigned(numPoses)) {
    searchEnd = initialGuess + radiusForwards;
  } else {
    searchEnd = numPoses;
  }

  for (unsigned n = initialGuess; n <= searchEnd - 1; n++) {
    int nm1 = std::max(0, (int)n - 1);
    tf2::Vector3 p_0_n_0;
    tf2::Quaternion q_0_n_0;

    // Get translation and rotation of pose n
    geometryPoseToTf(path_->poses_[n], p_0_n_0, q_0_n_0);
    geometry_msgs::msg::Vector3 rpy_0_n_0 = common::rosutils::quat2rpy(q_0_n_0);

    // Approximate distance between robot and pose n
    double length;
    path_->computeDpMag(p_0_n_0, p_0_v_0, length);

    // Approximate rotation between robot and pose n
    double rotation;
    path_->computeDphiMag(rpy_0_v_0, rpy_0_n_0, rotation);

    // Set the bestDistance as the current distance + factor for angular
    // "distance"
    double distance = length + k_omega * rotation;

    if (distance < bestDistance) {
      bestDistance = distance;
      bestGuess = (uint64_t)n;
    }
    if (forwardPoseSearch &&
        path_->scheduled_ctrl_mode_[nm1] == VertexCtrlType::DIR_SW_POSE) {
      break;
    }
  }

  // Search backwards
  if (radiusBackwards < initialGuess) {
    searchEnd = initialGuess - radiusBackwards;
  } else {
    searchEnd = 0;
  }
  if (initialGuess > 0) {
    for (uint64_t n = initialGuess; n > searchEnd; n--) {
      tf2::Vector3 p_0_n_0;
      tf2::Quaternion q_0_n_0;

      // Get translation and rotation of pose n
      geometryPoseToTf(path_->poses_[n], p_0_n_0, q_0_n_0);
      geometry_msgs::msg::Vector3 rpy_0_n_0 =
          common::rosutils::quat2rpy(q_0_n_0);

      // Estimate length between robot and pose n
      double length;
      path_->computeDpMag(p_0_n_0, p_0_v_0, length);

      // Estimate rotation between robot and pose n
      double rotation;
      path_->computeDphiMag(rpy_0_v_0, rpy_0_n_0, rotation);

      // Set the bestDistance as the current distance + factor for angular
      // "distance"
      double distance = length + k_omega * rotation;
      CLOG(INFO, "path_tracker")
          << "Backwards search - pose: " << n << " with distance " << distance;

      if (distance < bestDistance) {
        bestDistance = distance;
        bestGuess = (uint64_t)n;
      }
    }
  }

  if (path_->scheduled_ctrl_mode_[bestGuess] == VertexCtrlType::DIR_SW_POSE) {
    if (bestDistance < 0.45) {
      // Note distance includes length and rotation
      CLOG(DEBUG, "path_tracker")
          << "Passing DIR_SW_POSE with dist: " << bestDistance
          << " and best guess " << bestGuess;
      if (bestGuess <= unsigned(numPoses - 2)) {
        bestGuess++;
        CLOG(DEBUG, "path_tracker") << "incremented bestGuess";
      }
    }
  } else {
    // Ensure splineRegion_ is ahead (behind) robot if v_cmd is positive
    // (negative, respectively)
    tf2::Vector3 p_0_bestGuess_0(0, 0, 0);
    common::rosutils::getTfPoint(path_->poses_[bestGuess], p_0_bestGuess_0);

    tf2::Vector3 p_v_bestGuess_v = T_0_v.inverse() * p_0_bestGuess_0;
    float Error_x = p_v_bestGuess_v.getX();

    float sign_v_cmd = utils::getSign(path_->scheduled_speed_[bestGuess]);
    if ((Error_x < 0 && sign_v_cmd > 0) || (Error_x > 0 && sign_v_cmd < 0)) {
      bestGuess = std::min((int)bestGuess + 1, numPoses - 1);
    }
  }

  local_path.T_0_v = T_0_v;
  // local_path.transformStamp = T_0_v.stamp_;
  local_path.current_pose_num = bestGuess;
  local_path.current_vertex_id = path_->vertexID(bestGuess);
  int bestGuess_p1 = std::min((int)bestGuess + 1, numPoses - 1);
  local_path.next_vertex_id = path_->vertexID(bestGuess_p1);

  CLOG(DEBUG, "path_tracker")
      << "[local_path] best guess pose num: " << bestGuess
      << ", giving current_vertex_id: " << local_path.current_vertex_id
      << " and next_vertex_id " << local_path.next_vertex_id;
}

void PathTrackerMPC::geometryPoseToTf(const geometry_msgs::msg::Pose &pose,
                                      tf2::Vector3 &point,
                                      tf2::Quaternion &quaternion) {
  point.setX(pose.position.x);
  point.setY(pose.position.y);
  point.setZ(pose.position.z);
  quaternion.setX(pose.orientation.x);
  quaternion.setY(pose.orientation.y);
  quaternion.setZ(pose.orientation.z);
  quaternion.setW(pose.orientation.w);
}

void PathTrackerMPC::reset() {
  CLOG(INFO, "path_tracker") << "Path tracker resetting for new run";
  vision_pose_.reset();
}

}  // namespace path_tracker
}  // namespace vtr
