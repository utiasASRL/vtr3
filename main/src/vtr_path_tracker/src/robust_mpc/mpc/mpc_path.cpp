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
 * \file mpc_path.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_path_tracker/robust_mpc/mpc/mpc_path.hpp>

namespace vtr {
namespace path_tracker {

void MpcPath::loadGainScheduleConfigFile() {
  // clang-format off
  const auto dw = node_->declare_parameter<std::vector<double>>(param_prefix_ + ".curvature_thresholds", std::vector<double>{0.01, 0.2, 1.5, 5.0, 10.0});
  const auto v = node_->declare_parameter<std::vector<double>>(param_prefix_ + ".target_linear_speed", std::vector<double>{1.01, 1.02, 1.03, 1.04, 1.05});
  const auto k_eh = node_->declare_parameter<std::vector<double>>(param_prefix_ + ".heading_error_gain", std::vector<double>{0.75, 0.75, 0.75, 0.75, 0.75});
  const auto k_el = node_->declare_parameter<std::vector<double>>(param_prefix_ + ".lateral_error_gain", std::vector<double>{0.3, 0.3, 0.3, 0.3, 0.3});
  const auto sat = node_->declare_parameter<std::vector<double>>(param_prefix_ + ".saturation_limit", std::vector<double>{2.0, 2.0, 2.0, 2.0, 2.0});
  const auto ld = node_->declare_parameter<std::vector<double>>(param_prefix_ + ".look_ahead_distance", std::vector<double>{0.75, 0.75, 1.2, 1.5, 1.5});
  const auto la = node_->declare_parameter<std::vector<double>>(param_prefix_ + ".angular_look_ahead", std::vector<double>{0.3, 0.3, 0.3, 0.3, 0.3});
  // clang-format on

  /// Load curvature thresholds
  params_.curvature_thresholds.clear();
  for (unsigned int i = 0; i < dw.size(); i++) {
    params_.curvature_thresholds.push_back(dw[i]);

    if (params_.curvature_thresholds[i] < 0) {
      std::string err{
          "Curvature config thresholds cannot be smaller than zero."};
      CLOG(ERROR, "path_tracker") << err;
      throw std::runtime_error{err};
    } else if (i > 0 && params_.curvature_thresholds[i] <=
                            params_.curvature_thresholds[i - 1]) {
      std::string err{
          "Curvature config thresholds must be monotonically increasing."};
      CLOG(ERROR, "path_tracker") << err;
      throw std::runtime_error{err};
    }
  }

  if (dw.size() != v.size()) {
    std::string err{
        "Number of curvature thresholds does not match number of speed "
        "schedules."};
    CLOG(ERROR, "path_tracker") << err;
    throw std::runtime_error{err};
  }

  CLOG(DEBUG, "path_tracker")
      << "Loaded curvature thresholds: " << params_.curvature_thresholds;

  /// Local scheduled speeds and gains
  params_.speed_schedules.clear();
  params_.speed_schedules.resize(v.size());
  gain_schedules_.resize(v.size());

  // declare a variable for the current gain schedule
  GainSchedule gain_schedule_tmp;

  // Load values of gainSchedule independent of speed from ROS parameter server
  // clang-format off
  gain_schedule_tmp.tos_angular_speed = node_->declare_parameter<double>(param_prefix_ + ".angular_speed_turn_on_spot", 0.2);
  gain_schedule_tmp.tos_x_error_gain = node_->declare_parameter<double>(param_prefix_ + ".gain_x_error_turn_on_spot", 1.0);
  gain_schedule_tmp.end_heading_error_gain = node_->declare_parameter<double>(param_prefix_ + ".gain_heading_error_ctrl_to_end", 1.0);
  gain_schedule_tmp.end_x_error_gain = node_->declare_parameter<double>(param_prefix_ + ".gain_x_error_ctrl_to_end", 1.0);
  gain_schedule_tmp.dir_sw_heading_error_gain = node_->declare_parameter<double>(param_prefix_ + ".gain_heading_error_ctrl_to_dir_sw", 1.0);
  gain_schedule_tmp.dir_sw_x_error_gain = node_->declare_parameter<double>(param_prefix_ + ".gain_x_error_ctrl_to_dir_sw", 1.0);
  // clang-format on

  // Set each level of the gain schedule
  for (unsigned i = 0; i < v.size(); i++) {
    // Check that scheduled speeds are positive and monotonically increasing
    if (v[i] <= 0) {
      std::string err{"Path tracker speed schedule must be positive."};
      CLOG(ERROR, "path_tracker") << err;
      throw std::runtime_error{err};
    }
    if (i > 0 && v[i - 1] >= v[i]) {
      std::string err{
          "Path tracker speed schedule must be monotonically increasing."};
      CLOG(ERROR, "path_tracker") << err;
      throw std::runtime_error{err};
    }

    gain_schedule_tmp.target_linear_speed = v[i];
    gain_schedule_tmp.heading_error_gain = k_eh[i];
    gain_schedule_tmp.lateral_error_gain = k_el[i];
    gain_schedule_tmp.saturation_limit = sat[i];
    gain_schedule_tmp.look_ahead_distance = ld[i];
    gain_schedule_tmp.angular_look_ahead = la[i];

    /// \todo (old) Why keep speed schedules AND gain schedules around?
    params_.speed_schedules[i] = v[i];
    gain_schedules_[i] = gain_schedule_tmp;
  }

  // get the minimum scheduled speed
  params_.min_speed = params_.min_speed < params_.speed_schedules[0]
                          ? params_.min_speed
                          : params_.speed_schedules[0];

  // Generate negative speeds from positive
  int num_speeds = params_.speed_schedules.size();
  int desired_size = 2 * num_speeds;

  std::vector<GainSchedule> temp_schedule(gain_schedules_);
  gain_schedules_.resize(desired_size);
  params_.speed_schedules.resize(desired_size);

  for (int i = 0; i < desired_size; i++) {
    if (i < num_speeds) {
      gain_schedules_[i] = temp_schedule[num_speeds - i - 1];
      gain_schedules_[i].target_linear_speed =
          -gain_schedules_[i].target_linear_speed;
      params_.speed_schedules[i] = gain_schedules_[i].target_linear_speed;
    } else {
      gain_schedules_[i] = temp_schedule[i - num_speeds];
      params_.speed_schedules[i] = gain_schedules_[i].target_linear_speed;
    }
  }

  // Debugging
  std::stringstream ss;
  // clang-format off
  ss << "Gain & Speed Schedule Parameters" << std::endl;
  ss << "Linear Speed | Gain (H, L) | Saturation | Look Ahead (L, A) | TOS (X, A) | End Ctrl (X, A) | Dir Switch (X, A)"
     << std::endl;
  for (unsigned i = 0; i < params_.speed_schedules.size(); i++) {
    ss << std::setw(6) << params_.speed_schedules[i]
       << std::setw(6) << gain_schedules_[i].target_linear_speed
       << std::setw(7) << gain_schedules_[i].heading_error_gain
       << std::setw(7) << gain_schedules_[i].lateral_error_gain
       << std::setw(13) << gain_schedules_[i].saturation_limit
       << std::setw(10) << gain_schedules_[i].look_ahead_distance
       << std::setw(10) << gain_schedules_[i].angular_look_ahead
       << std::setw(7) << gain_schedules_[i].tos_x_error_gain
       << std::setw(6) << gain_schedules_[i].tos_angular_speed
       << std::setw(9) << gain_schedules_[i].end_x_error_gain
       << std::setw(9) << gain_schedules_[i].end_heading_error_gain
       << std::setw(10) << gain_schedules_[i].dir_sw_x_error_gain
       << std::setw(10) << gain_schedules_[i].dir_sw_heading_error_gain
       << std::endl;
  }
  // clang-format on
  CLOG(DEBUG, "path_tracker") << ss.str();
}

void MpcPath::loadPathParams() {
  // clang-format off
  params_.flg_allow_turn_on_spot = node_->declare_parameter<bool>(param_prefix_ + ".enable_turn_on_spot", false);
  params_.flg_slow_start = node_->declare_parameter<bool>(param_prefix_ + ".enable_slow_start", true);

  // speed limits
  params_.v_max = node_->declare_parameter<double>(param_prefix_ + ".max_allowable_linear_speed", 1.0);
  params_.v_max_slow = node_->declare_parameter<double>(param_prefix_ + ".max_allowable_slow_linear_speed", 1.0);
  params_.w_max = node_->declare_parameter<double>(param_prefix_ + ".max_allowable_angular_speed", 1.5);
  params_.max_accel = node_->declare_parameter<double>(param_prefix_ + ".max_allowable_acceleration", 0.1);
  params_.max_decel = node_->declare_parameter<double>(param_prefix_ + ".max_allowable_deceleration", 0.05);  // for scheduling

  // slow speed zone for direction switch, turn on the spot, and end of path
  params_.min_slow_speed_zone_length = node_->declare_parameter<double>(param_prefix_ + ".slow_speed_zone_length", 0.4);

  // turn on spot classification
  params_.max_dx_turnOnSpotMode = node_->declare_parameter<double>(param_prefix_ + ".max_pose_separation_turn_on_spot_mode", 0.15);
  params_.max_turn_radius_turnOnSpotMode = node_->declare_parameter<double>(param_prefix_ + ".max_path_turn_radius_turn_on_spot_mode", 0.9);

  // thresholds used to determine when path is complete
  params_.path_end_x_thresh = node_->declare_parameter<double>(param_prefix_ + ".path_end_x_threshold", 0.05);
  params_.path_end_heading_thresh = node_->declare_parameter<double>(param_prefix_ + ".path_end_heading_threshold", 0.05);

  // thresholds for tracking error (this also becomes MPC contraints)
  params_.default_tight_tracking_error = node_->declare_parameter<double>(param_prefix_ + ".default_tight_tracking_error", 0.1);
  params_.default_loose_tracking_error = node_->declare_parameter<double>(param_prefix_ + ".default_loose_tracking_error", 0.3);
  params_.max_tracking_error_rate_of_change = node_->declare_parameter<double>(param_prefix_ + ".max_tracking_error_rate_of_change", 0.3);
  params_.default_heading_constraint = node_->declare_parameter<double>(param_prefix_ + ".max_heading_constraint", 0.2); // rads

  // parameters for resetting from a pause
  params_.min_speed = node_->declare_parameter<double>(param_prefix_ + ".min_speed", 0.0);
  params_.reset_from_pause_slow_speed = node_->declare_parameter<double>(param_prefix_ + ".reset_from_pause_slow_speed", 0.3);
  params_.reset_from_pause_slow_speed_zone_length_vertices = node_->declare_parameter<int>(param_prefix_ + ".reset_from_pause_slow_speed_zone_length_vertices", 2);
  // clang-format on

  // thresholds for tracking error
  // clang-format off
  CLOG(DEBUG, "path_tracker") << "Loading path parameters";
  CLOG(DEBUG, "path_tracker") << "enable_turn_on_spot: " << params_.flg_allow_turn_on_spot;
  CLOG(DEBUG, "path_tracker") << "enable_slow_start: " << params_.flg_slow_start;
  CLOG(DEBUG, "path_tracker") << "max_allowable_linear_speed: " << params_.v_max;
  CLOG(DEBUG, "path_tracker") << "max_allowable_slow_linear_speed: " << params_.v_max_slow;
  CLOG(DEBUG, "path_tracker") << "max_allowable_angular_speed: " << params_.w_max;
  CLOG(DEBUG, "path_tracker") << "max_allowable_acceleration: " << params_.max_accel;
  CLOG(DEBUG, "path_tracker") << "max_allowable_deceleration: " << params_.max_decel;  // For use in scheduling
  CLOG(DEBUG, "path_tracker") << "min_slow_speed_zone_length: " << params_.min_slow_speed_zone_length;
  CLOG(DEBUG, "path_tracker") << "max_pose_separation_turn_on_spot_mode: " << params_.max_dx_turnOnSpotMode;
  CLOG(DEBUG, "path_tracker") << "max_path_turn_radius_turn_on_spot_mode: " << params_.max_turn_radius_turnOnSpotMode;
  CLOG(DEBUG, "path_tracker") << "path_end_x_threshold: " << params_.path_end_x_thresh;
  CLOG(DEBUG, "path_tracker") << "path_end_heading_threshold: " << params_.path_end_heading_thresh;
  CLOG(DEBUG, "path_tracker") << "default_tight_tracking_error: " << params_.default_tight_tracking_error;
  CLOG(DEBUG, "path_tracker") << "default_loose_tracking_error: " << params_.default_loose_tracking_error;
  CLOG(DEBUG, "path_tracker") << "max_tracking_error_rate_of_change: " << params_.max_tracking_error_rate_of_change;
  CLOG(DEBUG, "path_tracker") << "max_heading_constraint: " << params_.default_heading_constraint;  // Radians
  // clang-format on
}

void MpcPath::extractPathInformation(const Chain::ConstPtr &chain) {
  num_poses_ = chain->sequence().size();
  if (num_poses_ < 1) {
    std::string err{"Path for path tracker has less than 1 pose."};
    CLOG(ERROR, "path_tracker") << err;
    throw std::runtime_error{err};
  }

  // Clear raw path variables
  poses_.clear();
  poses_.resize(num_poses_);

  turn_radius_.clear();
  turn_radius_.resize(num_poses_);

  turn_angle_.clear();
  turn_angle_.resize(num_poses_);

  travel_backwards_.clear();
  travel_backwards_.resize(num_poses_);

  dist_from_start_.clear();
  dist_from_start_.resize(num_poses_);

  dx_.clear();
  dx_.resize(num_poses_);

  vertex_Id_.clear();
  vertex_Id_.resize(num_poses_);

  tf2::Quaternion q_0_n_0, q_0_np1_0;
  tf2::Vector3 p_0_n_0, p_0_np1_0;
  geometry_msgs::msg::Vector3 rpy_0_n_0, rpy_0_np1_0;

  /** Prepare lists that have as many entries as poses **/
  // Prepare pose 0
  poses_[0] =
      common::rosutils::toPoseMessage(chain->pose(chain->begin()).matrix());
  geometryPoseToTf(poses_[0], p_0_n_0, q_0_n_0);
  rpy_0_n_0 = common::rosutils::quat2rpy(q_0_n_0);
  dist_from_start_[0] = 0.;
  turn_angle_[0] = 0.;
  turn_radius_[0] = 0.;
  dx_[0] = 0.;
  vertex_Id_[0] = chain->begin()->v()->id();
  largest_vertex_Id_ = vertex_Id_[0];

  // Extract the path poses from the chain
  for (int n = 0; n < num_poses_; n++) {
    // compute the n + 1 pose index and iterator. Guard against overflow.
    int np1 = std::min(n + 1, num_poses_ - 1);
    auto it_np1 = chain->begin() + np1;
    auto it_n = chain->begin() + n;

    // Extract pose np1
    poses_[np1] = common::rosutils::toPoseMessage(chain->pose(it_np1).matrix());

    // Get the points and quaternions required for this segment
    geometryPoseToTf(poses_[np1], p_0_np1_0, q_0_np1_0);
    rpy_0_np1_0 = common::rosutils::quat2rpy(q_0_np1_0);

    // Estimate rotation change between pose n and np1
    computeDphiMag(rpy_0_n_0, rpy_0_np1_0, turn_angle_[np1]);
    // This just computes the norm of the rpy between poses n and np1

    // Estimate distance between pose n and np1
    computeDpMag(p_0_n_0, p_0_np1_0, dx_[np1]);
    dist_from_start_[np1] = dist_from_start_[n] + dx_[np1];

    // Estimate curvature between pose n and np1, for look-ahead curvature
    // calculation
    computePoseCurvature(turn_angle_[np1], dx_[np1], turn_radius_[n]);

    // Find direction switches
    tf2::Transform C_0_n;
    C_0_n.setIdentity();
    C_0_n.setRotation(q_0_n_0);
    tf2::Vector3 p_n_np1_n = C_0_n.inverse() * (p_0_np1_0 - p_0_n_0);

    // Set direction flag
    if (p_n_np1_n.getX() < 0.) {
      travel_backwards_[n] = true;
    } else {
      travel_backwards_[n] = false;
    }

    // Get the vertex ID
    vertex_Id_[n] = it_n->v()->id();
    if (static_cast<int>(vertex_Id_[n]) > largest_vertex_Id_) {
      largest_vertex_Id_ = vertex_Id_[n];
    }

    // Prepare for next iteration
    p_0_n_0 = p_0_np1_0;
    q_0_n_0 = q_0_np1_0;
    rpy_0_n_0 = common::rosutils::quat2rpy(q_0_n_0);
  }

  // set variables for the last pose in the path
  turn_radius_[num_poses_ - 1] = 0;
  if (num_poses_ > 1) {
    travel_backwards_[num_poses_ - 1] = travel_backwards_[num_poses_ - 2];
  } else {
    travel_backwards_[0] = false;
  }

  // Get the distance from the start of the path.
  dist_by_vertexId_.clear();
  pose_num_by_vertex_id_.clear();

  for (int n = 0; n < num_poses_; n++) {
    dist_by_vertexId_[vertex_Id_[n]] = dist_from_start_[n];
    pose_num_by_vertex_id_[vertex_Id_[n]] = n;
  }
}

void MpcPath::getSpeedProfile() {
  int N = num_poses_;

  // clear and resize important vectors
  scheduled_ctrl_mode_.clear();
  scheduled_ctrl_mode_.resize(N);
  scheduled_speed_.clear();
  scheduled_speed_.resize(N);
  poses_tol_positive_.clear();
  poses_tol_positive_.resize(N);
  poses_tol_negative_.clear();
  poses_tol_negative_.resize(N);
  poses_heading_constraint_pos_.clear();
  poses_heading_constraint_pos_.resize(N);
  poses_heading_constraint_neg_.clear();
  poses_heading_constraint_neg_.resize(N);
  gain_schedule_idx_.clear();
  gain_schedule_idx_.resize(N);

  // Set control modes for each vertex of the path
  setInitialPathModes();
  findFalsePositiveDirSwPoses();

  // Some smoothing... could this go after expandDirSwAndEndModes?
  smoothCurvature();
  expandDirSwAndEndModes();

  // Assign speed schedule
  assignSpeedProfileAndTrackingtolerance();
  smoothScheduledSpeed();

  // Set speed schedule to one of the discrete user specified values
  floorSpeedSchedToDiscreteConfig();

  original_scheduled_speed_ = scheduled_speed_;

  if (params_.flg_slow_start) {
    CLOG(INFO, "path_tracker") << "Modify speed schedule for slow start.";
    // Finally, taper the speed profile up at the beginning
    int slow_speed_length = 2;  // vertices
    double slow_speed = 0.3;
    int start_region = 0;
    adjustSpeedProfileHoldSpeed(start_region, slow_speed_length, slow_speed);
  }

  CLOG(INFO, "path_tracker") << "Speed schedule set.";
}

void MpcPath::setInitialPathModes() {
  CLOG(INFO, "path_tracker") << "Setting initial control modes.";

  int N = num_poses_;
  scheduled_ctrl_mode_[0] =
      VertexCtrlType::START;  // Necessary for DIR_SW_POSE check

  for (int n = 0; n <= N - 1; n++) {
    int nm1 = n - 1;  // n-minus-1, used to protect when n=0
    if (n == 0) nm1 = n;

    int np1 = n + 1;  // n-plus-1
    if (n + 1 > N - 1) np1 = n;

    // Finally, set path mode: TURN_ON_SPOT
    if ((turn_radius_[n] < params_.max_turn_radius_turnOnSpotMode) &&
        (dx_[np1] < params_.max_dx_turnOnSpotMode) &&
        params_.flg_allow_turn_on_spot) {
      scheduled_ctrl_mode_[n] = VertexCtrlType::TURN_ON_SPOT;

      // START
    } else if (dist_from_start_[n] < params_.min_slow_speed_zone_length) {
      scheduled_ctrl_mode_[n] = VertexCtrlType::START;

      // DIR_SW
    } else if (travel_backwards_[n] != travel_backwards_[nm1]) {
      scheduled_ctrl_mode_[nm1] = VertexCtrlType::DIR_SW_POSE;
      scheduled_ctrl_mode_[n] = VertexCtrlType::NORMAL;

      // NORMAL
    } else {
      scheduled_ctrl_mode_[n] = VertexCtrlType::NORMAL;
    }
  }
  // Set the control mode at the last vertex to zero
  scheduled_ctrl_mode_[N - 1] = VertexCtrlType::END;
}

void MpcPath::findFalsePositiveDirSwPoses() {
  CLOG(INFO, "path_tracker") << "Removing false positive direction switches.";

  int N = num_poses_;
  int dir_sw_window_len = 11;
  int dir_sw_window_center = (dir_sw_window_len - 1) / 2;
  std::deque<double> dir_sw_window;
  double dir_sw_window_sum = 0.0;

  for (int n = 0; n <= N - 2; n++) {
    if (travel_backwards_[n] == 1) {
      dir_sw_window.push_front(-1);
    } else {
      dir_sw_window.push_front(1);
    }

    if (n > (dir_sw_window_len - 1)) {
      dir_sw_window_sum = 0;
      for (int i = 0; i < dir_sw_window_len; i++) {
        // Implement wavelet filter
        if (i < dir_sw_window_center) {
          dir_sw_window_sum += dir_sw_window.at(i);
        } else if (i > dir_sw_window_center) {
          dir_sw_window_sum -= dir_sw_window.at(i);
        }
      }

      dir_sw_window.pop_back();

      if (scheduled_ctrl_mode_[n - dir_sw_window_center] ==
          VertexCtrlType::DIR_SW_POSE) {
        if (std::abs(dir_sw_window_sum) < 5) {
          CLOG(INFO, "path_tracker")
              << "Invalid dir sw pose (" << n - dir_sw_window_center
              << ")....................with dir_sw indicator: "
              << std::abs(dir_sw_window_sum) << "/10";
          scheduled_ctrl_mode_[n - dir_sw_window_center] =
              VertexCtrlType::NORMAL;
        } else {
          CLOG(INFO, "path_tracker")
              << "Valid dir sw pose (" << n - dir_sw_window_center
              << ").....................with dir_sw indicator: "
              << std::abs(dir_sw_window_sum) << "/10";
        }
      }
    }  // if (n > dir_sw_window_len - 1)
  }    // for loop
}

void MpcPath::smoothCurvature() {
  CLOG(INFO, "path_tracker") << "Smoothing curvature estimate of the path.";
  int N = num_poses_;

  int curv_window_length = 3;
  int curv_window_center = (curv_window_length - 1) / 2;
  double curvature_sum = 0.0;
  std::queue<double> curv_window;

  for (int n = 0; n <= N - 2; n++) {
    // Smooth curvature estimate
    curv_window.push(turn_radius_[n]);

    if (n > curv_window_length - 1) {
      curvature_sum = curvature_sum + turn_radius_[n] - curv_window.front();
      curv_window.pop();
      turn_radius_[n - curv_window_center] =
          curvature_sum / (float)curv_window_length;
    } else {
      curvature_sum = curvature_sum + turn_radius_[n];
    }

    // Root out false negatives in on-the-spot turns
    if (n < N - 2) {
      if (scheduled_ctrl_mode_[n] == VertexCtrlType::TURN_ON_SPOT) {
        if (scheduled_ctrl_mode_[n + 1] != VertexCtrlType::TURN_ON_SPOT &&
            scheduled_ctrl_mode_[n + 2] == VertexCtrlType::TURN_ON_SPOT) {
          scheduled_ctrl_mode_[n + 1] = VertexCtrlType::TURN_ON_SPOT;
        }
      }
    }

  }  // for loop
}

void MpcPath::expandDirSwAndEndModes() {
  CLOG(INFO, "path_tracker") << "Pre-processing control mode in regions around "
                                "direction switches and path end.";
  int N = num_poses_;

  // PASS 2: Expand DIR_SW and END path modes
  for (int n = 0; n <= N - 2; n++) {
    // Apply DIR_SW mode +/- bufferDistance from actual direction sw index
    int dir_sw_strt = 0;  // Represents index where "mode" should begin
    int initSearchPose = n;
    if (scheduled_ctrl_mode_[n] == VertexCtrlType::DIR_SW_POSE) {
      // Find pose index bufferDistance backwards from pose n
      bool getWindowForwards = false;
      getWindow(dist_from_start_, turn_angle_,
                params_.min_slow_speed_zone_length,
                50.0,            // No limit on angular distance
                dir_sw_strt,     // start
                initSearchPose,  // end
                getWindowForwards);

      CLOG(DEBUG, "path_tracker")
          << "After getWindow, dir_sw_strt: " << dir_sw_strt
          << "  initSearchPose" << initSearchPose;

      // Update path mode around pose n with DIR_SW
      for (int m = dir_sw_strt; m < initSearchPose; m++) {
        bool cancelDirSwZone =
            (scheduled_ctrl_mode_[m] == VertexCtrlType::TURN_ON_SPOT) ||
            (scheduled_ctrl_mode_[m] == VertexCtrlType::END) ||
            (scheduled_ctrl_mode_[m] == VertexCtrlType::START) ||
            (scheduled_ctrl_mode_[m] == VertexCtrlType::DIR_SW_POSE);
        if (!cancelDirSwZone) {
          scheduled_ctrl_mode_[m] = VertexCtrlType::DIR_SW_REGION;
        }
      }

      getWindowForwards = true;
      int dir_sw_end = 0;  // Represents index where "mode" should end
      initSearchPose = n;
      getWindow(dist_from_start_, turn_angle_,
                params_.min_slow_speed_zone_length,
                50.0,            // No limit on angular distance
                initSearchPose,  // start
                dir_sw_end,      // end
                getWindowForwards);

      for (int m = n; m < dir_sw_end; m++) {
        bool cancelDirSwZone =
            (scheduled_ctrl_mode_[m] == VertexCtrlType::TURN_ON_SPOT) ||
            (scheduled_ctrl_mode_[m] == VertexCtrlType::END) ||
            (scheduled_ctrl_mode_[m] == VertexCtrlType::START) ||
            (scheduled_ctrl_mode_[m] == VertexCtrlType::DIR_SW_POSE);

        if (!cancelDirSwZone) {
          scheduled_ctrl_mode_[m] = VertexCtrlType::DIR_SW_REGION;
        }
      }
    }

    // Apply "END" mode as approaching end of path
    if ((dist_from_start_[N - 1] - dist_from_start_[n] <
         params_.min_slow_speed_zone_length) &&
        scheduled_ctrl_mode_[n] != VertexCtrlType::TURN_ON_SPOT)
      scheduled_ctrl_mode_[n] = VertexCtrlType::END;
  }
}

void MpcPath::assignSpeedProfileAndTrackingtolerance() {
  CLOG(INFO, "path_tracker")
      << "Assigning speed profile based on curvature and control mode, and "
         "setting tracking error constraints to default.";
  int N = num_poses_;

  // PASS 3: Assign speed profile and tracking error tolerances
  for (int n = 0; n <= N - 1; n++) {
    // Set curvature_indx
    int curvature_indx = 0;
    if (scheduled_ctrl_mode_[n] == VertexCtrlType::NORMAL) {
      // Find closest curvature
      for (int i = 0; i <= (int)params_.curvature_thresholds.size() - 1; i++) {
        if (turn_radius_[n] >= params_.curvature_thresholds[i]) {
          curvature_indx = i;
        }
      }
    } else {  // start, end, turn-on-spot, dir_sw
      curvature_indx = 0;
    }

    // Set path constraints / tolerances
    if (scheduled_ctrl_mode_[n] == VertexCtrlType::END) {
      poses_tol_positive_[n] = params_.default_tight_tracking_error;
      poses_tol_negative_[n] = -params_.default_tight_tracking_error;
    } else {
      poses_tol_positive_[n] = params_.default_loose_tracking_error;
      poses_tol_negative_[n] = -params_.default_loose_tracking_error;
    }

    poses_heading_constraint_pos_[n] = params_.default_heading_constraint;
    poses_heading_constraint_neg_[n] = -params_.default_heading_constraint;

    // Set the speed profile based on curvature
    int num_curvature_calibrations = params_.curvature_thresholds.size();
    if (travel_backwards_[n]) {
      scheduled_speed_[n] =
          params_
              .speed_schedules[num_curvature_calibrations - curvature_indx - 1];
    } else {
      scheduled_speed_[n] =
          params_.speed_schedules[num_curvature_calibrations + curvature_indx];
    }
  }
}

void MpcPath::smoothScheduledSpeed() {
  CLOG(INFO, "path_tracker")
      << "Smoothing speed schedule based on max allowed acceleration.";
  int N = num_poses_;

  // Smooth speed profile based on max allowed acceleration
  for (int n = 1; n <= N - 1; n++) {
    double sign;
    if (travel_backwards_[n]) {
      sign = -1;
    } else {
      sign = 1;
    }

    // Estimate acceleration: a = (v2^2 - v1^2)/(2d)
    double safe_dx = std::max(0.01, dx_[n]);
    double accel_est =
        (pow(scheduled_speed_[n], 2) - pow(scheduled_speed_[n - 1], 2)) /
        (2 * safe_dx);

    if (accel_est > params_.max_accel) {
      scheduled_speed_[n] =
          sign * std::pow(std::pow(scheduled_speed_[n - 1], 2) +
                              2 * dx_[n] * params_.max_accel,
                          0.5);
    } else if (accel_est < -params_.max_decel) {
      // Estimate speed for pose nm1 given speed at pose n, then recursively
      // work backwards
      double estimated_speed =
          sign * std::pow(std::pow(scheduled_speed_[n], 2) +
                              2 * dx_[n] * params_.max_decel,
                          0.5);
      int m = n - 1;
      // "sign" applied to both sides to ensure we can check using only "greater
      // than"
      while (sign * scheduled_speed_[m] > sign * estimated_speed && m >= 1) {
        scheduled_speed_[m] = estimated_speed;
        estimated_speed = sign * std::pow(std::pow(scheduled_speed_[m], 2) +
                                              2 * dx_[m] * params_.max_decel,
                                          0.5);
        m--;
      }
    }

    // Compute change in tracking tolerance
    double tolerance_change_rate =
        (poses_tol_positive_[n] - poses_tol_positive_[n - 1]) / safe_dx;

    if (tolerance_change_rate > params_.max_tracking_error_rate_of_change) {
      poses_tol_positive_[n] =
          poses_tol_positive_[n - 1] +
          params_.max_tracking_error_rate_of_change * dx_[n];
      poses_tol_negative_[n] = -poses_tol_positive_[n];
    } else if (tolerance_change_rate <
               -params_.max_tracking_error_rate_of_change) {
      // Estimate tolerance for pose nm1 given tolerance at pose n, then
      // recursively work backwards
      double allowed_tolerance_nm1 =
          poses_tol_positive_[n] +
          params_.max_tracking_error_rate_of_change * dx_[n];
      int m = n - 1;
      while (poses_tol_positive_[m] > allowed_tolerance_nm1 && m >= 1) {
        poses_tol_positive_[m] = allowed_tolerance_nm1;
        poses_tol_negative_[m] = -allowed_tolerance_nm1;
        allowed_tolerance_nm1 =
            poses_tol_positive_[m] +
            params_.max_tracking_error_rate_of_change * dx_[m];
        m--;
      }
    }
  }

  // Smooth speed profile based on nearby scheduled speeds
  // TODO: Make smoothing_window_size a configurable parameter

  CLOG(INFO, "path_tracker")
      << "Smoothing speed schedule based on nearby speeds";

  int smoothing_window_size = 5;
  double window_sum = 0.0;
  std::vector<double> speed_profile_temp = scheduled_speed_;

  for (int n = 0; n <= N - 1; n++) {
    double sign;
    if (travel_backwards_[n]) {
      sign = -1;
    } else {
      sign = 1;
    }

    int window_start = std::max(0, n - smoothing_window_size);
    int window_end = std::min(N - 1, n + smoothing_window_size);
    auto window_size = (double)(window_end - window_start + 1);

    // Initialize window sum (0 -> window_end)
    if (n == 0) {
      for (int i = window_start; i <= window_end; i++) {
        window_sum = window_sum + std::abs(speed_profile_temp[i]);
      }

      // Add new points to window_sum, window size is growing
    } else if (n <= smoothing_window_size && n > 0) {
      window_sum = window_sum + std::abs(speed_profile_temp[window_end]);

      // Remove points from window_sum, window size is shrinking
    } else if (n >= N - smoothing_window_size) {
      window_sum = window_sum - std::abs(speed_profile_temp[window_start - 1]);

      // Add new points and remove old points, window size remains constant
    } else {
      window_sum = window_sum - std::abs(speed_profile_temp[window_start - 1]) +
                   std::abs(speed_profile_temp[window_end]);
    }

    if (n > 0 && n < (N - 1)) {
      scheduled_speed_[n] = sign * window_sum / window_size;
    }
  }
}

void MpcPath::smoothTolerancesFwd(const int &pose_num) {
  int n = pose_num;
  int np1 = pose_num + 1;

  if (np1 < static_cast<int>(poses_.size())) {
    double safe_dx = std::max(0.01, dx_[np1]);

    // Compute change in tracking tolerance
    double tolerance_change_rate =
        (poses_tol_positive_[np1] - poses_tol_positive_[n]) / safe_dx;

    if (tolerance_change_rate > params_.max_tracking_error_rate_of_change) {
      double allowed_tolerance_np1 =
          poses_tol_positive_[n] +
          params_.max_tracking_error_rate_of_change * dx_[np1];

      while (poses_tol_positive_[np1] > allowed_tolerance_np1 &&
             np1 < static_cast<int>(poses_.size()) - 1) {
        poses_tol_positive_[np1] = allowed_tolerance_np1;
        poses_tol_negative_[np1] = -allowed_tolerance_np1;
        allowed_tolerance_np1 =
            poses_tol_positive_[np1] +
            params_.max_tracking_error_rate_of_change * dx_[np1 + 1];
        np1++;
      }
    }
  }
}

void MpcPath::smoothTolerancesBck(const int &pose_num) {
  int n = pose_num;
  int nm1 = pose_num - 1;

  if (nm1 > 0) {
    double safe_dx = std::max(0.01, dx_[n]);

    // Compute change in tracking tolerance
    double tolerance_change_rate =
        (poses_tol_positive_[n] - poses_tol_positive_[nm1]) / safe_dx;

    if (tolerance_change_rate < -params_.max_tracking_error_rate_of_change) {
      // Estimate tolerance for pose nm1 given tolerance at pose n, then
      // recursively work backwards
      double allowed_tolerance_nm1 =
          poses_tol_positive_[n] +
          params_.max_tracking_error_rate_of_change * dx_[n];
      int nm1 = n - 1;

      while (nm1 >= 1 && poses_tol_positive_[nm1] > allowed_tolerance_nm1) {
        poses_tol_positive_[nm1] = allowed_tolerance_nm1;
        poses_tol_negative_[nm1] = -allowed_tolerance_nm1;
        allowed_tolerance_nm1 =
            poses_tol_positive_[nm1] +
            params_.max_tracking_error_rate_of_change * dx_[nm1];
        nm1--;
      }
    }
  }
}

void MpcPath::floorSpeedSchedToDiscreteConfig() {
  CLOG(INFO, "path_tracker") << "Converting speed profile to discrete gain "
                                "schedule for feedback-linearized controller.";
  int N = num_poses_;
  for (int n = 0; n <= N - 1; n++)
    gain_schedule_idx_[n] = findClosestSpeed(scheduled_speed_[n]);
}

int MpcPath::findClosestSpeed(float v) {
  // If including negative gain schedules
  int min_indx = -1;
  for (unsigned int i = 0; i < gain_schedules_.size(); i++) {
    // Positive gain schedules
    if (v > 0 && gain_schedules_[i].target_linear_speed > 0) {
      if (v <= gain_schedules_[i].target_linear_speed ||
          i == gain_schedules_.size() - 1 ||
          (v > gain_schedules_[i].target_linear_speed &&
           v < gain_schedules_[i + 1].target_linear_speed)) {
        min_indx = i;
        break;
      }
    } else if (v < 0 && gain_schedules_[i].target_linear_speed < 0) {
      if (v <= gain_schedules_[i].target_linear_speed ||
          v * gain_schedules_[i + 1].target_linear_speed < 0) {
        min_indx = i;
        break;
      }
    }
  }

  if (min_indx == -1) {
    std::string err;
    if (v < 0)
      err = "Negative speed encountered but no negative gain.";
    else if (v > 0)
      err = "Positive speed encountered but no positive gain.";
    else if (v == 0)
      err =
          "Cannot pass the path tracker a speed of zero because it doesn't "
          "know which gain schedule to use (i.e., a positive or negative)";
    CLOG(ERROR, "path_tracker") << err;
    throw std::runtime_error{err};
  }
  return min_indx;
}

void MpcPath::printPath() {
  std::stringstream ss;
  ss << "=== MPC Path Summary ===" << std::endl;
  ss << "Vertex ID | Dist from Start | Dist from Last | Turn Angle | Turn "
        "Radius | Forward | Translation (x, y, z) || Control Mode | Pose Tol "
        "(Pos, Neg) | Heading Constr (Pos, Neg) | Scheduled Speed | Gain "
        "Schedule Idx"
     << std::endl;
  for (size_t i = 0; i < poses_.size(); i++) {
    std::stringstream ss1;
    ss1 << vertex_Id_[i];
    ss << std::setw(9) << ss1.str();
    ss << std::setw(18) << dist_from_start_[i];
    ss << std::setw(17) << dx_[i];
    ss << std::setw(13) << turn_angle_[i];
    ss << std::setw(14) << turn_radius_[i];
    ss << std::setw(10) << !travel_backwards_[i];
    std::stringstream ss2;
    ss2 << "(" << std::setprecision(1) << std::fixed << poses_[i].position.x
        << ", " << std::setprecision(1) << std::fixed << poses_[i].position.y
        << ", " << std::setprecision(1) << std::fixed << poses_[i].position.z
        << ")";
    ss << std::setw(24) << ss2.str();
    ss << std::setw(16) << scheduled_ctrl_mode_[i];
    std::stringstream ss3;
    ss3 << "(" << std::setprecision(1) << poses_tol_positive_[i] << ", "
        << std::setprecision(1) << poses_tol_negative_[i] << ")";
    ss << std::setw(22) << ss3.str();
    std::stringstream ss4;
    ss4 << "(" << std::setprecision(1) << poses_heading_constraint_pos_[i]
        << ", " << std::setprecision(1) << poses_heading_constraint_neg_[i]
        << ")";
    ss << std::setw(28) << ss4.str();
    ss << std::setw(18) << scheduled_speed_[i];
    ss << std::setw(20) << gain_schedule_idx_[i];
    ss << std::endl;
  }
  ss << "Total Number of Poses: " << num_poses_ << std::endl;
  ss << "Largest Vertex Id: " << largest_vertex_Id_;
  CLOG(INFO, "path_tracker") << ss.str();
}

int MpcPath::getWindow(const std::vector<double> &path_length,
                       const std::vector<double> &path_turn_angles,
                       const double &distance_window,
                       const double &angular_window, int &start, int &end,
                       const bool get_future_window) {
  double d = 0;
  double omega = 0;
  uint64_t indx;
  if (get_future_window) {
    indx = start;
  } else {
    indx = end;
  }

  // Find window start / end depending on future/past flag
  while (d < distance_window && omega < angular_window) {
    if (get_future_window) {
      if (indx >= path_length.size() - 1) {
        break;
      } else {
        indx++;
      }
      d += path_length[indx] - path_length[indx - 1];
      omega +=
          path_turn_angles[indx];  // Not intended to be restricted to -PI -> PI
    } else {                       // Backwards
      if (indx <= 0) {
        break;
      } else {
        indx--;
      }
      d += path_length[indx + 1] - path_length[indx];
      omega +=
          path_turn_angles[indx +
                           1];  // Not intended to be restricted to -PI -> PI
    }
  }

  // Pass indx back, window will always include at least two poses
  if (get_future_window) {
    end = indx;
  } else {
    start = indx;
  }

  return indx;
}

void MpcPath::updatePathProgress(int &pose_i, int &pose_im1, const float v_des,
                                 const Eigen::VectorXf &x_k,
                                 const local_path_t local_path) {
  // check if the robot has passed pose i. If so, update pose_i and check the
  // next pose.
  for (int test_pose = 0; test_pose < 4; test_pose++) {
    bool passed_pose = checkIfPastPose(
        v_des, x_k, local_path.x_des_fwd.block<3, 1>(0, pose_i));
    if (passed_pose) {
      pose_i = std::min(pose_i + 1, (int)local_path.x_des_fwd.cols() - 1);
      pose_im1 = std::max(0, pose_i - 1);
    } else {
      continue;
    }
  }
}

bool MpcPath::checkIfPastPose(const float &v_des, const Eigen::VectorXf &x_k,
                              const Eigen::MatrixXf &x_desired) {
  float x_k_0 = x_k[0] - x_desired(0, 0);
  float y_k_0 = x_k[1] - x_desired(1, 0);
  float x_k_k = cos(x_desired(2, 0)) * x_k_0 + sin(x_desired(2, 0)) * y_k_0;

  if (v_des > 0 && x_k_k > 0) {
    return true;
  } else if (v_des < 0 && x_k_k < 0) {
    return true;
  } else {
    return false;
  }
}

void MpcPath::geometryPoseToTf(const geometry_msgs::msg::Pose &pose,
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

void MpcPath::computeDpMag(const tf2::Vector3 &p_0_n_0,
                           const tf2::Vector3 &p_0_np1_0, double &dp_mag) {
  tf2::Vector3 dp_n_np1_0;
  dp_n_np1_0 = p_0_np1_0 - p_0_n_0;
  dp_mag = pow(pow(dp_n_np1_0.getX(), 2) + pow(dp_n_np1_0.getY(), 2) +
                   pow(dp_n_np1_0.getZ(), 2),
               0.5);
}

void MpcPath::computeDphiMag(const geometry_msgs::msg::Vector3 &rpy_0_n_0,
                             const geometry_msgs::msg::Vector3 &rpy_0_np1_0,
                             double &dphi_mag) {
  // Estimate rotation change between pose n and np1
  dphi_mag = pow(utils::thetaWrap(rpy_0_np1_0.x - rpy_0_n_0.x), 2) +
             pow(utils::thetaWrap(rpy_0_np1_0.y - rpy_0_n_0.y), 2) +
             pow(utils::thetaWrap(rpy_0_np1_0.z - rpy_0_n_0.z), 2);
  dphi_mag = pow(dphi_mag, 0.5);
}

void MpcPath::computePoseCurvature(const double &angle, const double &dist,
                                   double &curvature) {
  if (std::abs(angle) > 0.0001) {
    curvature = std::abs(dist / angle);
  } else {
    curvature = std::abs(dist / 0.0001);
  }
}

void MpcPath::clearSpeedAndGainSchedules() {
  // initialize the current gain schedule
  current_gain_schedule_.target_linear_speed = 0;
  current_gain_schedule_.look_ahead_distance = 0;
  current_gain_schedule_.angular_look_ahead = 0;

  // initialize controller gains
  current_gain_schedule_.heading_error_gain = 0;
  current_gain_schedule_.lateral_error_gain = 0;
  current_gain_schedule_.tos_angular_speed = 0;
  current_gain_schedule_.tos_x_error_gain = 0;
  current_gain_schedule_.end_heading_error_gain = 0;
  current_gain_schedule_.end_x_error_gain = 0;
  current_gain_schedule_.dir_sw_heading_error_gain = 0;
  current_gain_schedule_.dir_sw_x_error_gain = 0;

  // initialize saturation limit
  current_gain_schedule_.saturation_limit = 0;
}

void MpcPath::clearCurrentGainSchedule() {
  // Initialize the currentGainSchedule
  current_gain_schedule_.target_linear_speed = 0;
  current_gain_schedule_.look_ahead_distance = 0;
  current_gain_schedule_.angular_look_ahead = 0;

  // Initialize controller gains
  current_gain_schedule_.heading_error_gain = 0;
  current_gain_schedule_.lateral_error_gain = 0;
  current_gain_schedule_.tos_angular_speed = 0;
  current_gain_schedule_.tos_x_error_gain = 0;
  current_gain_schedule_.end_heading_error_gain = 0;
  current_gain_schedule_.end_x_error_gain = 0;
  current_gain_schedule_.dir_sw_heading_error_gain = 0;
  current_gain_schedule_.dir_sw_x_error_gain = 0;

  // Initialize saturation limit
  current_gain_schedule_.saturation_limit = 0;
}

void MpcPath::adjustSpeedProfileHoldSpeed(int start, int length) {
  // Get current speed
  float target_speed = fabs(scheduled_speed_[start]);
  adjustSpeedProfileHoldSpeed(start, length, target_speed);
}

void MpcPath::adjustSpeedProfileHoldSpeed(int start, int length,
                                          double target_speed) {
  int numPoses = original_scheduled_speed_.size();
  if (start < numPoses) {
    int end;

    double scheduled_start_speed = std::min(
        fabs(original_scheduled_speed_[start]), fabs(scheduled_speed_[start]));
    target_speed = utils::getSign(original_scheduled_speed_[start]) *
                   std::min(fabs(scheduled_start_speed), fabs(target_speed));

    end = std::min(start + length, numPoses);
    if (start < numPoses - 1) {
      for (int i = start; i <= end; i++) {
        scheduled_speed_[i] =
            utils::getSign(scheduled_speed_[i]) *
            std::min(fabs(scheduled_speed_[i]), fabs(target_speed));
      }
    }
    adjustSpeedProfileTaperUp(end - 1);
  }
}

void MpcPath::adjustSpeedProfileTaperUp(int start) {
  int numPoses = original_scheduled_speed_.size();

  if (start < numPoses - 1) {
    int i = start;
    int im1 = std::max(0, i - 1);
    float dx;

    // Taper from im1
    double adjusted_speed = fabs(scheduled_speed_[im1]);

    bool done = false;
    do {
      // Compute speed at pose i while observing deceleration limits
      dx = dist_from_start_[i] - dist_from_start_[im1];
      adjusted_speed =
          pow(pow(adjusted_speed, 2) + 2 * dx * params_.max_accel, 0.5);
      adjusted_speed =
          std::min(adjusted_speed, fabs(original_scheduled_speed_[i]));

      scheduled_speed_[i] =
          std::min(adjusted_speed, fabs(original_scheduled_speed_[i])) *
          utils::getSign(original_scheduled_speed_[i]);

      i++;
      im1 = i - 1;

      if (i >= numPoses - 1) {
        break;
      }
      done =
          fabs(scheduled_speed_[im1]) >= fabs(original_scheduled_speed_[im1]);
    } while (!done);
  }
}

}  // namespace path_tracker
}  // namespace vtr
