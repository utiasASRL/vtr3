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
 * \file mpc_path.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include <vtr_common/rosutils/transformations.hpp>
#include <vtr_pose_graph/path/localization_chain.hpp>
#include <vtr_pose_graph/path/path.hpp>

#include <vtr_path_tracker/robust_mpc/mpc/mpc_types.hpp>  // for VertexTrackingType
#include <vtr_path_tracker/robust_mpc/mpc/utilities.hpp>

namespace vtr {
namespace path_tracker {

// useful typedef
using Chain = pose_graph::LocalizationChain;
using pose_graph::VertexId;

// The gain schedule struct
struct GainSchedule {
  double target_linear_speed;
  double look_ahead_distance;
  double angular_look_ahead;
  double heading_error_gain;
  double lateral_error_gain;
  double tos_angular_speed;
  double tos_x_error_gain;
  double end_heading_error_gain;
  double end_x_error_gain;
  double dir_sw_heading_error_gain;
  double dir_sw_x_error_gain;
  double saturation_limit;
};

struct PathParams {
  /// Control Mode Flags & Parameters
  bool flg_allow_turn_on_spot;
  bool flg_slow_start;

  double v_max, v_max_slow, w_max;
  double max_accel, max_decel;

  double min_slow_speed_zone_length;

  double max_dx_turnOnSpotMode;
  double max_turn_radius_turnOnSpotMode;

  // turn on spot, ctrl to end, ctrl to dir sw

  // path completion
  double path_end_x_thresh;
  double path_end_heading_thresh;

  /// thresholds for tracking error
  double default_tight_tracking_error;
  double default_loose_tracking_error;
  double max_tracking_error_rate_of_change;
  double default_heading_constraint;

  // parameters for resetting from a pause
  double min_speed;
  double reset_from_pause_slow_speed;
  int reset_from_pause_slow_speed_zone_length_vertices;

  // Schedules
  std::vector<double> speed_schedules;
  std::vector<double> curvature_thresholds;
};

/**
 * TODO: Using a std::map for path properties.
 * EXAMPLE FOR USING A MAP INDEXED BY SEQ_ID. Do it this way once finished the
 * first time through.
struct path_data {
  float pose_tol_positive;
};
std::map<uint64_t,path_data> path_data_map;
path_data_map[6] = path_data_map();
path_data_map.insert({6,path_data_map()});
path_data_map[6].pos_tol_positive;
*/

/**
 * \brief Class for storing additional information about the path important for
 * MPC.
 */
class MpcPath {
 public:
  using Vid = pose_graph::VertexId;

  /**
   * \brief Constructor
   * \param node pointer to the node handle for the node responsible for this
   * path.
   * \param param_prefix ros parameter namespace
   */
  MpcPath(const std::shared_ptr<rclcpp::Node> &node, std::string param_prefix)
      : node_(node), param_prefix_(param_prefix) {}

  /** \brief Pointer to node for reference */
  const std::shared_ptr<rclcpp::Node> node_;

  /** \brief Parameter namespace. Should be something like "path_tracker/" */
  std::string param_prefix_;

  /// Raw data, extracted for ease-of-use

  /**
   * \brief The transform from the origin to each vertex in the path.
   * T_vertexI_origin
   */
  std::vector<geometry_msgs::msg::Pose> poses_;

  std::vector<float> poses_tol_positive_;
  std::vector<float> poses_tol_negative_;
  std::vector<float> poses_heading_constraint_pos_;
  std::vector<float> poses_heading_constraint_neg_;
  std::vector<double> turn_radius_;
  std::vector<double> turn_angle_;
  std::vector<double> dist_from_start_;

  /** \brief Distance from point k to point k+1 along the x-axis of frame k */
  std::vector<double> dx_;
  /** \brief Whether the vehicle was traveling backwards */
  std::vector<bool> travel_backwards_;
  /** \brief Vertex id of each pose along the path */
  std::vector<Vid> vertex_Id_;

  // These vectors are of length max(vertexId)
  int largest_vertex_Id_;
  int num_poses_;
  std::unordered_map<Vid, double> dist_by_vertexId_;  /// \todo not unique
  std::unordered_map<Vid, uint64_t>
      pose_num_by_vertex_id_;  /// \todo not unique

  // Processed data
  std::vector<VertexCtrlType> scheduled_ctrl_mode_;
  std::vector<double> scheduled_speed_;
  std::vector<double> original_scheduled_speed_;

  // Params
  PathParams params_;

  // Vector of gain schedules.
  std::vector<GainSchedule> gain_schedules_;
  std::vector<size_t> gain_schedule_idx_;
  GainSchedule current_gain_schedule_;

  /**
   * \brief Loads parameters related to tracking error and speed/acceleration
   * constraints for speed scheduling
   */
  void loadPathParams();

  /** \brief Loads a gain schedule configuration file. */
  void loadGainScheduleConfigFile();

  /**
   * \brief Extract additional information important for speed scheduling from
   * the path
   * \details Given the localization chain which contains the path to be driven,
   * extract the path curvature, turn angle, turn radius, distance from start,
   * and largest vertex ID.
   * \param chain the localization chain
   * \todo (old) remove all conversions from tf/lgmath/geometry_msgs and just
   * use lgmath. This will require some additions to lgmath.
   */
  void extractPathInformation(const Chain::ConstPtr &chain);

  /**
   * \brief Gets the speed profile based on configuration parameters.
   * \note Chris O's speed profiler used to set suggested speeds based on past
   * experiences.
   * \param chain  Pointer to the localization chain.
   */
  void getSpeedProfile();

  /// Helper functions for speed scheduling

  /**
   * \brief Sets the initial path modes to TURN_ON_SPOT, START, NORMAL,
   * DIR_SW_POSE, END based on thresholds in params_.
   */
  void setInitialPathModes();

  /**
   * \brief Finds false positive direction switches.
   * \details Good direction switches have enough travel_backwards ahead of them
   * and travel forwards behind them.
   * \todo (old) Make parameters configurable.
   */
  void findFalsePositiveDirSwPoses();

  /** \brief Smoothes the curvature estimate for the desired path */
  void smoothCurvature();

  /**
   * \brief Expands DIR_SW and END path modes in scheduled_ctrl_mode_.
   * \details
   *   1. Set scheduled_ctrl_mode_ around DIR_SW_POSE to DIR_SW_REGION
   *   2. Set scheduled_ctrl_mode_ to END for poses within
   *      params_.min_slow_speed_zone_length of the end of the path.
   */
  void expandDirSwAndEndModes();

  /**
   * \brief Assign speed profile and tracking error tolerances
   * \todo (old) The speed schedule was set based on experience and curvature.
   * We should be able to set the desired speed to a max and let the safe
   * learning figure out the rest.
   */
  void assignSpeedProfileAndTrackingtolerance();

  /**
   * \brief Smooth the scheduled speed based on max allowed acceleration and
   * nearby scheduled speeds.
   */
  void smoothScheduledSpeed();

  /**
   * \brief Make sure path tracking tolerances changes smoothly
   * \param pose_num The pose number for the end of a segment who's tolerances
   * have been modified
   */
  void smoothTolerancesFwd(const int &pose_num);

  /**
   * \brief Make sure path tracking tolerances changes smoothly
   * \param pose_num The pose number for the start of a segment who's tolerances
   * have been modified
   */
  void smoothTolerancesBck(const int &pose_num);

  /** \brief Sets path tracker gains based on scheduled peed. */
  void floorSpeedSchedToDiscreteConfig();

  /**
   * \brief Find the gain schedule closest to the scheduled speed that is less
   * than the scheduled speed.
   * \note this only works because of the way the gain schdules are ordered (see
   * loadConfigFile). The gain schedules are organized in increasing order of
   * their target speed (i.e., from negative to positive) Given a desired speed,
   * this function will pick the gain schedules with the closest speed, but in
   * conservative manner. For example, if the desired speed is 0.45 and there
   * are gain schedules for speeds of 0.25 and 0.5, it will pick the 0.25 speed
   * and gain schedule. If the desired speed is 0.5, then the 0.5 speed and gain
   * schedule will be chosen.
   * \note the reason I am only selecting specific speeds and not linearly
   * interpolating between speeds is for the following two reasons:
   *   (i) the gains have been specifically tuned for these speeds,
   *   (ii) the system seems to respond better to a step input than a ramp input
   * \todo (old) What is this about (i) and (ii) above?!?!?!
   *
   * \param v speed to be rounded
   * \return \todo
   */
  int findClosestSpeed(float v);

  /** \brief Prints a summary of the path for investigation/debugging. */
  void printPath();

  /**
   * \brief Finds the farthest index within angular_window and distance_window
   * of the start.
   * \param path_length distance along the path
   * \param path_turn_angles turn angles
   * \param distance_window max distance allowed
   * \param angular_window max angle allowed
   * \param start starting index
   * \param end end index
   * \param get_future_window whether to search from the beginning or end of the
   * path.
   * \return the window
   */
  int getWindow(const std::vector<double> &path_length,
                const std::vector<double> &path_turn_angles,
                const double &distance_window, const double &angular_window,
                int &start, int &end, const bool get_future_window);

  /**
   * \brief Checks if the pose has passed path vertex pose_i
   * \param pose_i index of the current next vertex
   * \param pose_im1 index of the current previous vertex
   * \param v_des current desired speed
   * \param x_k current position relative to the trunk
   * \param local_path the local path containing the next desired poses in the
   * frame of the trunk.
   */
  void updatePathProgress(int &pose_i, int &pose_im1, const float v_des,
                          const Eigen::VectorXf &x_k,
                          const local_path_t local_path);

  /**
   * \brief Checks if the robot has passed a pose
   * The robot is considered to have passed a vertex if the x-coordinate of the
   * robot in the frame of the vertex has passed a pose.
   * \param v_des the current desired speed
   * \param x_k the current pose in the frame of the trunk
   * \param x_desired the desired pose in the frame of the trunk
   * \return
   */
  bool checkIfPastPose(const float &v_des, const Eigen::VectorXf &x_k,
                       const Eigen::MatrixXf &x_desired);

  /// Helper functions from "old PathUtilities" \todo: check if still needed
  /// \todo (old) PUT SOME OF THESE IN LGMATH OR REMOVE IN FAVOUR OF BETTER
  /// INTERPOLATION

  /**
   * \brief Converts from geometry_msgs::pose to point, quaternion
   * \param[in] pose the pose (input)
   * \param[out] point the point (output)
   * \param[out] quaternion the quaternion (output)
   */
  void geometryPoseToTf(const geometry_msgs::msg::Pose &pose,
                        tf2::Vector3 &point, tf2::Quaternion &quaternion);

  /**
   * \brief Computes the Euclidean distance between two points
   * \param p_0_n_0 point n
   * \param p_0_np1_0 point n+1
   * \param dpMag Euclidean distance between points n and n+1
   */
  void computeDpMag(const tf2::Vector3 &p_0_n_0, const tf2::Vector3 &p_0_np1_0,
                    double &dpMag);

  /**
   * \brief Computes the angular distance between two poses given their roll,
   * pitch, yaw.
   * \param rpy_0_n_0 rpy for pose n
   * \param rpy_0_np1_0 rpy for pose n + 1
   * \param dphiMag the magnitude of the angle between pose n and n+1
   */
  void computeDphiMag(const geometry_msgs::msg::Vector3 &rpy_0_n_0,
                      const geometry_msgs::msg::Vector3 &rpy_0_np1_0,
                      double &dphiMag);

  /**
   * \brief Compute the curvature between two poses.
   * \param angle angle separating the two poses
   * \param dist distance between the two poses
   * \param curvature returned curvature value.
   */
  void computePoseCurvature(const double &angle, const double &dist,
                            double &curvature);

  /** \brief Sets all the fields of current_gain_schedule_ to zero. */
  void clearSpeedAndGainSchedules();

  /// Getters

  /** \brief Gets number of poses in the path */
  int numPoses() const { return num_poses_; }

  /** \brief Gets vertex id given the sequqnce id of a pose */
  Vid vertexID(int seq_id) { return vertex_Id_[seq_id]; }

  void setCurrentGainSchedule(int curr_pose_seq_id) {
    current_gain_schedule_ =
        gain_schedules_[gain_schedule_idx_[curr_pose_seq_id]];
  }

  /** \brief Set current_gain_schedule_ to zero. */
  void clearCurrentGainSchedule();

  /// Taper the speed schedule after resuming from a pause
  void adjustSpeedProfileHoldSpeed(int start, int length);
  void adjustSpeedProfileHoldSpeed(int start, int length, double target_speed);
  void adjustSpeedProfileTaperUp(int start);
};
}  // namespace path_tracker
}  // namespace vtr
