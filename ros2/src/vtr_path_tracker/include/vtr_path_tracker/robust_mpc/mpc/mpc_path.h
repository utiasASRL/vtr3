#pragma once

#include <rclcpp/rclcpp.hpp>

#include <vtr_common/rosutils/transformations.hpp>
#include <vtr_pose_graph/path/localization_chain.hpp>
#include <vtr_pose_graph/path/path.hpp>

#include <fstream>

#include <vtr_path_tracker/robust_mpc/mpc/mpc_types.h> // for VertexTrackingType
#include <vtr_path_tracker/robust_mpc/mpc/utilities.h>

namespace vtr {
namespace path_tracker {

// useful typedef
using Chain = pose_graph::LocalizationChain;
using pose_graph::VertexId;

// The gain schedule struct
typedef struct {
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
} gain_schedule_t;

typedef struct {
  // Flags
  bool flg_allow_turn_on_spot;
  bool flg_slow_start;

  // Schedules
  std::vector<double> speed_schedules;
  std::vector<double> curvature_thresholds;

  // Parameters
  double min_slow_speed_zone_length;
  double max_dx_turnOnSpotMode;
  double max_turn_radius_turnOnSpotMode;
  double default_tight_tracking_error, default_loose_tracking_error, max_tracking_error_rate_of_change;
  double default_heading_constraint;
  double v_max, w_max;
  double max_accel, max_decel;
  double min_speed;
  double v_max_slow;
  double reset_from_pause_slow_speed;
  int reset_from_pause_slow_speed_zone_length_vertices;

  double path_end_x_thresh;
  double path_end_heading_thresh;

} path_params_t;

/// The tolerance limit struct
typedef struct {
  VertexId start_vertex_id;
  VertexId end_vertex_id;
  float new_tolerance_lim;
} tolerance_lim_t;

/// Vector of tolerance_lim_t
typedef std::vector<tolerance_lim_t> tolerance_lim_vec_t;

/*
 *
 *  TODO: Using a std::map for path properties.
 * EXAMPLE FOR USING A MAP INDEXED BY SEQ_ID. Do it this way once finished the first time through.
struct path_data {
  float pose_tol_positive;
};
std::map<uint64_t,path_data> path_data_map;
path_data_map[6] = path_data_map();
path_data_map.insert({6,path_data_map()});
path_data_map[6].pos_tol_positive;
*/


/** @brief Class for storing additional information about the path important for MPC.
*/
class MpcPath {
 public:
  typedef pose_graph::VertexId Vid;

  /** @brief      Constructor
  * @param nh_ptr - pointer to the node handle for the node responsible for this path. Used to get parameters.
  * @return
  */
  MpcPath(const std::shared_ptr<rclcpp::Node> node, std::string param_prefix) : node_(node) {
    param_prefix_ = param_prefix;
    LOG(INFO) << "MPC path using namespace: " << param_prefix_.c_str();
  }

  /** \brief Pointer to node for reference
 */
  const std::shared_ptr<rclcpp::Node> node_;

  /** \brief Parameter namespace. Should be something like "path_tracker/"
 */
  std::string param_prefix_;

  // Raw data, extracted for ease-of-use

  /** \brief The transform from the origin to each vertex in the path. T_vertexI_origin
 */
  std::vector<geometry_msgs::msg::Pose> poses_;

  std::vector<float> poses_tol_positive_;
  std::vector<float> poses_tol_negative_;
  std::vector<float> poses_heading_constraint_pos_;
  std::vector<float> poses_heading_constraint_neg_;
  std::vector<double> turn_radius_;
  std::vector<double> turn_angle_;
  std::vector<double> dist_from_start_;

  /** \brief Distance from point k to point k+1 along the x-axis of frame k
 */
  std::vector<double> dx_;

  /** \brief Whether the vehicle was traveling backwards
 */
  std::vector<bool> travel_backwards_;

  /** \brief Vertex id of each pose along the path
 */
  std::vector<Vid> vertex_Id_;

  // These vectors are of length max(vertexId)
  int largest_vertex_Id_;
  int num_poses_;
  std::unordered_map<Vid, double> dist_by_vertexId_;
  std::unordered_map<Vid, uint64_t> pose_num_by_vertex_id_; // TODO: Is this necessarily unique???

  // Processed data
  std::vector<VertexCtrlType> scheduled_ctrl_mode_;
  std::vector<double> scheduled_speed_;
  std::vector<double> original_scheduled_speed_;

  // Params
  path_params_t params_;
  std::vector<double> list_of_constrained_vertices_from_;
  std::vector<double> list_of_constrained_vertices_to_;

  // Vector of gain schedules.
  std::vector<gain_schedule_t> gain_schedules_;
  std::vector<unsigned int> gain_schedule_idx_;
  gain_schedule_t current_gain_schedule_;

  /** @brief Loads a gain schedule configuration file.
 *
 * @param  config_file_name  The configuration file name
 *
 * @return     { description_of_the_return_value }
 */
  bool loadGainScheduleConfigFile();

  /** @brief Loads a curvature configuration file.
 *
 * @param  config_file_name string for file name. e.g. ...gains/asrl_grizzly/following_gains/curvature_thresholds.yaml
 *
 * @return     success
 */
  bool loadCurvatureConfigFile();

/** @brief Loads parameters related to tracking error and speed/acceleration constraints for speed scheduling
 *
 * @return success
 */
  bool loadPathParams();

  /** @brief Load parameters from ROS and config files.
 *      Loads gain schedule, curvature config, and path parameters.
 * @return success
 */
  bool getConfigs(); // get all configuration parameters using the three methods above

  /**
 * @brief Extract additional information important for speed scheduling from the path
 *
 * Given the localization chain which contains the path to be driven, extract the path curvature, turn angle, turn radius, distance from start, and largest vertex ID.
 * @param  chain  The localization chain
 *  \todo: (old) remove all conversions from tf/lgmath/geometry_msgs and just use lgmath. This will require some additions to lgmath.
 */
  void extractPathInformation(const std::shared_ptr<Chain> &chain);

  /** @brief Set current_gain_schedule_ to zero.
 */
  void clearCurrentGainSchedule();

  /** @brief Gets the speed profile based on XXX
 *
 *  This is the same as Chris O's speed profiler but does not use experience to set a suggested speed.
 *
 * @param chain  Pointer to the localization chain.
 */
  void getSpeedProfile();

  // Helper functions for speed scheduling
  /** @brief Sets the initial path modes to TURN_ON_SPOT, START, NORMAL, DIR_SW_POSE, END based on thresholds in params_.
 */
  void setInitialPathModes();

  /** @brief Find false positive direction switches.
 *
 *     Good direction switches have enough travel_backwards ahead of them and travel forwards behind them.
 *     \todo: (old) Make parameters configurable.
 */
  void findFalsePositiveDirSwPoses();

  /** @brief Smooth the curvature estimate for the desired path
 */
  void smoothCurvature();

  /** @brief Expand DIR_SW and END path modes in scheduled_ctrl_mode_.
 *
 *             1. Set scheduled_ctrl_mode_ around DIR_SW_POSE to DIR_SW_REGION
 *             2. Set scheduled_ctrl_mode_ to END for poses within params_.min_slow_speed_zone_length of the end of the path.
 */
  void expandDirSwAndEndModes();

  /** @brief      Assign speed profile and tracking error tolerances
 *
 * \todo: (old) The speed schedule was set based on experience and curvature. We should be able to set the desired speed to a max and let the safe learning figure out the rest.
 */
  void assignSpeedProfileAndTrackingtolerance();

  /** @brief Smooth the scheduled speed based on max allowed acceleration and nearby scheduled speeds.
 */
  void smoothScheduledSpeed();

  /** @brief Modify the tracking constraints for vertices specified by the user using the list_of_constrained_vertices_(from/to) parameter
 */
  void processConstrainedVertices();

  /** @brief Adjust tracking tolerance for user segments
 *
 * @param new_limits_list
 */
  void adjustToleranceLimits(const tolerance_lim_vec_t &new_limits_list);

  /** @brief Make sure path tracking tolerances changes smoothly
 *
 * @param  pose_num  The pose number for the end of a segment who's tolerances have been modified
 */
  void smoothTolerancesFwd(const int &pose_num);

/** @brief Make sure path tracking tolerances changes smoothly
 *
 * @param pose_num  The pose number for the start of a segment who's tolerances have been modified
 */
  void smoothTolerancesBck(const int &pose_num);

  /** @brief MpcPath::floorSpeedSchedToDiscreteConfig
 * \todo: (old) Does nothing at the moment... Remove if unnecessary
 */
  void floorSpeedSchedToDiscreteConfig();

/** @brief Find the gain schedule closest to the scheduled speed that is less than the scheduled speed.
 *
 *  Note: this only works because of the way the gain schdules are ordered (see loadConfigFile)
 *  The gain schedules are organized in increasing order of their target speed (i.e., from negative to positive)
 *  Given a desired speed, this function will pick the gain schedules with the closest speed, but in conservative
 *  manner. For example, if the desired speed is 0.45 and there are gain schedules for speeds of 0.25 and 0.5, it
 *  will pick the 0.25 speed and gain schedule. If the desired speed is 0.5, then the 0.5 speed and gain schedule
 *  will be chosen. NOTE: the reason I am only selecting specific speeds and not linearly interpolating between
 *  speeds is for the following two reasons: (i) the gains have been specifically tuned for these speeds,
 *  (ii) the system seems to respond better to a step input than a ramp input
 *
 *  \todo: (old) What is this about (i) and (ii) above?!?!?!
 *
 * @param  v    the speed to be rounded
 *
 * @return     { description_of_the_return_value }
 */
  int findClosestSpeed(float v);

  /** @brief Print a summary of the path preprocessing
 */
  void printPreprocessingResults();

  /** @brief Find the farthest index within angular_window and distance_window of the start.
 *
 * @param  pathLength       Distance along the path
 * @param  pathTurnAngles   Turn angles
 * @param  distance_window  Max distance allowed
 * @param  angular_window   max angle allowed
 * @param      start            Starting index
 * @param      end              End index
 * @param  getFutureWindow  Whether to search from the beginning or end of the path.
 *
 * @return     The window.
 */
  int getWindow(const std::vector<double> &path_length,
                const std::vector<double> &path_turn_angles,
                const double &distance_window,
                const double &angular_window,
                int &start,
                int &end,
                const bool get_future_window);

  /** @brief Check if the pose has passed path vertex pose_i
 *
 * @param pose_i: index of the current next vertex
 * @param pose_im1: index of the current previous vertex
 * @param v_des: current desired speed
 * @param x_k: current position relative to the trunk
 * @param local_path: the local path containing the next desired poses in the frame of the trunk.
 */
  void updatePathProgress(int &pose_i, int &pose_im1,
                          const float v_des,
                          const Eigen::VectorXf &x_k,
                          const local_path_t local_path);

  /** @brief Checks if the robot has passed a pose
 *
 * @param v_des: the current desired speed
 * @param x_k: The current pose in the frame of the trunk
 * @param x_desired: the desired pose in the frame of the trunk
 *
 * The robot is considered to have passed a vertex if the x-coordinate of the robot in the frame
 * of the vertex has passed a pose.
 * @return
 */
  bool checkIfPastPose(const float &v_des,
                       const Eigen::VectorXf &x_k,
                       const Eigen::MatrixXf &x_desired);

  /// Helper functions from "old PathUtilities" \todo: check if these still needed
  /// \todo: (old) PUT SOME OF THESE IN LGMATH OR REMOVE IN FAVOUR OF BETTER INTERPOLATION

  /** @brief Convert from geometry_msgs::pose to point, quaternion
 *
 * @param  pose        The pose (input)
 * @param  point       The point (output)
 * @param  quaternion  The quaternion (output)
 */
  void geometryPoseToTf(const geometry_msgs::msg::Pose &pose, tf2::Vector3 &point, tf2::Quaternion &quaternion);

  /** @brief MpcPath::compute_dpMag compute the Euclidean distance between two points
 *
 * @param p_0_n_0: Point n
 * @param p_0_np1_0: point n+1
 * @param dp_mag: Euclidean distance between points n and n+1
 */
  void computeDpMag(const tf2::Vector3 &p_0_n_0, const tf2::Vector3 &p_0_np1_0, double &dpMag);

  /** @brief MpcPath::compute_dphiMag: compute the angular distance between two poses given their roll, pitch, yaw
   *
 * @param rpy_0_n_0: rpy for pose n
 * @param rpy_0_np1_0: rpy for pose n + 1
 * @param dphi_mag: the magnitude of the angle between pose n and n+1
 */
  void computeDphiMag(const geometry_msgs::msg::Vector3 &rpy_0_n_0,
                      const geometry_msgs::msg::Vector3 &rpy_0_np1_0,
                      double &dphiMag);

/** @brief MpcPath::compute_pose_curvature: Compute the curvature between two poses.
 *
 * @param angle: angle separating the two poses
 * @param dist: distance between the two poses
 * @param curvature: returned curvature value.
 */
  void computePoseCurvature(const double &angle, const double &dist, double &curvature);

/** @brief Sets all the fields of current_gain_schedule_ to zero.
 */
  void clearSpeedAndGainSchedules();

  // Getters
  /// Get number of poses in the path
  inline int numPoses() const { return num_poses_; }

  /// Get vertex id given the sequqnce id of a pose
  inline Vid vertexID(int seq_id) { return vertex_Id_[seq_id]; }

  inline void setCurrentGainSchedule(int curr_pose_seq_id) {
    current_gain_schedule_ = gain_schedules_[gain_schedule_idx_[curr_pose_seq_id]];
  }

  /// Taper the speed schedule after resuming from a pause
  void adjustSpeedProfileHoldSpeed(int start, int length);
  void adjustSpeedProfileHoldSpeed(int start, int length, double target_speed);
  void adjustSpeedProfileTaperUp(int start);

  /**
 * @brief      Print a summary of the path. For debugging
 */
  void printPath();
};
} // path_tracker
} // vtr
