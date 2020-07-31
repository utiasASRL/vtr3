#pragma once


/*
 * Author: Chris McKinnon
 * Email: chris.mckinnon@robotics.utias.utoronto.ca
 */

/**
 * @file
 */

// This builder is based on ros
#pragma GCC diagnostic ignored "-pedantic"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wunused-parameter"

// ROS includes
#include <ros/ros.h>

#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

// ASRL includes
#include <asrl/common/rosutil/transformation_utilities.hpp>
#include <asrl/common/rosutil/node_utilities.hpp>
#include <asrl/pose_graph/path/LocalizationChain.hpp>
#include <asrl/pose_graph/path/Path.hpp>

// for path interpolation. Replace eventually
#include <asrl/common/rosutil/transformation_utilities.hpp>
#include <angles/angles.h>

// Other includes
#include <fstream>
#include <yaml-cpp/yaml.h>

// From this project
#include <vtr/path_tracker/robust_mpc/mpc/mpc_types.h> // for VertexTrackingType
#include <vtr/path_tracker/robust_mpc/mpc/utilities.h>


namespace vtr {
namespace path_tracker {

// useful typedef
typedef asrl::pose_graph::LocalizationChain Chain;
using asrl::pose_graph::VertexId;


// The gainschedule struct
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

typedef struct{
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
  double reset_from_pause_slow_speed_zone_length_vertices;

  double path_end_x_thresh;
  double path_end_heading_thresh;

} path_params_t;



/// The tolerance limit struct
typedef struct{
  VertexId start_vertex_id;
  VertexId end_vertex_id;
  float new_tolerance_lim;
} tolerance_lim_t;
/// Vector of tolerance_lim_t
typedef  std::vector<tolerance_lim_t> tolerance_lim_vec_t;


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


/**
* @brief      Class for storing additional information about the path important for MPC.
*/
class MpcPath
{
public:
  typedef ::asrl::pose_graph::VertexId Vid;

  /**
  * @brief      Constructor
  * @param nh_ptr - pointer to the node handle for the node responsible for this path. Used to get parameters.
  * @return
  */
  MpcPath(ros::NodeHandle& nh, std::string param_prefix) : nh_(nh){

    param_prefix_ = param_prefix;
    LOG(INFO) << "MPC path using namespace: " << param_prefix_.c_str();

  }

  // Pointer to node handle for reference
  ros::NodeHandle& nh_;
  std::string param_prefix_; ///< parameter namespace. Should be something like "path_tracker/"

  // Raw data, extracted for ease-of-use
  std::vector<geometry_msgs::Pose> poses_; ///< The transform from the origin to each vertex in the path. T_vertexI_origin
  std::vector<float> poses_tol_positive_;
  std::vector<float> poses_tol_negative_;
  std::vector<float> poses_heading_constraint_pos_;
  std::vector<float> poses_heading_constraint_neg_;
  std::vector<double> turn_radius_;
  std::vector<double> turn_angle_;
  std::vector<double> dist_from_start_;
  std::vector<double> dx_;               ///< Distance from point k to point k+1 along the x-axis of frame k
  std::vector<bool> travel_backwards_;   ///< Whether the vehicle was traveling backwards
  std::vector<Vid> vertex_Id_;      ///< Vertex id of each pose along the path

  // These vectors are of length max(vertexId)
  int largest_vertex_Id_;
  int num_poses_;
  boost::unordered_map<Vid, double> dist_by_vertexId_;
  boost::unordered_map<Vid, uint64_t> pose_num_by_vertex_id_; // TODO: Is this necessarily unique???

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




  /// Loads a gain schedule configuration file.
  bool loadGainScheduleConfigFile(std::string config_file_name);
  /// Loads a curvature configuration file.
  bool loadCurvatureConfigFile(std::string config_file_name);
  /// Loads parameters related to tracking error and speed/acceleration constraints for speed scheduling
  bool loadPathParams();
  /// Load parameters from ROS and config files.
  bool getConfigs(); // get all configuration parameters using the three methods above


  /// for loading local path object with poses from desired path.
  void extractPathInformation(const std::shared_ptr<Chain> & chain);
  void clearCurrentGainSchedule();

  /// Main function for speed scheduling:
  void getSpeedProfile(); // main function

  // Helper functions for speed scheduling
  void setInitialPathModes();
  void findFalsePositiveDirSwPoses();
  void smoothCurvature();
  void expandDirSwAndEndModes();
  void assignSpeedProfileAndTrackingtolerance();
  void smoothScheduledSpeed();

  // set path tracking constraints for user specified nodes.
  void processConstrainedVertices();
  void adjustToleranceLimits(const tolerance_lim_vec_t & new_limits_list);
  void adjustToleranceLimits(const int & start_vertexId, const int & end_vertex_id, const float & new_tol_limit);
  void smoothTolerancesFwd(const int & pose_num);
  void smoothTolerancesBck(const int & pose_num);

  // round (floor) scheduled speed to given speed profile values.
  void floorSpeedSchedToDiscreteConfig();
  int findClosestSpeed(float v);

  void printPreprocessingResults();

  /// secondary helper functions for speed scheduling
  int getWindow(const std::vector<double> & path_length,
                const std::vector<double> & path_turn_angles,
                const double & distance_window,
                const double & angular_window,
                int & start,
                int & end,
                const bool get_future_window);

  // For checking progress along the path
  void updatePathProgress(int &pose_i, int &pose_im1,
                          const float v_des,
                          const Eigen::VectorXf &x_k,
                          const local_path_t local_path);
  bool checkIfPastPose(const float & v_des,
                       const Eigen::VectorXf &x_k,
                       const Eigen::MatrixXf &x_desired);

  // helper functions.
  // TODO: PUT SOME OF THESE IN LGMATH OR REMOVE IN FAVOUR OF BETTER INTERPOLATION
  void geometryPoseToTf(const geometry_msgs::Pose & pose, tf::Point & point, tf::Quaternion & quaternion);
  void computeDpMag(const tf::Point & p_0_n_0, const tf::Point & p_0_np1_0, double & dpMag);
  void computeDphiMag(const geometry_msgs::Vector3 & rpy_0_n_0, const geometry_msgs::Vector3 & rpy_0_np1_0, double & dphiMag);
  void computePoseCurvature(const double & angle, const double & dist, double & curvature);

  // Setters
  void clearSpeedAndGainSchedules();

  // Getters
  /// Number of poses in the path
  inline int numPoses() const { return num_poses_; }

  /// vertex id given the sequqnce id of a pose
  inline Vid vertexID(int seq_id) { return vertex_Id_[seq_id]; }

  inline void setCurrentGainSchedule(int curr_pose_seq_id) {
    current_gain_schedule_ = gain_schedules_[gain_schedule_idx_[curr_pose_seq_id]];
  }

  /// Taper the speed schedule after resuming from a pause
  void adjustSpeedProfileHoldSpeed(int start, int length);
  void adjustSpeedProfileHoldSpeed(int start, int length, double target_speed);
  void adjustSpeedProfileTaperUp(int start);

  /// print something about the path. For debugging.
  void printPath();
};
}
}
