#pragma once

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_pose_graph/id/graph_id.hpp>
#include <Eigen/Core>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace vtr {
namespace path_tracker {

/** \brief Struct for MPC parameters
 */
typedef struct {

  // solver parameters
  double init_step_size;
  int max_solver_iterations;

  // enable flags
  bool
      flg_en_time_delay_compensation; /// compensate for delay between localization and when the control will be applied
  bool flg_allow_ctrl_to_dir_sw; /// Special controller for direction switch
  bool flg_allow_ctrl_tos; /// Special controller for turn on the spot
  bool flg_allow_ctrl_to_end; /// another special controller for the end
  bool flg_use_steam_velocity; /// use velocity estimate from steam instead of finite difference
  bool flg_use_vtr2_covariance; /// use covariance from VTR2 instead of hard coded constant
  bool flg_enable_fudge_block; /// Use block to place hard limit speed as a function of turn rate (i.e. slow corners)
  bool
      flg_use_fixed_ctrl_rate; /// Run the controller at a constant rate instead of with a fixed sleep time between control steps
  bool
      flg_enable_varied_pred_step; /// Use longer time-step for later steps in the prediction horizon to extend the look-ahead

  // params for robust control
  double default_xy_disturbance_uncertainty;
  double default_theta_disturbance_uncertainty;
  double robust_control_sigma;
  int max_lookahead;
  double look_ahead_step_ms;
  double control_delay_ms;

  // params for checking if the path is finished
  double path_end_x_threshold;
  double path_end_heading_threshold;

  // params for flattening the local path around the vehicle
  int local_path_poses_forward;
  int local_path_poses_back;
  unsigned
      num_poses_end_check; ///< Number of vertices to the end before the path tracker will start checking if the path is done

  // params for adding artificial disturbances
  double Kv_artificial; // The speed command actually sent to the vehicle is Kv * velocity command
  double Kw_artificial; // The turn rate command actually sent to the vehicle is Kw * turn rate command

  double extrapolate_timeout;  // time [s] to trust Steam trajectory since last update
} mpc_params_t;

typedef struct {
  Eigen::MatrixXf x_des_fwd, x_des_bck,
      x_des_interp; ///< x_des_fwd/bk is the transform from pose k to k+1 expressed in frame k.
  Eigen::MatrixXf x_lb, x_ub, x_lb_interp, x_ub_interp; ///< Lateral and Heading Error Constraints
  Eigen::VectorXf x_act; ///< current pose [x, y, theta]
  Eigen::MatrixXf x_act_cov; ///< uncertainty in the current pose estimate
  tf2::Transform T_0_v; ///< T_0_v is the transform from the vehicle frame to the root
  common::timing::time_point transform_stamp;
  int current_pose_num;
  pose_graph::VertexId current_vertex_id;
  pose_graph::VertexId next_vertex_id;
} local_path_t;

/**
 * @brief The VertexTrackingType enum: indicates the type of vertex. Used to handle control for special cases.
 */
enum class VertexCtrlType {
  START,
  END,
  DIR_SW_REGION,
  DIR_SW_POSE,
  TURN_ON_SPOT,
  NORMAL
};

} // path_tracker
} // vtr
