#pragma once

/*
File:
Edited By:      Chris Ostafew
Date:           Aug 11, 2014

Purpose:        To do

Functions:      To do
*/


// Standard C++ includes
#define _USE_MATH_DEFINES // for M_PI
#include <cmath>

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

#include <asrl/pose_graph/id/VertexId.hpp>
#include <asrl/common/logging.hpp>

//#include <asrl/path_tracker_mpc/asrl_sparse_mtx.hpp>

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>

namespace vtr {
namespace path_tracker {

using asrl::pose_graph::VertexId;


// Indices for gp dependencies
const int DIST_ALONG_PATH       = 0;
const int OMEGA_ACT_KM1         = 1;
const int V_ACT_KM1             = 2;
const int OMEGA_CMD_KM1         = 3;
const int V_CMD_KM1             = 4;
const int OMEGA_CMD_K           = 5;
const int V_CMD_K               = 6;
const int HEAD_ERROR_K          = 7;
const int LAT_ERROR_K           = 8;

const int CONTROL_SIZE = 1;
const int STATE_SIZE = 3;
const bool MODEL_INCLUDES_VELOCITY = false;
const bool MODEL_DEP_UKM1 = false;
const int ERROR_SIZE = STATE_SIZE - 1;
const int DIST_DEP_SIZE = 9;

const int POSE_SIZE = 3;
const int VELOCITY_SIZE = 2;

const float K_OMEGA = 0.6;

class mtx_triplet {
public:
  mtx_triplet(int i_in, int j_in, float v_ij_in);

  int i, j;
  float v_ij;
};

void adjust_mtx_triplet_row_col(mtx_triplet & mtx_triplet, int & i_in, int & j_in);

void scalar_mult_mtx_triplet(mtx_triplet & mtx_triplet, float & scalar_p);

typedef std::vector< mtx_triplet > mtx_triplet_list_t;
typedef std::vector< mtx_triplet_list_t > vector_valued_hessian_t;

class MpcNominalModel
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructor
  MpcNominalModel();

  // Destructor
  ~MpcNominalModel();

  /** Define struct to hold state x_k **/
  typedef struct {
    // This struct is meant to be used by the mpc algorithm

    // Main variables, these variables must be present for the mpc algorithm
    Eigen::VectorXf x_k;        // x_kp1 = f(x_k, u_k) + g(a_k)
    std::vector<Eigen::VectorXf> x_k_wc;
    Eigen::MatrixXf var_x_k;    // variance of x_k, used for robust mpc
    Eigen::MatrixXf dvar_x_dv;  // derivative of variance w.r.t. v
    Eigen::MatrixXf dvar_x_dth; // derivative of variance w.r.t. th
    float lateral_uncertainty;
    float heading_uncertainty;

    Eigen::VectorXf g_a_k;        // Computed / Measured disturbance, in the robot frame
    Eigen::MatrixXf var_g_a_k;    // Variance of disturbance, matrix enables correlated disturbances, in robot frame

    Eigen::VectorXf g_a_k_des_frame;
    Eigen::MatrixXf var_g_a_k_des_frame;

    // Additional variables, these variables will depend on your implementation
    // Tracking errors,
    float dist_along_path_k;
    Eigen::VectorXf tracking_error_k;
    Eigen::VectorXf tracking_error_km1;
    Eigen::VectorXf tracking_error_k_interp;

    // Velocities and Commands
    Eigen::VectorXf velocity_km1;
    //Eigen::VectorXf velocity_km1_filt;
    Eigen::VectorXf command_k;
    Eigen::VectorXf command_km1;

    // Derivatives
    Eigen::MatrixXf df_dxk, df_dxkm1, df_duk, df_dukm1, df_dvk, df_dvkm1;
    Eigen::MatrixXf dg_dxk, dg_dxkm1, dg_duk, dg_dukm1, dg_dvk, dg_dvkm1;
    Eigen::MatrixXf dg_da;

    // Gradients
    Eigen::MatrixXf grad_x, grad_xkm1, grad_u, grad_ukm1, grad_v, grad_vkm1;

    // Hessians (Jacobians of Gradients)
    Eigen::MatrixXf Jv_gx_mtx;
    vector_valued_hessian_t Jx_gx, Ju_gx, Jv_gx,
    Jx_gu, Ju_gu, Jv_gu,
    Jx_gv, Ju_gv, Jv_gv;

    bool flg_hessians_cleared;

  } model_state_t;

  typedef std::vector<model_state_t> model_trajectory_t;

  /** Define struct to hold GP data points (for fast computation) **/
  typedef struct {
    Eigen::VectorXf x_meas;
    Eigen::VectorXf g_x_meas;
  } gp_data_t;

  /** Define struct to hold an "experience" **/
  typedef struct {

    // State variables
    VertexId at_vertex_id;
    VertexId to_vertex_id;

    ros::Time transform_time;
    ros::Time store_time;
    ros::Duration transform_delay;

    model_state_t x_k;

    float tol_posL, tol_posH;

    float path_curvature;
    float dist_from_vertex;

    Eigen::VectorXf velocity_k;
    bool velocity_is_valid;
    bool disturbance_is_valid;

    Eigen::Matrix<double,6,1> full_velocity_k;

    // Variables prepped for GP
    gp_data_t gp_data;

    // Recorded variables for post-processing
    tf::Transform T_0_v;
    float predicted_wc_lateral_error;
    float predicted_wc_heading_error;
    float v_k_odo;
    float w_k_odo;
    int16_t keypoint_match_count;
    int worst_case_scenario;
    bool mpc_valid;

    std::vector<float> J_iter;
    std::vector<float> u_iter;

    double t_mpc;
    double t_robust;

    bool flg_do_not_delete;

  } experience_t;


  /**  Define mpc nominal model (including first derivatives)
   *   Note: x_kp1 = f(x_k,u_k) + g(a), where "a" is the disturbance dependency
   */

  // Motion Model for the MPC algorithm, using linearization to propagate uncertainty
  void f_x_linearizedUncertainty(const model_state_t & x_k, model_state_t & x_kp1, float dt);
  void f_x_linearizedUncertaintyV2(const model_state_t & x_k, model_state_t & x_kp1, float dt);
  // Motion Model for the MPC algorithm, using the unscented transform to propagate uncertainty
  bool f_x_unscentedUncertainty(const model_state_t & x_k, model_state_t & x_kp1, float dt);
  // Compute nominal model Jacobians
  void get_dF_dx(const model_state_t & x_k, Eigen::MatrixXf & dF_dx, float d_t);
  void get_dF_du(const model_state_t &, Eigen::MatrixXf & dF_du, Eigen::MatrixXf & dF_dukm1, float d_t);
  void get_dF_dv(const model_state_t & x_k, Eigen::MatrixXf & dF_dv, Eigen::MatrixXf & dF_dvkm1, float d_t);
  void get_gdot(model_state_t & x_k, float d_t);
  void get_Jdot_gdot(model_state_t & x_k, float d_t);
  void get_derivative_of_variance(model_state_t & x_k, float d_t);
  // Given dg(a)/da, use da/dx to compute dg(a)/dx
  void compute_dg_dx_and_dg_dxkm1   (Eigen::MatrixXf & dG_dx, Eigen::MatrixXf & dG_dxkm1, const Eigen::MatrixXf & dg_da, const float & th_des, const float & d_t);
  void compute_dg_du_and_dg_dukm1   (Eigen::MatrixXf & dG_du, Eigen::MatrixXf & dG_dukm1, const Eigen::MatrixXf & dg_da, const float & d_t);

  /**
     Define functions related to predicting pose sequences
      **/

  // Generate worst-case trajectories
  //bool generate_worstCase_trajectories(const model_trajectory_t & x_sequence, std::vector<model_trajectory_t> & worst_case_trajectories, const float & d_t, const double & robust_control_sigma);
  bool generateWorstCaseTrajectories(model_trajectory_t & x_sequence, const double & robust_control_sigma);
  // Check if robot has passed desired state in sequence
  bool robot_has_passed_desired_poseNew(const float & v_des, const Eigen::VectorXf & x_k, const Eigen::MatrixXf & x_desired);
  // Compute the distance along the path
  float computeDistAlongPath(const float & nearest_path_length, const float & e_x, const float & linear_speed);

  /**
       Define functions related to computing tracking errors
        **/

  // Given two desired poses and an actual pose, compute an interpolated desired pose
  void computeInterpolatedDesiredPoseNew(const Eigen::MatrixXf & x_des_im1, const Eigen::MatrixXf & x_des_i, vtr::path_tracker::MpcNominalModel::model_state_t & x_pred, Eigen::MatrixXf & x_des_interp);

  // Compute errors for a pose or sequence of poses
  void compute_sequence_errors(model_trajectory_t & x_sequence, const Eigen::MatrixXf & x_desired);
  Eigen::VectorXf compute_pose_errors(model_state_t & x_state, const Eigen::MatrixXf & x_desired);
  Eigen::VectorXf compute_pose_errors(model_state_t & x_state, const Eigen::MatrixXf & x_desired, const tf::Point & p_0_k_0, const tf::Transform & C_0_k);
  Eigen::VectorXf compute_pose_errorsNew(const model_state_t & x_state, const Eigen::MatrixXf & x_desired);
  Eigen::VectorXf compute_pose_errorsNew(const model_state_t & x_state, const Eigen::MatrixXf & x_desired, const tf::Point & p_0_k_0, const tf::Transform & C_0_k);
  // Extract errors for a sequence of poses
  void extract_pose_errors(Eigen::MatrixXf & traj_errors, const model_trajectory_t & x_sequence);
  // Convert pose errors to heading and lateral errors
  Eigen::MatrixXf compute_pose_to_hl_error_conversion_mtx(float & th_des);

  /**
       Define functions related to computing disturbances
        **/

  // Compute disturbances based on two sequential poses
  bool computeDisturbancesForExperienceKm2(experience_t & experience_km1, const experience_t & experience_k);
  bool computeDisturbancesForExperienceKm2SteamVel(experience_t & experience_km2, const experience_t & experience_km1);

  bool compute_disturbance_from_state(Eigen::VectorXf & g_a_k_meas, const model_state_t & state_km1, const model_state_t & state_k, const float & d_t);
  // Compute velocities based on two sequential poses
  void computeVelocitiesForExperienceKm1(const experience_t & experience_km2, experience_t & experience_km1, experience_t & experience_k);
  void compute_velocities_from_state(Eigen::VectorXf & velocity, const Eigen::VectorXf & state_km1, const Eigen::VectorXf & state_k_act, const float dt);
  // Simple functions and initializations
  void initialize_experience(experience_t & experience_k);
  void initialize_state(model_state_t & x_k);
  void computeDisturbanceDependancy(model_state_t & x_k,
                                      const model_state_t & x_km1,
                                      const Eigen::MatrixXf & x_des,
                                      const float & nearest_path_length,
                                      float & d_t);
  void extract_disturbance_dependencies(const model_state_t & x_input, Eigen::VectorXf & x_test);
  void set_disturbance_model_zero(model_state_t & x_input);

  /**
       Extras
        **/
  // Extract the translation from a pose
  void getTfPoint(const geometry_msgs::Pose_<std::allocator<void> >& pose, tf::Point& point);
  // Extract the quaternion from a pose
  void getTfQuaternion(const geometry_msgs::Pose_<std::allocator<void> >& pose, tf::Quaternion& q);
  // Ensure theta is between -pi and pi
  float thetaWrap(float th_in);

};

} // path_tracker
} // asrl

