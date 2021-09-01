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
 * \file mpc_nominal_model.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#define _USE_MATH_DEFINES  // for M_PI
#include <cmath>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/rclcpp.hpp>

#include <vtr_logging/logging.hpp>
#include <vtr_pose_graph/id/graph_id.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace vtr {
namespace path_tracker {

using pose_graph::VertexId;

// Indices for gp dependencies
const int DIST_ALONG_PATH = 0;
const int OMEGA_ACT_KM1 = 1;
const int V_ACT_KM1 = 2;
const int OMEGA_CMD_KM1 = 3;
const int V_CMD_KM1 = 4;
const int OMEGA_CMD_K = 5;
const int V_CMD_K = 6;
const int HEAD_ERROR_K = 7;
const int LAT_ERROR_K = 8;

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
  mtx_triplet(int i_in, int j_in, float v_ij_in)
      : i(i_in), j(j_in), v_ij(v_ij_in) {}

  int i, j;
  float v_ij;
};

void adjust_mtx_triplet_row_col(mtx_triplet &mtx_triplet, int &i_in, int &j_in);

void scalar_mult_mtx_triplet(mtx_triplet &mtx_triplet, float &scalar_p);

typedef std::vector<mtx_triplet> mtx_triplet_list_t;
typedef std::vector<mtx_triplet_list_t> vector_valued_hessian_t;

/**
 * \brief MPC nominal model (including first derivatives)
 * \note x_kp1 = f(x_k,u_k) + g(a), where "a" is the disturbance dependency
 */
class MpcNominalModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor, do nothing */
  MpcNominalModel() = default;

  /** \brief Destructor, do nothing */
  ~MpcNominalModel() = default;

  /**
   * \brief Struct to hold state x_k
   * This struct is meant to be used by the MPC algorithm
   */
  typedef struct {
    // Main variables, these variables must be present for the mpc algorithm
    Eigen::VectorXf x_k;  /// x_kp1 = f(x_k, u_k) + g(a_k)
    std::vector<Eigen::VectorXf> x_k_wc;
    Eigen::MatrixXf var_x_k;     ///< variance of x_k, used for robust mpc
    Eigen::MatrixXf dvar_x_dv;   ///< derivative of variance w.r.t. v
    Eigen::MatrixXf dvar_x_dth;  ///< derivative of variance w.r.t. th
    float lateral_uncertainty;
    float heading_uncertainty;

    Eigen::VectorXf
        g_a_k;  ///< Computed / Measured disturbance, in the robot frame
    Eigen::MatrixXf var_g_a_k;  ///< Variance of disturbance, matrix enables
                                ///< correlated disturbances, in robot frame

    Eigen::VectorXf g_a_k_des_frame;  /// \todo: add documentation for these
    Eigen::MatrixXf var_g_a_k_des_frame;

    // Additional variables, these variables will depend on your implementation
    // Tracking errors,
    float dist_along_path_k;
    Eigen::VectorXf tracking_error_k;
    Eigen::VectorXf tracking_error_km1;
    Eigen::VectorXf tracking_error_k_interp;

    // Velocities and Commands
    Eigen::VectorXf velocity_km1;
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
    vector_valued_hessian_t Jx_gx, Ju_gx, Jv_gx, Jx_gu, Ju_gu, Jv_gu, Jx_gv,
        Ju_gv, Jv_gv;

    bool flg_hessians_cleared;

  } model_state_t;

  typedef std::vector<model_state_t> model_trajectory_t;

  /** Define struct to hold GP data points (for fast computation) **/
  typedef struct {
    Eigen::VectorXf x_meas;
    Eigen::VectorXf g_x_meas;
  } gp_data_t;

  /** \brief Struct to hold an "experience" */
  typedef struct {
    // State variables
    VertexId at_vertex_id;
    VertexId to_vertex_id;

    rclcpp::Time transform_time;
    rclcpp::Time store_time;

    model_state_t x_k;

    float tol_posL, tol_posH;

    float path_curvature;
    float dist_from_vertex;

    Eigen::VectorXf velocity_k;
    bool velocity_is_valid;
    bool disturbance_is_valid;

    Eigen::Matrix<double, 6, 1> full_velocity_k;

    // Variables prepped for GP
    gp_data_t gp_data;

    // Recorded variables for post-processing
    tf2::Transform T_0_v;
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

  /** \brief Motion Model for the MPC algorithm, using linearization to
   * propagate uncertainty */
  void f_x_linearizedUncertainty(const model_state_t &x_k, model_state_t &x_kp1,
                                 float dt);

  /**
   * \brief Motion Model for the MPC algorithm, using the unscented transform to
   * propagate uncertainty This function is written to function regardless of
   * the system definition It assumes x_kp1 = f(x_k,u_k) + g_a_k with uncertain
   * x_k and g_a_k. x_k and g_a_k must be of the same length. It relies on the
   * proper definition of f_x_linearizedUncertainty()
   */
  bool f_x_unscentedUncertainty(const model_state_t &x_k, model_state_t &x_kp1,
                                float dt);

  // Compute nominal model Jacobians

  void get_gdot(model_state_t &x_k, float d_t);

  void get_Jdot_gdot(model_state_t &x_k, float d_t);

  /// Define functions related to predicting pose sequences

  /** \brief Given a nominal sequence of states with uncertainty, generate worst
   * case trajectories */
  bool generateWorstCaseTrajectories(model_trajectory_t &x_sequence,
                                     const double &robust_control_sigma);

  /**
   * \brief Check if robot has passed desired state in sequence
   * \param v_des: the current desired speed
   * \param x_k: The current pose in the frame of the trunk
   * \param x_desired: the desired pose in the frame of the trunk
   *
   * The robot is considered to have passed a vertex if the x-coordinate of the
   * robot in the frame of the vertex has passed a pose. \return
   */
  bool robot_has_passed_desired_poseNew(const float &v_des,
                                        const Eigen::VectorXf &x_k,
                                        const Eigen::MatrixXf &x_desired);

  /** \brief Compute the distance along the path */
  float computeDistAlongPath(const float &nearest_path_length, const float &e_x,
                             const float &linear_speed);

  /// Define functions related to computing tracking errors

  /**
   * \brief Finds the closest point x_des_interp to x_pred along the line
   * connecting x_des_im1 and x_des_i \param x_des_im1: the desired point behind
   * the robot \param x_des_i: the desired point ahead of the robot \param
   * x_pred: The robots predicted pose \param x_des_interp: the resulting
   * interpolated path waypoint
   */
  void computeInterpolatedDesiredPoseNew(
      const Eigen::MatrixXf &x_des_im1, const Eigen::MatrixXf &x_des_i,
      vtr::path_tracker::MpcNominalModel::model_state_t &x_pred,
      Eigen::MatrixXf &x_des_interp);

  // Compute errors for a pose or sequence of poses
  void compute_sequence_errors(model_trajectory_t &x_sequence,
                               const Eigen::MatrixXf &x_desired);
  Eigen::VectorXf compute_pose_errors(model_state_t &x_state,
                                      const Eigen::MatrixXf &x_desired);
  Eigen::VectorXf compute_pose_errors(model_state_t &x_state,
                                      const Eigen::MatrixXf &x_desired,
                                      const tf2::Vector3 &p_0_k_0,
                                      const tf2::Transform &C_0_k);

  /**
   * \brief MpcNominalModel::compute_pose_errorsNew
   * \param x_state: The state of the robot.
   * \param x_desired: The path vertex (can be i (ahead of the robot) or i-1
   * (behind the robot) \return the error (x,y,theta) between the desired pose
   * and the robot in the frame of the desired pose.
   */
  Eigen::VectorXf compute_pose_errorsNew(const model_state_t &x_state,
                                         const Eigen::MatrixXf &x_desired);

  /** \brief Compute pose errors. Called by compute_pose_errorsNew(with two
   * arguments) */
  Eigen::VectorXf compute_pose_errorsNew(const model_state_t &x_state,
                                         const Eigen::MatrixXf &x_desired,
                                         const tf2::Vector3 &p_0_k_0,
                                         const tf2::Transform &C_0_k);

  /** \brief Extract errors for a sequence of poses */
  void extract_pose_errors(Eigen::MatrixXf &traj_errors,
                           const model_trajectory_t &x_sequence);

  /** \brief Convert pose errors to heading and lateral errors */
  Eigen::MatrixXf compute_pose_to_hl_error_conversion_mtx(float &th_des);

  /// Define functions related to computing disturbances

  /**
   * \brief Compute disturbances based on two sequential poses
   *  Computing for km2 because we only know velocity for km1 at time k
   */
  bool computeDisturbancesForExperienceKm2(experience_t &experience_km1,
                                           const experience_t &experience_k);

  /**
   * \brief MpcNominalModel::computeDisturbancesForExperienceKm2SteamVel
   * Compute model errors based on velocity estimates from STEAM. This is more
   * accurate than computeDisturbancesForExperienceKm2. It requires STEAM to be
   * enabled. To do this, set extrapolate_VO = true in a config file.
   * \param experience_km2: Experience from timestep k-2
   * \param experience_km1: Experience from timestep k-1
   * \return true unless the experience could not be computed for some reason.
   */
  bool computeDisturbancesForExperienceKm2SteamVel(
      experience_t &experience_km2, const experience_t &experience_km1);

  bool compute_disturbance_from_state(Eigen::VectorXf &g_a_k_meas,
                                      const model_state_t &state_km1,
                                      const model_state_t &state_k,
                                      const float &d_t);

  /** \brief Compute the velocity by finite difference using the pose estimates
   * from experience_k and experience_km1.
   *
   * If the difference between time-stamps
   * for these pose estimates is too small, keep the last velocity estimate.
   * \param experience_km2: Experience at time k-2
   * \param experience_km1: Experience at time k-1
   * \param experience_k: Experience at the current time, time k
   */
  void computeVelocitiesForExperienceKm1(const experience_t &experience_km2,
                                         experience_t &experience_km1,
                                         experience_t &experience_k);

  void compute_velocities_from_state(Eigen::VectorXf &velocity,
                                     const Eigen::VectorXf &state_km1,
                                     const Eigen::VectorXf &state_k_act,
                                     const float dt);

  void initialize_experience(experience_t &experience_k, rclcpp::Clock &clock);

  /**
   * \brief MpcNominalModel::initialize_state Set all the values in x_k to zero.
   * \param x_k
   */
  void initialize_state(model_state_t &x_k);

  void computeDisturbanceDependancy(model_state_t &x_k,
                                    const model_state_t &x_km1,
                                    const Eigen::MatrixXf &x_des,
                                    const float &nearest_path_length,
                                    float &d_t);

  /** \brief Extract relevant variables from model_state_t */
  void extract_disturbance_dependencies(const model_state_t &x_input,
                                        Eigen::VectorXf &x_test);

  void set_disturbance_model_zero(model_state_t &x_input);
};

}  // namespace path_tracker
}  // namespace vtr
