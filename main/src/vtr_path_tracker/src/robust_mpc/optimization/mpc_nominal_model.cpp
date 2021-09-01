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
 * \file mpc_nominal_model.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_logging/logging.hpp>
#include <vtr_path_tracker/robust_mpc/mpc/utilities.hpp>
#include <vtr_path_tracker/robust_mpc/optimization/mpc_nominal_model.hpp>

namespace vtr {
namespace path_tracker {

void adjust_mtx_triplet_row_col(mtx_triplet &mtx_triplet, int &i_in,
                                int &j_in) {
  mtx_triplet.i = mtx_triplet.i + i_in;
  mtx_triplet.j = mtx_triplet.j + j_in;
}

void scalar_mult_mtx_triplet(mtx_triplet &mtx_triplet, float &scalar_p) {
  mtx_triplet.v_ij = mtx_triplet.v_ij * scalar_p;
}

void MpcNominalModel::f_x_linearizedUncertainty(
    const MpcNominalModel::model_state_t &x_k,
    MpcNominalModel::model_state_t &x_kp1, float d_t) {
  Eigen::MatrixXf B(STATE_SIZE, 1);
  if (MODEL_INCLUDES_VELOCITY == false) {
    // commented_out, command_k -> command_km1
    if (MODEL_DEP_UKM1 == false) {
      B << d_t * cos((float)x_k.x_k[2]) * x_k.command_k[0],
          d_t * sin((float)x_k.x_k[2]) * x_k.command_k[0],
          d_t * K_OMEGA * x_k.command_k[1];
    } else {
      B << d_t * cos((float)x_k.x_k[2]) * x_k.command_k[0],
          d_t * sin((float)x_k.x_k[2]) * x_k.command_k[0],
          d_t * K_OMEGA * x_k.command_km1[1];
    }
  } else {
    B << d_t * cos((float)x_k.x_k[2]) * x_k.command_k[0],
        d_t * sin((float)x_k.x_k[2]) * x_k.command_k[0], d_t * x_k.x_k[3],
        d_t * K_OMEGA * (x_k.command_k[1] - x_k.x_k[3]);
  }
  x_kp1.x_k = x_k.x_k + B + d_t * x_k.g_a_k_des_frame;
  x_kp1.x_k[2] = utils::thetaWrap(x_kp1.x_k[2]);
  x_kp1.command_km1 = x_k.command_k;
  compute_velocities_from_state(x_kp1.velocity_km1, x_k.x_k, x_kp1.x_k, d_t);
}

void MpcNominalModel::get_gdot(model_state_t &x_k, float d_t) {
  /*** What?
    Given: x_kp1 = f(x_k,u_k), x_k element-of R^n
    (i.e., x_k = (x_0, x_1, x_2) and f(x_k,u_k) is vector-valued function):
    x_kp1,0 = f_0(x_k,u_k)
    x_kp1,1 = f_1(x_k,u_k)
    x_kp1,2 = f_2(x_k,u_k)

    Now: Compute gradients:
    gx[0] = gradient of f_0:  gx[0] = (df_0/dx0, df_0/dx1, df_0/dx2), column
    vector gx[1] = gradient of f_1:  gx[1] = (df_1/dx0, df_1/dx1, df_1/dx2),
    column vector gx[2] = gradient of f_2:  gx[2] = (df_2/dx0, df_2/dx1,
    df_2/dx2), column vector N.B. df/dx_k^T = [gx[0], gx[1], gx[2]]

    ***/

  /** gradient of f w.r.t. x_k **/
  x_k.grad_x = x_k.dg_dxk.transpose();
  x_k.grad_x(0, 0) += 1.0f;
  x_k.grad_x(1, 1) += 1.0f;
  x_k.grad_x(2, 2) += 1.0f;
  x_k.grad_x(2, 0) += -d_t * sin(x_k.x_k[2]) * x_k.command_k[0];
  x_k.grad_x(2, 1) += d_t * cos(x_k.x_k[2]) * x_k.command_k[0];

  /** gradient of f w.r.t. x_km1 **/
  x_k.grad_xkm1 = x_k.dg_dxkm1.transpose();

  /** gradient of f w.r.t. u_k **/
  x_k.grad_u = x_k.dg_duk.transpose();
  x_k.grad_u(0, 2) += d_t * K_OMEGA;

  /** gradient of f w.r.t. u_km1 **/
  x_k.grad_ukm1 = x_k.dg_dukm1.transpose();

  /** gradient of f w.r.t. v_k **/
  x_k.grad_v = x_k.dg_dvk.transpose();
  x_k.grad_v(0, 0) += d_t * cos(x_k.x_k[2]);
  x_k.grad_v(0, 1) += d_t * sin(x_k.x_k[2]);

  /** gradient of f w.r.t. v_km1 **/
  x_k.grad_vkm1 = x_k.dg_dvkm1.transpose();
}

void MpcNominalModel::get_Jdot_gdot(model_state_t &x_k, float d_t) {
  /*** What?
    Given: x_kp1 = f(x_k,u_k), x_k element-of R^n
    (i.e., x_k = (x_0, x_1, x_2) and f(x_k,u_k) is vector-valued function):
    x_kp1,0 = f_0(x_k,u_k)
    x_kp1,1 = f_1(x_k,u_k)
    x_kp1,2 = f_2(x_k,u_k)

    Now: Compute gradients:
    gx[0] = gradient of f_0:  gx[0] = (df_0/dx0, df_0/dx1, df_0/dx2), column
    vector gx[1] = gradient of f_1:  gx[1] = (df_1/dx0, df_1/dx1, df_1/dx2),
    column vector gx[2] = gradient of f_2:  gx[2] = (df_2/dx0, df_2/dx1,
    df_2/dx2), column vector N.B. df/dx_k^T = [gx[0], gx[1], gx[2]]

    Finally, Jdot_gdot represents Jacobians of Gradient functions!

    So,
    Jx_gx[0] = Jacobian of gx[0] w.r.t. x_k
    Jx_gx[1] = Jacobian of gx[1] w.r.t. x_k
    Jx_gx[2] = Jacobian of gx[2] w.t.r. x_k

    Ju_gx[0] = Jacobian of gx[0] w.r.t. u_k
    Ju_gx[1] = Jacobian of gx[1] w.r.t. u_k
    Ju_gx[2] = Jacobian of gx[2] w.r.t. u_k

    etc...

    ***/

  float d_t_cos_th_k = d_t * cos(x_k.x_k(2));
  float d_t_sin_th_k = d_t * sin(x_k.x_k(2));

  // d/dx_k * gx
  x_k.Jx_gx[0].clear();
  x_k.Jx_gx[0].push_back(mtx_triplet(2, 2, -d_t_cos_th_k * x_k.command_k[0]));
  x_k.Jx_gx[1].clear();
  x_k.Jx_gx[1].push_back(mtx_triplet(2, 2, -d_t_sin_th_k * x_k.command_k[0]));
  x_k.Jx_gx[2].clear();

  x_k.Ju_gx[0].clear();
  x_k.Ju_gx[1].clear();
  x_k.Ju_gx[2].clear();

  // d/dv_k * gx
  x_k.Jv_gx[0].clear();
  x_k.Jv_gx[0].push_back(mtx_triplet(2, 0, -d_t_sin_th_k));
  x_k.Jv_gx[1].clear();
  x_k.Jv_gx[1].push_back(mtx_triplet(2, 0, d_t_cos_th_k));
  x_k.Jv_gx[2].clear();

  if (!x_k.flg_hessians_cleared) {
    // d/dx_k * gu
    x_k.Jx_gu[0].clear();
    x_k.Jx_gu[1].clear();
    x_k.Jx_gu[2].clear();

    // d/du_k * gu
    x_k.Ju_gu[0].clear();
    x_k.Ju_gu[1].clear();
    x_k.Ju_gu[2].clear();

    // d/dv_k * gu
    x_k.Jv_gu[0].clear();
    x_k.Jv_gu[1].clear();
    x_k.Jv_gu[2].clear();
  }

  // d/dx_k * gv
  x_k.Jx_gv[0].clear();
  x_k.Jx_gv[0].push_back(mtx_triplet(0, 2, -d_t_sin_th_k));
  x_k.Jx_gv[1].clear();
  x_k.Jx_gv[1].push_back(mtx_triplet(0, 2, d_t_cos_th_k));
  x_k.Jx_gv[2].clear();
  x_k.Jv_gx_mtx(0, 2) = -d_t_sin_th_k;
  x_k.Jv_gx_mtx(1, 2) = d_t_cos_th_k;

  if (!x_k.flg_hessians_cleared) {
    // d/du_k * gv
    x_k.Ju_gv[0].clear();
    x_k.Ju_gv[1].clear();
    x_k.Ju_gv[2].clear();

    // d/du_k * gv
    x_k.Jv_gv[0].clear();
    x_k.Jv_gv[1].clear();
    x_k.Jv_gv[2].clear();

    x_k.flg_hessians_cleared = true;
  }
}

void MpcNominalModel::compute_velocities_from_state(
    Eigen::VectorXf &velocity, const Eigen::VectorXf &state_km1,
    const Eigen::VectorXf &state_k, const float d_t) {
  Eigen::VectorXf d_x(STATE_SIZE);
  d_x = state_k - state_km1;

  velocity = Eigen::VectorXf::Zero(VELOCITY_SIZE);

  float v_pos = std::pow(std::pow(d_x(0), 2) + std::pow(d_x(1), 2), 0.5) / d_t;
  if (d_x(0) > 0) {
    velocity(0) = v_pos;
  } else {
    velocity(0) = -v_pos;
  }

  if (MODEL_INCLUDES_VELOCITY == false) {
    velocity(1) = d_x(2) / d_t;
  } else {
    velocity(1) = state_km1(3);
  }
}

void MpcNominalModel::extract_pose_errors(
    Eigen::MatrixXf &hl_traj_errors,
    const MpcNominalModel::model_trajectory_t &x_sequence) {
  int num_poses = x_sequence.size();
  int index_err;
  for (int index1 = 1; index1 < num_poses; index1++) {
    index_err = ERROR_SIZE * (index1 - 1);

    hl_traj_errors(index_err, 0) = x_sequence[index1].tracking_error_k(1);
    hl_traj_errors(index_err + 1, 0) = x_sequence[index1].tracking_error_k(2);

    if (MODEL_INCLUDES_VELOCITY == true) {
      hl_traj_errors(index_err + 2, 0) = x_sequence[index1].tracking_error_k(3);
    }
  }
}

Eigen::MatrixXf MpcNominalModel::compute_pose_to_hl_error_conversion_mtx(
    float &th_des) {
  float c_th = cos(th_des);
  float s_th = sin(th_des);

  Eigen::MatrixXf errX_to_errHL_k;
  errX_to_errHL_k = Eigen::MatrixXf::Zero(ERROR_SIZE, STATE_SIZE);

  errX_to_errHL_k(0, 0) = -s_th;
  errX_to_errHL_k(0, 1) = c_th;
  errX_to_errHL_k(1, 2) = 1.0;

  if (MODEL_INCLUDES_VELOCITY == true) {
    errX_to_errHL_k(2, 3) = 1.0;
  }
  return errX_to_errHL_k;
}

void MpcNominalModel::compute_sequence_errors(
    MpcNominalModel::model_trajectory_t &x_sequence,
    const Eigen::MatrixXf &x_desired) {
  int num_poses = x_sequence.size();

  for (int k = 0; k < num_poses; k++) {
    // Translation from the reference frame to the desired pose, expressed in
    // the reference frame
    tf2::Vector3 p_0_k_0(x_desired(0, k), x_desired(1, k), 0.0);

    // Rotation from the reference frame to the desired pose
    tf2::Quaternion q;
    q.setRPY(0, 0, x_desired(2, k));
    tf2::Transform C_0_k(q);

    compute_pose_errors(x_sequence[k], x_desired.block<STATE_SIZE, 1>(0, k),
                        p_0_k_0, C_0_k);
  }
}

// Compute pose errors
Eigen::VectorXf MpcNominalModel::compute_pose_errorsNew(
    const MpcNominalModel::model_state_t &x_state,
    const Eigen::MatrixXf &x_desired) {
  // Translation from the reference frame to the desired pose, expressed in the
  // reference frame
  tf2::Vector3 p_0_k_0(x_desired(0, 0), x_desired(1, 0), 0.0);

  // Rotation from the reference frame to the desired pose
  tf2::Quaternion q;
  q.setRPY(0, 0, x_desired(2, 0));
  tf2::Transform C_0_k(q);

  return compute_pose_errorsNew(x_state, x_desired, p_0_k_0, C_0_k);
}

Eigen::VectorXf MpcNominalModel::compute_pose_errorsNew(
    const MpcNominalModel::model_state_t &x_state,
    const Eigen::MatrixXf &x_desired, const tf2::Vector3 &p_0_k_0,
    const tf2::Transform &C_0_k) {
  // Translation from the reference frame to the vehicle, expressed in the
  // reference frame
  tf2::Vector3 p_0_v_0(x_state.x_k(0), x_state.x_k(1), 0.0);

  // Translation from the desired pose to the vehicle, expressed in the desired
  // pose frame
  tf2::Vector3 p_k_v_k = C_0_k.inverse() * (p_0_v_0 - p_0_k_0);

  Eigen::VectorXf tracking_error = Eigen::VectorXf::Zero(STATE_SIZE);
  tracking_error(0) = 0.0 - p_k_v_k.getX();
  tracking_error(1) = 0.0 - p_k_v_k.getY();
  tracking_error(2) = 0.0 - (x_state.x_k(2) - x_desired(2, 0));

  return tracking_error;
}

Eigen::VectorXf MpcNominalModel::compute_pose_errors(
    MpcNominalModel::model_state_t &x_state, const Eigen::MatrixXf &x_desired) {
  // Translation from the reference frame to the desired pose, expressed in the
  // reference frame
  tf2::Vector3 p_0_k_0(x_desired(0, 0), x_desired(1, 0), 0.0);

  // Rotation from the reference frame to the desired pose
  tf2::Quaternion q;
  q.setRPY(0, 0, x_desired(2, 0));
  tf2::Transform C_0_k(q);

  return compute_pose_errors(x_state, x_desired, p_0_k_0, C_0_k);
}

Eigen::VectorXf MpcNominalModel::compute_pose_errors(
    MpcNominalModel::model_state_t &x_state, const Eigen::MatrixXf &x_desired,
    const tf2::Vector3 &p_0_k_0, const tf2::Transform &C_0_k) {
  // Translation from the reference frame to the vehicle, expressed in the
  // reference frame
  tf2::Vector3 p_0_v_0(x_state.x_k(0), x_state.x_k(1), 0.0);

  // Translation from the desired pose to the vehicle, expressed in the desired
  // pose frame
  tf2::Vector3 p_k_v_k(0, 0, 0);
  p_k_v_k = C_0_k.inverse() * (p_0_v_0 - p_0_k_0);

  // Set path tracking errors in x_state
  x_state.tracking_error_k = Eigen::VectorXf::Zero(STATE_SIZE);
  x_state.tracking_error_k(0) = 0.0 - p_k_v_k.getX();
  x_state.tracking_error_k(1) = 0.0 - p_k_v_k.getY();
  x_state.tracking_error_k(2) = 0.0 - (x_state.x_k(2) - x_desired(2, 0));

  if (MODEL_INCLUDES_VELOCITY == true) {
    x_state.tracking_error_k(3) = (x_desired(3, 0) - x_state.x_k(3));
  }

  return x_state.tracking_error_k;
}

bool MpcNominalModel::robot_has_passed_desired_poseNew(
    const float &v_des, const Eigen::VectorXf &x_k,
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

void MpcNominalModel::extract_disturbance_dependencies(
    const MpcNominalModel::model_state_t &x_input, Eigen::VectorXf &x_test) {
  // Get the prediction state
  x_test = Eigen::VectorXf::Zero(DIST_DEP_SIZE);
  x_test(DIST_ALONG_PATH) = x_input.dist_along_path_k;
  x_test(OMEGA_ACT_KM1) = x_input.velocity_km1[1];  // omega act
  x_test(V_ACT_KM1) = x_input.velocity_km1[0];      // v act
  x_test(OMEGA_CMD_KM1) = x_input.command_km1[1];   // omega cmd
  x_test(V_CMD_KM1) = x_input.command_km1[0];       // v cmd
  x_test(OMEGA_CMD_K) = x_input.command_k[1];       // omega cmd
  x_test(V_CMD_K) = x_input.command_k[0];           // v cmd
  x_test(HEAD_ERROR_K) = x_input.tracking_error_k[2];
  x_test(LAT_ERROR_K) = x_input.tracking_error_k[1];
}

bool MpcNominalModel::computeDisturbancesForExperienceKm2(
    MpcNominalModel::experience_t &experience_km2,
    const MpcNominalModel::experience_t &experience_km1) {
  bool success = true;

  /**
    Check validity of experiences
    **/

  if (!experience_km2.velocity_is_valid) {
    success = false;
  }

  bool state_size_correct =
      ((MODEL_INCLUDES_VELOCITY == true) && (STATE_SIZE == 4)) ||
      ((MODEL_INCLUDES_VELOCITY == false) && (STATE_SIZE == 3));
  if (!state_size_correct) {
    success = false;
    CLOG(WARNING, "path_tracker") << "State size doesn't match disturbance "
                                     "definition. Can't compute disturbance.";
  }

  rclcpp::Duration dt_ros =
      experience_km1.transform_time - experience_km2.transform_time;
  auto d_t = (float)dt_ros.seconds();
  if (d_t < 0.01) {
    success = false;
    CLOG(DEBUG, "path_tracker")
        << "It has been less than 0.01s since last VT&R pose update, no new "
           "disturbance calculated since likely this is old pose estimate.";
  }

  if (!success) {
    experience_km2.gp_data.g_x_meas = Eigen::VectorXf::Zero(DIST_DEP_SIZE);
    experience_km2.disturbance_is_valid = false;
    return success;
  }

  /**x_k
    Compute state km1 in frame km2
    **/

  // MpcNominalModel NominalModel;
  model_state_t x_km2, x_km1_act;
  initialize_state(x_km2);

  // Transform the robot poses
  tf2::Transform T_km2_km1 =
      experience_km2.T_0_v.inverse() * experience_km1.T_0_v;
  tf2::Vector3 p_km2_km1_km2 = T_km2_km1.getOrigin();
  tf2::Transform C_km2_km1(T_km2_km1.getRotation());
  tf2::Vector3 xhat(1, 0, 0);
  tf2::Vector3 th_vec = C_km2_km1 * xhat;
  float th_km1 = atan2(th_vec.getY(), th_vec.getX());

  if (MODEL_INCLUDES_VELOCITY == false) {
    // Create variables to use nominal model
    // x_km2.x_k = Eigen::VectorXf::Zero(STATE_SIZE);
    // x_km2.g_a_k_des_frame = Eigen::VectorXf::Zero(STATE_SIZE);
    x_km2.command_k = experience_km2.x_k.command_k;
    x_km2.command_km1 = experience_km2.x_k.command_km1;

    x_km1_act.x_k = Eigen::VectorXf::Zero(STATE_SIZE);
    x_km1_act.x_k << p_km2_km1_km2.getX(), p_km2_km1_km2.getY(), th_km1;

  } else {
    // Create variables to use nominal model
    // x_km2.x_k = Eigen::VectorXf::Zero(STATE_SIZE);
    x_km2.x_k(3) = experience_km2.velocity_k[1];
    // x_km2.g_a_k_des_frame = Eigen::VectorXf::Zero(STATE_SIZE);
    x_km2.command_k = experience_km2.x_k.command_k;
    x_km2.command_km1 = experience_km2.x_k.command_km1;

    x_km1_act.x_k = Eigen::VectorXf::Zero(STATE_SIZE);
    x_km1_act.x_k << p_km2_km1_km2.getX(), p_km2_km1_km2.getY(), th_km1,
        experience_km1.velocity_k[1];
  }

  /**
    Given state km1 in frame km2 and control km2, compute disturbance
    **/

  bool result = compute_disturbance_from_state(experience_km2.gp_data.g_x_meas,
                                               x_km2, x_km1_act, d_t);

  extract_disturbance_dependencies(experience_km2.x_k,
                                   experience_km2.gp_data.x_meas);

  for (int i = 0; i < DIST_DEP_SIZE; i++) {
    if (std::isnan(experience_km2.gp_data.x_meas(i))) {
      result = false;
    }
  }

  if (result) {
    experience_km2.disturbance_is_valid = true;
  } else {
    experience_km2.disturbance_is_valid = false;
  }

  return result;
}

bool MpcNominalModel::computeDisturbancesForExperienceKm2SteamVel(
    MpcNominalModel::experience_t &experience_km2,
    const MpcNominalModel::experience_t &experience_km1) {
  bool success = true, result = true;

  if (!experience_km2.velocity_is_valid) {
    success = false;
  }

  bool state_size_correct =
      ((MODEL_INCLUDES_VELOCITY == true) && (STATE_SIZE == 4)) ||
      ((MODEL_INCLUDES_VELOCITY == false) && (STATE_SIZE == 3));
  if (state_size_correct == false) {
    success = false;
    CLOG(WARNING, "path_tracker") << "State size doesn't match disturbance "
                                     "definition. Can't compute disturbance.";
  }

  if (!success) {
    experience_km2.gp_data.g_x_meas = Eigen::VectorXf::Zero(DIST_DEP_SIZE);
    experience_km2.disturbance_is_valid = false;
    return success;
  }

  // Get the rotation from frame km2 to km1
  tf2::Transform T_km2_km1 =
      experience_km2.T_0_v.inverse() * experience_km1.T_0_v;
  tf2::Transform C_km2_km1(T_km2_km1.getRotation());

  if (MODEL_INCLUDES_VELOCITY == true) {
    CLOG(ERROR, "path_tracker")
        << "Should never get here. Experiences will be invalid.";
    result = false;
  }

  // Rotate the body frame velocity from timestep km1 to frame km2 and compute
  // model error
  auto v_full = experience_km1.full_velocity_k;
  auto v_cmd = experience_km2.x_k.command_k(0);
  auto w_cmd = experience_km2.x_k.command_k(1);
  tf2::Vector3 v_xy(v_full(0), v_full(1), 0.);
  tf2::Vector3 g_xy = C_km2_km1 * v_xy;
  experience_km2.gp_data.g_x_meas << g_xy.x() - v_cmd, g_xy.y(),
      v_full(5) - K_OMEGA * w_cmd;

  extract_disturbance_dependencies(experience_km2.x_k,
                                   experience_km2.gp_data.x_meas);

  // Check for errors
  for (int i = 0; i < DIST_DEP_SIZE; i++) {
    if (std::isnan(experience_km2.gp_data.x_meas(i))) {
      result = false;
    }
  }

  if (result) {
    experience_km2.disturbance_is_valid = true;
  } else {
    experience_km2.disturbance_is_valid = false;
  }

  return result;
}

bool MpcNominalModel::compute_disturbance_from_state(
    Eigen::VectorXf &g_a_k_meas,
    const MpcNominalModel::model_state_t &state_km1,
    const MpcNominalModel::model_state_t &state_k_act, const float &d_t) {
  model_state_t state_k_nom;

  // Compute x_k from x_km1, pass by reference
  f_x_linearizedUncertainty(state_km1, state_k_nom, d_t);
  // bool result = f_x_unscentedUncertainty(state_km1, state_k_nom, d_t);

  g_a_k_meas = (state_k_act.x_k - state_k_nom.x_k) / d_t;

  bool result = true;
  for (int i = 0; i < STATE_SIZE; i++) {
    if (std::isnan(g_a_k_meas(i))) {
      result = false;
    }
  }

  if (!result) {
    g_a_k_meas = Eigen::VectorXf::Zero(STATE_SIZE);
  }

  return result;
}

bool MpcNominalModel::f_x_unscentedUncertainty(
    const MpcNominalModel::model_state_t &x_k,
    MpcNominalModel::model_state_t &x_kp1, float d_t) {
  if (x_k.x_k.size() != x_k.g_a_k_des_frame.size()) {
    CLOG(WARNING, "path_tracker")
        << "x_k (" << x_k.x_k.size() << ") and g_a_k_des_frame ("
        << x_k.g_a_k_des_frame.size() << ") are not of the same size.";
    f_x_linearizedUncertainty(x_k, x_kp1, d_t);
  } else {
    model_state_t x_kp1_linear = x_kp1;
    f_x_linearizedUncertainty(x_k, x_kp1_linear, d_t);

    MpcNominalModel NominalModel;

    // Preliminaries
    int size_x_k = x_k.x_k.size();
    int size_test_vec = 2 * size_x_k;
    float kappa = 2;
    float n_plus_kappa = (float)size_test_vec + kappa;

    // Get sigma points

    // Variable to hold the sigma points
    Eigen::MatrixXf x_test(size_test_vec, 2 * size_test_vec + 1);

    // State uncertainty, including both x_k and d_k
    Eigen::MatrixXf P_k;
    P_k = Eigen::MatrixXf::Zero(size_test_vec, size_test_vec);

    bool is_nan = false;
    bool is_neg = false;
    for (int i = 0; i < STATE_SIZE; i++) {
      if (std::isnan(x_k.var_g_a_k_des_frame(i, i))) {
        is_nan = true;
      }
      if (x_k.var_g_a_k_des_frame(i, i) <= 0) {
        is_neg = true;
      }
    }

    // Copy out state variances
    P_k.block(0, 0, size_x_k, size_x_k) = x_k.var_x_k;

    if (is_nan || is_neg) {
      CLOG(WARNING, "path_tracker")
          << "var_g_a_k_des_frame is nan/neg.  Replacing with default.";
      P_k.block(size_x_k, size_x_k, size_x_k, size_x_k) =
          0.00001 * Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);
    } else {
      P_k.block(size_x_k, size_x_k, size_x_k, size_x_k) =
          x_k.var_g_a_k_des_frame;
    }

    // Compute the Cholesky decomposition of P_k
    Eigen::MatrixXf S_k = P_k.llt().matrixL();  // llt = L*L^T

    // Compute the nominal sigma point
    x_test.block(0, 2 * size_test_vec, size_x_k, 1) = x_k.x_k;
    x_test.block(size_x_k, 2 * size_test_vec, size_x_k, 1) =
        x_k.g_a_k_des_frame;

    // Compute the other six points
    for (int i = 0; i < size_test_vec; i++) {
      x_test.block(0, 2 * i, size_test_vec, 1) =
          x_test.block(0, 2 * size_test_vec, size_test_vec, 1) +
          std::pow(n_plus_kappa, 0.5) * S_k.block(0, i, size_test_vec, 1);
      x_test.block(0, 2 * i + 1, size_test_vec, 1) =
          x_test.block(0, 2 * size_test_vec, size_test_vec, 1) -
          std::pow(n_plus_kappa, 0.5) * S_k.block(0, i, size_test_vec, 1);
    }

    // Pass the sigma points through the nonlinear function
    Eigen::MatrixXf y_test(size_x_k, 2 * size_test_vec + 1);
    model_state_t x_test_point, y_test_point;
    NominalModel.initialize_state(x_test_point);
    NominalModel.initialize_state(y_test_point);

    for (int i = 0; i <= 2 * size_test_vec; i++) {
      x_test_point = x_k;
      x_test_point.x_k = x_test.block(0, i, size_x_k, 1);
      x_test_point.g_a_k_des_frame = x_test.block(size_x_k, i, size_x_k, 1);
      NominalModel.f_x_linearizedUncertainty(x_test_point, y_test_point, d_t);
      y_test.block(0, i, size_x_k, 1) = y_test_point.x_k;
    }

    // Estimate new mean and cov
    Eigen::MatrixXf sum_of_test;
    sum_of_test = Eigen::MatrixXf::Zero(size_x_k, 1);
    Eigen::MatrixXf ut_output_mean(size_x_k, 1);
    for (int i = 0; i < y_test.rows(); i++) {
      float temp_row = 0;
      for (int j = 0; j < y_test.cols() - 1; j++) {
        temp_row = temp_row + y_test(i, j);
      }
      sum_of_test(i, 0) = temp_row;
    }

    ut_output_mean = (1 / n_plus_kappa) *
                     (kappa * y_test.block(0, 2 * size_test_vec, size_x_k, 1) +
                      0.5 * sum_of_test);

    Eigen::MatrixXf y_0_m_mean(size_x_k, 1);
    y_0_m_mean =
        y_test.block(0, 2 * size_test_vec, size_x_k, 1) - ut_output_mean;
    Eigen::MatrixXf y_j_m_mean(size_x_k, 1);
    Eigen::MatrixXf sum_cov, temp;
    sum_cov = Eigen::MatrixXf::Zero(size_x_k, size_x_k);
    for (int i = 0; i < 2 * size_test_vec; i++) {
      y_j_m_mean = y_test.block(0, i, size_x_k, 1) - ut_output_mean;
      temp = sum_cov + y_j_m_mean * y_j_m_mean.transpose();
      sum_cov = temp;
    }

    x_kp1.var_x_k =
        1 / (n_plus_kappa) *
        (kappa * (y_0_m_mean * y_0_m_mean.transpose()) + 0.5 * sum_cov);

    is_nan = false;
    is_neg = false;
    for (int i = 0; i < STATE_SIZE; i++) {
      if (std::isnan(x_kp1.var_x_k(i, i))) {
        is_nan = true;
        CLOG(ERROR, "path_tracker") << "Prediction step resulted in nan";
      }
      if (x_kp1.var_x_k(i, i) < 0) {
        is_neg = true;
        CLOG(ERROR, "path_tracker") << "Prediction step resulted in neg";
      }
    }

    if (is_nan || is_neg) {
      x_kp1.var_x_k =
          0.0000001 * Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);
    }

    /** Extract Worst-case Uncertainty (i.e., square bounding limits) **/
    Eigen::MatrixXf temp_var;
    temp_var = x_kp1.var_x_k;
    x_kp1.var_x_k =
        0.0000001 * Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);
    x_kp1.var_x_k(0, 0) = temp_var(0, 0);
    x_kp1.var_x_k(1, 1) = temp_var(1, 1);
    x_kp1.var_x_k(2, 2) = temp_var(2, 2);

    /** Compare output of SP and linearized predictions **/
    if ((x_kp1_linear.x_k - ut_output_mean).norm() > 0.1 || is_nan == true ||
        is_neg == true) {
      x_kp1.x_k = x_kp1_linear.x_k;
      CLOG_EVERY_N(10, WARNING, "path_tracker")
          << "Using linear prediction for the mean instead of the Sigma Point "
             "Transform";
    } else {
      x_kp1.x_k = ut_output_mean;
    }

    x_kp1.x_k[2] = utils::thetaWrap(x_kp1.x_k[2]);
    x_kp1.command_km1 = x_k.command_k;

    // set x_kp1.velocity_km1
    compute_velocities_from_state(x_kp1.velocity_km1, x_k.x_k, x_kp1.x_k, d_t);
  }
  return true;
}

void MpcNominalModel::computeInterpolatedDesiredPoseNew(
    const Eigen::MatrixXf &x_des_im1, const Eigen::MatrixXf &x_des_i,
    MpcNominalModel::model_state_t &x_pred, Eigen::MatrixXf &x_des_interp) {
  x_pred.tracking_error_k = compute_pose_errorsNew(x_pred, x_des_i);
  x_pred.tracking_error_km1 = compute_pose_errorsNew(x_pred, x_des_im1);

  // Compute the pct travelled between pose im1 and i
  float totalEx = std::abs(x_pred.tracking_error_km1(0)) +
                  std::abs(x_pred.tracking_error_k(0));
  float pct_im1 = std::abs(x_pred.tracking_error_k(0)) / totalEx;

  if (totalEx < 0.01) {
    pct_im1 = 0;
  }

  // Compute an interpolated desired pose for G-N formulation
  x_des_interp = pct_im1 * x_des_im1 + (1 - pct_im1) * x_des_i;
  x_des_interp(2, 0) = utils::thetaWrap(x_des_interp(2, 0));
}

void MpcNominalModel::computeDisturbanceDependancy(
    model_state_t &x_k, const model_state_t &x_km1,
    const Eigen::MatrixXf &x_des, const float &nearest_path_length,
    float &d_t) {
  // Compute tracking errors
  x_k.tracking_error_k_interp = compute_pose_errorsNew(x_k, x_des);

  // Copy commands
  x_k.command_km1 = x_km1.command_k;

  // Compute distance along path
  x_k.dist_along_path_k = computeDistAlongPath(
      nearest_path_length, x_k.tracking_error_k(0), x_k.command_km1[0]);

  // Compute velocity km1
  compute_velocities_from_state(x_k.velocity_km1, x_km1.x_k, x_k.x_k, d_t);
}

float MpcNominalModel::computeDistAlongPath(const float &nearest_path_length,
                                            const float &e_x,
                                            const float &linear_speed) {
  if (linear_speed < 0) {
    return nearest_path_length + e_x;
  } else {
    return nearest_path_length - e_x;
  }
}

bool MpcNominalModel::generateWorstCaseTrajectories(
    MpcNominalModel::model_trajectory_t &x_sequence,
    const double &robust_control_sigma) {
  int numPoses = x_sequence.size();

  // Rotation matrix between desired frame and the vehicle
  Eigen::MatrixXf C_r_k;
  C_r_k = Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);

  // Worst-case disturbance applied to state x at time k expressed in x_des_0
  // frame
  Eigen::MatrixXf three_sigma_disturbance, applied_disturbance;
  Eigen::VectorXf d_xy, d_th;
  three_sigma_disturbance = Eigen::MatrixXf::Zero(STATE_SIZE, 1);

  int num_scenarios = 2;
  std::vector<float> sign_vec(2);
  sign_vec[0] = -1;
  sign_vec[1] = 1;

  for (int k = 0; k < numPoses; k++) {
    // Compute a rotation mtx from the x_des_0 frame to the robot frame
    float th_robot = x_sequence[k].x_k(2);
    C_r_k(0, 0) = cos(th_robot);
    C_r_k(0, 1) = sin(th_robot);
    C_r_k(1, 0) = -sin(th_robot);
    C_r_k(1, 1) = cos(th_robot);

    Eigen::MatrixXf var_x_in_robot_frame =
        C_r_k * x_sequence[k].var_x_k * C_r_k.transpose();

    // Pick out disturbance 3*std deviations
    // Starts from 1 because we only want to transfer the uncertainty in the
    // lateral position
    three_sigma_disturbance = Eigen::MatrixXf::Zero(STATE_SIZE, 1);
    for (int i = 1; i < STATE_SIZE; i++) {
      three_sigma_disturbance(i, 0) = ((float)robust_control_sigma) *
                                      std::pow(var_x_in_robot_frame(i, i), 0.5);
    }

    // Store lateral and heading uncertainty
    x_sequence[k].lateral_uncertainty = three_sigma_disturbance(1, 0);
    x_sequence[k].heading_uncertainty = three_sigma_disturbance(2, 0);

    // Compute worst-case scenarios, for gui plotting only
    // Rotate back into x_des_0 frame
    three_sigma_disturbance = C_r_k.inverse() * three_sigma_disturbance;

    // Reinitialize d_xy and d_th
    d_xy = Eigen::MatrixXf::Zero(STATE_SIZE, 1);
    d_th = d_xy;

    d_xy(0, 0) = three_sigma_disturbance(0, 0);
    d_xy(1, 0) = three_sigma_disturbance(1, 0);
    d_th(2, 0) = three_sigma_disturbance(2, 0);

    int index = 0;
    x_sequence[k].x_k_wc.resize(num_scenarios);

    // Apply the adjustment
    for (int xy = 0; xy < 2; xy++) {
      applied_disturbance = sign_vec[xy] * d_xy;
      x_sequence[k].x_k_wc[index] = x_sequence[k].x_k + applied_disturbance;
      index++;
    }
  }  // for k = 0 -> numPoses
  return true;
}

void MpcNominalModel::initialize_state(MpcNominalModel::model_state_t &x_k) {
  // MPC variables
  x_k.x_k = Eigen::VectorXf::Zero(STATE_SIZE);
  x_k.x_k_wc.clear();
  x_k.var_x_k = 0.00001 * Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);
  x_k.dvar_x_dv = Eigen::MatrixXf::Zero(STATE_SIZE, STATE_SIZE);
  x_k.dvar_x_dth = Eigen::MatrixXf::Zero(STATE_SIZE, STATE_SIZE);

  x_k.lateral_uncertainty = 0.0f;
  x_k.heading_uncertainty = 0.0f;

  x_k.g_a_k = Eigen::VectorXf::Zero(STATE_SIZE);
  x_k.g_a_k_des_frame = Eigen::VectorXf::Zero(STATE_SIZE);
  x_k.var_g_a_k = 0.00001 * Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);
  x_k.var_g_a_k_des_frame =
      0.00001 * Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);

  // Tracking errors,
  x_k.dist_along_path_k = 0;
  x_k.tracking_error_k = Eigen::VectorXf::Zero(STATE_SIZE);
  x_k.tracking_error_km1 = Eigen::VectorXf::Zero(STATE_SIZE);
  x_k.tracking_error_k_interp = Eigen::VectorXf::Zero(STATE_SIZE);

  // Velocities and Commands
  x_k.velocity_km1 = Eigen::VectorXf::Zero(VELOCITY_SIZE);
  x_k.command_k = Eigen::VectorXf::Zero(VELOCITY_SIZE);
  x_k.command_km1 = Eigen::VectorXf::Zero(VELOCITY_SIZE);

  x_k.df_dxk = Eigen::MatrixXf::Zero(STATE_SIZE, STATE_SIZE);
  x_k.df_dxkm1 = Eigen::MatrixXf::Zero(STATE_SIZE, STATE_SIZE);
  x_k.df_duk = Eigen::MatrixXf::Zero(STATE_SIZE, CONTROL_SIZE);
  x_k.df_dukm1 = Eigen::MatrixXf::Zero(STATE_SIZE, CONTROL_SIZE);
  x_k.df_dvk = Eigen::MatrixXf::Zero(STATE_SIZE, CONTROL_SIZE);
  x_k.df_dvkm1 = Eigen::MatrixXf::Zero(STATE_SIZE, CONTROL_SIZE);

  x_k.dg_dxk = x_k.df_dxk;
  x_k.dg_dxkm1 = x_k.df_dxkm1;
  x_k.dg_duk = x_k.df_duk;
  x_k.dg_dukm1 = x_k.df_dukm1;
  x_k.dg_dvk = x_k.df_dvk;
  x_k.dg_dvkm1 = x_k.df_dvkm1;
  x_k.dg_da = Eigen::MatrixXf::Zero(STATE_SIZE, DIST_DEP_SIZE);

  x_k.grad_x = Eigen::MatrixXf::Zero(STATE_SIZE, STATE_SIZE);
  x_k.grad_xkm1 = Eigen::MatrixXf::Zero(STATE_SIZE, STATE_SIZE);
  x_k.grad_u = Eigen::MatrixXf::Zero(CONTROL_SIZE, STATE_SIZE);
  x_k.grad_ukm1 = Eigen::MatrixXf::Zero(CONTROL_SIZE, STATE_SIZE);
  x_k.grad_v = Eigen::MatrixXf::Zero(CONTROL_SIZE, STATE_SIZE);
  x_k.grad_vkm1 = Eigen::MatrixXf::Zero(CONTROL_SIZE, STATE_SIZE);

  x_k.Jx_gx.clear();
  x_k.Jx_gx.resize(STATE_SIZE);
  x_k.Ju_gx.clear();
  x_k.Ju_gx.resize(STATE_SIZE);
  x_k.Jv_gx.clear();
  x_k.Jv_gx.resize(STATE_SIZE);
  x_k.Jx_gu.clear();
  x_k.Jx_gu.resize(STATE_SIZE);
  x_k.Ju_gu.clear();
  x_k.Ju_gu.resize(STATE_SIZE);
  x_k.Jv_gu.clear();
  x_k.Jv_gu.resize(STATE_SIZE);
  x_k.Jx_gv.clear();
  x_k.Jx_gv.resize(STATE_SIZE);
  x_k.Ju_gv.clear();
  x_k.Ju_gv.resize(STATE_SIZE);
  x_k.Jv_gv.clear();
  x_k.Jv_gv.resize(STATE_SIZE);
  x_k.Jv_gx_mtx.setZero(STATE_SIZE, STATE_SIZE);
  x_k.flg_hessians_cleared = false;
}

void MpcNominalModel::initialize_experience(
    MpcNominalModel::experience_t &experience_k, rclcpp::Clock &clock) {
  // Initialize experience
  experience_k.at_vertex_id = VertexId(0, 0);
  experience_k.to_vertex_id = VertexId(0, 0);

  experience_k.transform_time = clock.now();
  experience_k.store_time = clock.now();

  initialize_state(experience_k.x_k);

  experience_k.tol_posL = 0.0f;
  experience_k.tol_posH = 0.0f;

  experience_k.path_curvature = 0;
  experience_k.dist_from_vertex = 0;

  experience_k.velocity_k = Eigen::VectorXf::Zero(VELOCITY_SIZE);
  experience_k.velocity_is_valid = false;
  experience_k.disturbance_is_valid = false;

  // Variables prepped for GP
  experience_k.gp_data.x_meas = Eigen::VectorXf::Zero(DIST_DEP_SIZE);
  experience_k.gp_data.g_x_meas = Eigen::VectorXf::Zero(STATE_SIZE);

  // Recorded variables for post-processing
  experience_k.T_0_v.setIdentity();
  experience_k.predicted_wc_lateral_error = 0;
  experience_k.predicted_wc_heading_error = 0;
  experience_k.v_k_odo = 0;
  experience_k.w_k_odo = 0;
  // experience_k.g_a_gp = Eigen::VectorXf::Zero(STATE_SIZE);
  // experience_k.var_g_a_gp = Eigen::MatrixXf::Zero(STATE_SIZE,STATE_SIZE);
  experience_k.keypoint_match_count = 0;
  experience_k.worst_case_scenario = -1;
  experience_k.mpc_valid = false;
  experience_k.J_iter.resize(1);
  experience_k.J_iter[0] = 0.0f;
  experience_k.u_iter.resize(1);
  experience_k.u_iter[0] = 0.0f;

  experience_k.t_mpc = 0.0;
  experience_k.t_robust = 0.0;

  experience_k.flg_do_not_delete = false;
}

void MpcNominalModel::set_disturbance_model_zero(model_state_t &x_input) {
  x_input.g_a_k = Eigen::VectorXf::Zero(STATE_SIZE);
  x_input.g_a_k_des_frame = Eigen::VectorXf::Zero(STATE_SIZE);
  x_input.var_g_a_k =
      0.00001 * Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);
  x_input.var_g_a_k_des_frame =
      0.00001 * Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);

  x_input.dg_dxk.setZero(STATE_SIZE, STATE_SIZE);
  x_input.dg_dxkm1.setZero(STATE_SIZE, STATE_SIZE);
  x_input.dg_duk.setZero(STATE_SIZE, 1);
  x_input.dg_dukm1.setZero(STATE_SIZE, 1);
  x_input.dg_dvk.setZero(STATE_SIZE, 1);
  x_input.dg_dvkm1.setZero(STATE_SIZE, 1);
}

void MpcNominalModel::computeVelocitiesForExperienceKm1(
    const MpcNominalModel::experience_t &experience_km2,
    MpcNominalModel::experience_t &experience_km1,
    MpcNominalModel::experience_t &experience_k) {
  // Transform the robot poses
  tf2::Transform T_km1_k = experience_km1.T_0_v.inverse() * experience_k.T_0_v;
  tf2::Vector3 p_km1_k_km1 = T_km1_k.getOrigin();
  tf2::Transform C_km1_k(T_km1_k.getRotation());
  tf2::Vector3 xhat(1, 0, 0);
  tf2::Vector3 th_vec = C_km1_k * xhat;
  float th_k = atan2(th_vec.getY(), th_vec.getX());

  // Arrange the change in pose
  Eigen::VectorXf x_km1;
  x_km1 = Eigen::VectorXf::Zero(STATE_SIZE);

  Eigen::VectorXf x_k(STATE_SIZE);
  x_k << p_km1_k_km1.getX(), p_km1_k_km1.getY(), th_k;

  // Compute the change in time
  rclcpp::Duration dt_ros =
      experience_k.transform_time - experience_km1.transform_time;
  auto d_t = (float)dt_ros.seconds();

  // Compute velocities
  if (d_t > 0.01) {
    compute_velocities_from_state(experience_km1.velocity_k, x_km1, x_k, d_t);
    experience_k.x_k.velocity_km1 = experience_km1.velocity_k;
    experience_km1.velocity_is_valid = true;
  } else {
    // Pose estimate is not new, copy v and w from previous time
    experience_km1.velocity_k = experience_km2.velocity_k;
    experience_km1.velocity_is_valid = false;
  }
}

}  // namespace path_tracker
}  // namespace vtr
