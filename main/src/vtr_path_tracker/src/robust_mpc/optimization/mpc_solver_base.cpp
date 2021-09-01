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
 * \file mpc_solver_base.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_path_tracker/robust_mpc/optimization/mpc_solver_base.hpp>

void vtr::path_tracker::MpcSolverBase::set_sizes(int size_x_in, int size_u_in,
                                                 int size_v_in) {
  size_x = size_x_in;
  size_u = size_u_in;
  size_v = size_v_in;
  u_opt.setZero(1, 1);
  test_out_ = Eigen::MatrixXf::Zero(1, 1);
}

void vtr::path_tracker::MpcSolverBase::set_weights(opt_params_t opt_params_in) {
  opt_params = opt_params_in;
}

void vtr::path_tracker::MpcSolverBase::reset_solver() {
  step_size = 0.0001;
  curr_iteration = 0;
  solver_mode = 0;
  step_max = 1.0;
  step_min = 0.001;
  reset_solver_specific();

  // Reset flags
  result_flgs.flg_uncertain_pose_fails_constraints = false;
  result_flgs.flg_nominal_pose_fails_constraints = false;
  result_flgs.flg_nominal_pose_grossly_fails_constraints = false;
  result_flgs.flg_x_opt_and_pred_dont_match = false;
  result_flgs.flg_hessian_not_pos_def = false;
  result_flgs.flg_des_vel_one_point_turn = false;
  result_flgs.delta_x_pred_opt = 0.0;
}

void vtr::path_tracker::MpcSolverBase::set_lookahead(int lookahead_in) {
  lookahead = lookahead_in;
}

void vtr::path_tracker::MpcSolverBase::copy_opt_km1(const int entries_to_skip,
                                                    const int entries_to_copy) {
  int length_vec = u_opt.rows();
  int entries_to_copy_checked =
      std::max(0, std::min(entries_to_copy - entries_to_skip, length_vec));

  if (entries_to_copy_checked > 0) {
    Eigen::MatrixXf vec_temp = u_opt;
    u_opt = Eigen::MatrixXf::Zero(lookahead, 1);
    u_opt.block(0, 0, entries_to_copy_checked, 1) =
        vec_temp.block(entries_to_skip, 0, entries_to_copy_checked, 1);
  } else {
    u_opt = Eigen::MatrixXf::Zero(lookahead, 1);
  }
}

void vtr::path_tracker::MpcSolverBase::reset_cmd_km1() {
  u_km1 = 0.0f;
  v_km1 = 0.0f;
}

void vtr::path_tracker::MpcSolverBase::compute_weight_matrices() {
  if (opt_params.weight_u == 0) {
    CLOG(INFO, "path_tracker")
        << "It is inadvisable to set a control input weight to zero.";
  }

  // Precompute weight matrices
  Eigen::MatrixXf weight_block;
  weight_block = Eigen::MatrixXf::Zero(ERROR_SIZE, ERROR_SIZE);
  opt_weight_mtxs.weight_Q =
      Eigen::MatrixXf::Zero(lookahead * ERROR_SIZE, lookahead * ERROR_SIZE);
  opt_weight_mtxs.weight_Ru = Eigen::MatrixXf::Zero(lookahead, lookahead);
  opt_weight_mtxs.weight_Rv = Eigen::MatrixXf::Zero(lookahead, lookahead);

  for (int index1 = 0; index1 < lookahead; ++index1) {
    int index_err = ERROR_SIZE * index1;

    if (index1 == lookahead - 1) {
      weight_block(0, 0) = opt_params.weight_lat + opt_params.weight_lat_final;
      weight_block(1, 1) =
          opt_params.weight_head + opt_params.weight_head_final;
    } else {
      weight_block(0, 0) = opt_params.weight_lat;
      weight_block(1, 1) = opt_params.weight_head;
    }

    opt_weight_mtxs.weight_Q.block(index_err, index_err, ERROR_SIZE,
                                   ERROR_SIZE) = weight_block;

    opt_weight_mtxs.weight_Ru(index1, index1) = opt_params.weight_u;
    opt_weight_mtxs.weight_Rv(index1, index1) = opt_params.weight_v;
  }
}
