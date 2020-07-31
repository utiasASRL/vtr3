#pragma once
/*
File:
Edited By:      Chris Ostafew
Date:           Aug 11, 2014

Purpose:        To do

Functions:      To do
*/

// Standard C++ includes
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <unsupported/Eigen/SparseExtra>

#include <vtr/path_tracker/robust_mpc/optimization/path_tracker_mpc_solver_base.h>

const int SOLN_OK = 1;
const int UNCERT_EXCEEDS_CONSTRAINTS = 2;

namespace vtr {
namespace path_tracker {

class MpcSolverXUopt: public MpcSolverBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructor
  MpcSolverXUopt();

  // Destructor
  ~MpcSolverXUopt();

  // Public functions
  void reset_solver_specific(void);
  MpcNominalModel::model_state_t * select_x_pred(int index);
  MpcNominalModel::model_trajectory_t * select_x_pred(void);
  void set_desired_speed_ctrl(int & index, const double & speed, const double & ctrl);
  void set_desired_speed(int & index, const double & speed);
  void get_desired_ctrl(int & index);
  void post_process_x_pred(int & index);
  void compute_solver_update(const local_path_t & local_path, int iteration);

  void get_u(Eigen::MatrixXf & u_mat);
  void get_v(Eigen::MatrixXf & v_mat);
  void get_w(Eigen::MatrixXf & w_mat);
  void set_cmd_km1(const float & u_km1_in, const float & v_km1_in);
  void set_exp_consts(float c_in, float c_out);
  float c_in, c_out;




private:
  // Timing variables
  std::vector<float> norm_ds_opt;
  float step_size_;
  int iterations_small_step;

  // The variable mu is used by constrained optimization and varies as a function of iteration
  float mu_value; int mu_index;
  void update_mu_value(float & mu_value, int & mu_index);

  // Optimization matrix sizes & indices
  int size_x_vec, size_u_vec, size_v_vec, size_y_vec, size_z_vec, size_w_vec;
  int x_offset_index, u_offset_index, v_offset_index, y_offset_index;
  int z_lb_offset_index, z_ub_offset_index, w_lb_offset_index, w_ub_offset_index;

  // Intermediate optimization variables
  Eigen::MatrixXf x_ub_lims, x_lb_lims;
  Eigen::MatrixXf x_ub_abc, x_lb_abc;
  mtx_triplet_list_t weight_Q_triplet, weight_Ru_triplet, weight_Rv_triplet;
  Eigen::SparseMatrix<float,0> weight_Q; // weight_Ru, weight_Rv;
  Eigen::MatrixXf weight_Ru_diag, weight_Rv_diag, dI_t_Rdu_dI, dI_t_Rdv_dI, weight_u0, weight_v0;
  int solution_result;
  Eigen::MatrixXf x_minus_fx_, gx_minus_w_;

  Eigen::MatrixXf exp_bw, f_1, df_1, d2f_1, d2_cost_w;

  // Final optimization variables: opt_update = - J_grad_L_mtx^(-1)*grad_L;
  Eigen::MatrixXf s_bar, ds_bar;
  Eigen::MatrixXf grad_L;
  Eigen::SparseMatrix<float> J_grad_L_mtx;
  mtx_triplet_list_t J_grad_L_triplet_list;
  std::vector<bool> J_grad_L_mtx_entries_flg;
  std::vector<int> J_grad_L_mtx_indices;
  Eigen::MatrixXf grad_L_im1, ds_opt_im1;


  // utility function
  // Finds the sign of a number
  float getSign(float number);

  // Optimization algorithm helper functions
  void compute_mtx_offsets_and_sizes(void);
  void reset_solver_variables(void);
  void compute_weight_mtxs(void);
  void extract_x(const local_path_t & local_path);
  void compute_constraints_V2(const local_path_t & local_path);
  void extract_grad_L(void);
  void extract_J_grad_L(int iteration);

  void insert_triplet_list(mtx_triplet_list_t & triplet_list_out, mtx_triplet_list_t & triplet_list_in, float & mult, int & i, int & j, float sign = 1.0f);
  void insert_mtx(mtx_triplet_list_t & triplet_list_out, const Eigen::MatrixXf & gradients_in, int & i, int & j, float sign = 1.0f, bool transpose = false);
  void insert_triplet(mtx_triplet_list_t & triplet_list_out, const int & i, const int & j, float v_ij, float sign = 1.0f);

  void insert_triplet_list(Eigen::MatrixXf & mtx, mtx_triplet_list_t & triplet_list_in, float & mult, int & i, int & j, float sign = 1.0f);
  void insert_mtx(Eigen::MatrixXf & mtx, const Eigen::MatrixXf & gradients_in, int & i, int & j, float sign = 1.0f, bool transpose = false);
  void insert_triplet(Eigen::MatrixXf & mtx, const int & i, const int & j, float v_ij, float sign = 1.0f);

  void populate_mtx(Eigen::SparseMatrix<float,0> & mtx_sm, mtx_triplet_list_t & triplet_list);
  void reset_index_list(mtx_triplet_list_t & triplet_list_in);
  void compute_limiting_step_size(float & step_size, float & s_km1, float & ds, float lb, float ub);


};

} // path_tracker
} // asrl

