#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <vtr_logging/logging.hpp>
#include <vtr_path_tracker/robust_mpc/mpc/mpc_types.h>
#include <vtr_path_tracker/robust_mpc/optimization/path_tracker_mpc_nominal_model.h>

namespace vtr {
namespace path_tracker {

/** \brief Base class defining the interface to the path tracker framework
*/
class MpcSolverBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \todo: add documentation
  /** \brief
 */
  typedef struct {
    float weight_head, weight_head_final, weight_lat, weight_lat_final;
    float weight_u, weight_v, weight_du, weight_dv;
    float barrier_norm;
    bool flg_en_mpcConstraints, flg_en_robustMpcConstraints;
    bool flg_compute_dw_externally = true;

    float w_max; ///< max allowed turn rate
  } opt_params_t;

  /** \brief
 */
  typedef struct {
    Eigen::MatrixXf weight_Q, weight_Ru, weight_Rv;
  } opt_weight_mtx_t;

  /** \brief
 */
  typedef struct {
    bool flg_uncertain_pose_fails_constraints;
    bool flg_nominal_pose_fails_constraints;
    bool flg_nominal_pose_grossly_fails_constraints;
    bool flg_x_opt_and_pred_dont_match;
    bool flg_hessian_not_pos_def;
    bool flg_des_vel_one_point_turn;
    float delta_x_pred_opt;
    int num_failed_opt_results;
  } opt_flgs_t;

  Eigen::MatrixXf test_out_;
  opt_params_t opt_params;

  float u_km1, v_km1;

  Eigen::MatrixXf x_target, v_desired, x_desired;
  std::vector<float> norm_ds;
  std::vector<float> slope_step, slope_step1, slope_step2;
  std::vector<float> norm_grad_L_ds_opt_vec;

 protected:
  int size_x, size_u, size_v;
  int lookahead;

  opt_weight_mtx_t opt_weight_mtxs;
  Eigen::MatrixXf opt_variable, u_opt, v_opt;

  int curr_iteration;

  int solver_mode;
  float step_size;
  float step_max;
  float step_min;

 public:
  /** Make sure to initialize all flags to false in solver_base_implementation **/
  opt_flgs_t result_flgs;

  void set_sizes(int size_x_in, int size_u_in, int size_v_in);
  void set_weights(opt_params_t opt_params_in);
  void set_lookahead(int lookahead_in);
  void reset_solver(void);
  virtual void reset_solver_specific(void) = 0;
  // virtual void set_constraints(int index, int & pose_i, Eigen::MatrixXf & x_errLim_ub, Eigen::MatrixXf & x_errLim_lb)=0;

  /**  \brief Copy out u_opt from previous cmd
*/
  void copy_opt_km1(const int entries_to_skip, const int entries_to_copy);
  void compute_weight_matrices();
  virtual MpcNominalModel::model_state_t *select_x_pred(int index) = 0;
  virtual MpcNominalModel::model_trajectory_t *select_x_pred(void) = 0;
  virtual void set_desired_speed(int &index, const double &speed) = 0;
  virtual void get_desired_ctrl(int &index) = 0;
  virtual void set_desired_speed_ctrl(int &index, const double &speed, const double &ctrl) = 0;
  virtual void post_process_x_pred(int &index) = 0;

  virtual void compute_solver_update(const local_path_t &local_path, int iteration) = 0;
  virtual void get_u(Eigen::MatrixXf &u_mat) = 0;
  virtual void get_v(Eigen::MatrixXf &v_mat) = 0;
  virtual void get_w(Eigen::MatrixXf &w_mat) = 0;
  void reset_cmd_km1();
  virtual void set_cmd_km1(const float &u_km1, const float &v_km1) = 0;

  MpcNominalModel::model_trajectory_t x_pred, x_opt;

};

} // path_tracker
} // asrl