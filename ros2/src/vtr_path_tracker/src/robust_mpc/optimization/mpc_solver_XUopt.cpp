
#include <Eigen/SVD>
#include <vtr_path_tracker/robust_mpc/optimization/mpc_solver_XUopt.h>

#define COMPUTE_DW_EXTERNALLY
const bool USE_LOG_BARRIER_FUNC = false;
const bool USE_LIN_BARRIER_FUNC = true;
const bool USE_EXP_BARRIER_FUNC = false;

namespace vtr {
namespace path_tracker {

void print_vec_str(std::vector<float> &vec_in, int start, int elements) {

  elements = std::min((int) vec_in.size(), start + elements);
  std::vector<float> failed_nums;

  for (int i = start; i < elements; i++) {
    char buffer[20];
    int length = std::snprintf(buffer, 20, "%.4f", vec_in[i]);
    if (length >= 19) {
      failed_nums.push_back(vec_in[i]);
      std::cout << "abc, ";
    } else {
      std::cout << buffer << ", ";
    }
  }
  std::cout << "\n";
}

void print_mtx_str(Eigen::MatrixXf &mtx_in, int row_start, int col_start, int rows, int cols, bool transpose) {

  LOG(INFO) << "Printing mtx (XUopt_impl): ";

  rows = std::min((int) mtx_in.rows(), row_start + rows);
  cols = std::min((int) mtx_in.cols(), col_start + cols);

  std::vector<float> failed_nums;

  if (!transpose) {
    for (int row = row_start; row < row_start + rows; row++) {
      for (int col = col_start; col < col_start + cols; col++) {

        char buffer[20];
        int length = std::snprintf(buffer, 20, "%.2f", mtx_in(row, col));
        if (length >= 19) {
          failed_nums.push_back(mtx_in(row, col));
          std::cout << "abc" << ", ";
        } else {
          std::cout << buffer << ", ";
        }
      }
      std::cout << "\n";
    }
  } else {
    for (int col = col_start; col < col_start + cols; col++) {
      for (int row = row_start; row < row_start + rows; row++) {

        char buffer[20];
        int length = std::snprintf(buffer, 20, "%.2f", mtx_in(row, col));
        if (length >= 19) {
          failed_nums.push_back(mtx_in(row, col));
          std::cout << "abc" << ", ";
        } else {
          std::cout << buffer << ", ";
        }
      }
      std::cout << "\n";
    }
  }

  if (!failed_nums.empty()) {
    LOG(WARNING) << "path tracker XU_opt_implementation:  Print mtx has elements that exceed the buffer size.";
    for (float &failed_num : failed_nums) {
      LOG(INFO) << failed_num;
    }
  }
}

float MpcSolverXUopt::getSign(float number) {
  return (float) ((0.0 < number) - (number < 0.0));
}

MpcSolverXUopt::MpcSolverXUopt() {
  result_flgs.num_failed_opt_results = 0;
}

MpcSolverXUopt::~MpcSolverXUopt() {}

void MpcSolverXUopt::set_cmd_km1(const float &u_km1_in, const float &v_km1_in) {
  u_km1 = u_km1_in;
  v_km1 = v_km1_in;
}

void MpcSolverXUopt::set_exp_consts(float c_in_new, float c_out_new) {
  c_in = c_in_new;
  c_out = c_out_new;
}

void MpcSolverXUopt::reset_solver_specific(void) {

  compute_mtx_offsets_and_sizes();

  reset_solver_variables();

  compute_weight_mtxs();

  x_lb_lims.setOnes(2, lookahead);
  x_ub_lims.setOnes(2, lookahead);
  x_lb_lims *= -50;
  x_ub_lims *= 50;

  step_size_ = 0.95;

  grad_L_im1.setZero(1, 1);
  ds_opt_im1.setZero(1, 1);
  slope_step.clear();
  slope_step1.clear();
  slope_step2.clear();

  iterations_small_step = 0;
}

void MpcSolverXUopt::compute_mtx_offsets_and_sizes(void) {

  // Compute the respective sizes
  size_x_vec = lookahead * size_x;
  size_u_vec = lookahead * size_u;
  size_v_vec = lookahead * size_v;
  size_y_vec = size_x_vec;

  // Compute the respective offset indices
  x_offset_index = 0;
  u_offset_index = x_offset_index + size_x_vec;
  v_offset_index = u_offset_index + size_u_vec;
  y_offset_index = v_offset_index + size_v_vec;

  // Compute the sizes and offset indices for variables involved in inequality constraints
  if (opt_params.flg_en_mpcConstraints) {
    size_z_vec = 2 * lookahead * (size_x - 1) + 2;
    size_w_vec = size_z_vec;
    z_lb_offset_index = y_offset_index + size_y_vec;
    z_ub_offset_index = z_lb_offset_index + size_z_vec / 2;
    w_lb_offset_index = z_lb_offset_index + size_z_vec;
    w_ub_offset_index = w_lb_offset_index + size_w_vec / 2;
    mu_value = 1;
    mu_index = 0;

  } else {
    size_z_vec = 0;
    size_w_vec = 0;
    z_lb_offset_index = 0;
    z_ub_offset_index = 0;
    w_lb_offset_index = 0;
    w_ub_offset_index = 0;
    mu_value = 1;
    mu_index = 0;
  }
}

void MpcSolverXUopt::reset_solver_variables(void) {

  // Initialize desired and target vectors
  // target: the current sequence of states produced by the current sequence of control inputs
  // desired: the desired sequence of states and control inputs (x_d, v_d)
  x_target.setZero(size_x_vec, 1);
  x_desired.setZero(size_x_vec, 1);
  v_desired.setZero(size_v_vec, 1);

  solution_result = SOLN_OK;

  // Initialize s_bar and grad_L (gradient of the Lagrangian w.r.t. s)
  int size_s_bar = size_x_vec + size_u_vec + size_v_vec + size_y_vec + size_z_vec + size_w_vec;
  s_bar = Eigen::MatrixXf::Zero(size_s_bar, 1);

  //s_bar.block(u_offset_index,0,size_u_vec,1) = 0.5*Eigen::MatrixXf::Ones(size_u_vec,1);
  grad_L = Eigen::MatrixXf::Zero(size_s_bar, 1);

  // Initialize Lagrange variables y and z)
  s_bar.block(y_offset_index, 0, size_y_vec + size_z_vec, 1) = 0.1 * Eigen::MatrixXf::Ones(size_y_vec + size_z_vec, 1);


  // Initialize the slack variables w
  if (opt_params.flg_en_mpcConstraints) {
    s_bar.block(w_lb_offset_index, 0, size_w_vec, 1) = Eigen::MatrixXf::Ones(size_w_vec, 1);
  }

  x_minus_fx_ = Eigen::MatrixXf::Zero(size_y_vec, 1);
  gx_minus_w_ = Eigen::MatrixXf::Zero(size_w_vec, 1);

  // Initialize the variables used to track the creation of the sparse Jacobian matrix
  J_grad_L_mtx_indices.clear();
  J_grad_L_mtx_entries_flg.clear();
  J_grad_L_mtx_indices.resize(size_s_bar * size_s_bar);
  J_grad_L_mtx_entries_flg.resize(size_s_bar * size_s_bar);
  for (unsigned i = 0; i < J_grad_L_mtx_entries_flg.size(); i++) {
    J_grad_L_mtx_entries_flg[i] = false;
  }

  norm_ds_opt.clear();
  norm_ds.clear();
  norm_grad_L_ds_opt_vec.clear();
}

void MpcSolverXUopt::compute_weight_mtxs(void) {
  int x_index, u_index, v_index;
  float wl = 2 * opt_params.weight_lat;
  float wlf = 2 * opt_params.weight_lat_final;
  float wh = 2 * opt_params.weight_head;
  float whf = 2 * opt_params.weight_head_final;
  float wu = 2 * opt_params.weight_u;
  float wv = 2 * opt_params.weight_v;

  weight_Q_triplet.clear();
  weight_Ru_triplet.clear();
  weight_Rv_triplet.clear();

  if (size_u_vec != size_v_vec) {
    LOG(WARNING) << "Expecting u and v vec of the same length (path_tracker_mpc_solver_XUopt_implementation.hpp).";
  }

  Eigen::MatrixXf dI_uv = Eigen::MatrixXf::Zero(size_u_vec, size_u_vec);
  Eigen::MatrixXf weight_du_vec(size_u_vec, 1);
  Eigen::MatrixXf weight_dv_vec(size_v_vec, 1);

  weight_Ru_diag.resize(size_u_vec, 1);
  weight_Rv_diag.resize(size_v_vec, 1);

  for (int index = 0; index < lookahead; index++) {
    x_index = size_x * index;
    u_index = size_u * index;
    v_index = size_v * index;

    if (index == lookahead - 1) {
      weight_Q_triplet.push_back(mtx_triplet(x_index, x_index, 1));
      weight_Q_triplet.push_back(mtx_triplet(x_index + 1, x_index + 1, wlf));
      weight_Q_triplet.push_back(mtx_triplet(x_index + 2, x_index + 2, whf));
    } else {
      weight_Q_triplet.push_back(mtx_triplet(x_index, x_index, 1));
      weight_Q_triplet.push_back(mtx_triplet(x_index + 1, x_index + 1, wl));
      weight_Q_triplet.push_back(mtx_triplet(x_index + 2, x_index + 2, wh));
    }

    weight_Ru_triplet.push_back(mtx_triplet(u_index, u_index, wu));
    weight_Rv_triplet.push_back(mtx_triplet(v_index, v_index, wv));

    weight_Ru_diag(index, 0) = wu;
    weight_Rv_diag(index, 0) = wv;

    /** Weighting to smooth inputs over time **/
    if (index == 0) {
      dI_uv(index, index) = 1;
    } else {
      dI_uv(index, index) = 1;
      dI_uv(index, index - 1) = -1;
    }
    weight_du_vec(index, 0) = opt_params.weight_du;
    weight_dv_vec(index, 0) = opt_params.weight_dv;
  }

  Eigen::MatrixXf u_0 = Eigen::MatrixXf::Zero(size_u_vec, 1);
  Eigen::MatrixXf v_0 = Eigen::MatrixXf::Zero(size_v_vec, 1);

  u_0(0, 0) = u_km1;
  v_0(0, 0) = v_km1;

  weight_u0 = -2 * dI_uv.transpose() * weight_du_vec.asDiagonal() * u_0;
  weight_v0 = -2 * dI_uv.transpose() * weight_dv_vec.asDiagonal() * v_0;

  //print_mtx_str(weight_u0,0,0,lookahead,1,true);

  dI_t_Rdu_dI = 2 * dI_uv.transpose() * weight_du_vec.asDiagonal() * dI_uv;
  dI_t_Rdv_dI = 2 * dI_uv.transpose() * weight_dv_vec.asDiagonal() * dI_uv;

  weight_Q.resize(size_x_vec, size_x_vec);
  populate_mtx(weight_Q, weight_Q_triplet);
}

void MpcSolverXUopt::compute_solver_update(const local_path_t &local_path, int iteration) {

  // Extract x_target, x_desired, x_lb, and x_ub from x_pred and local_path
  // Note: x_opt is constrained to be equal to x_target (dynamics) and weighted
  // to be close to x_desired
  extract_x(local_path);

  if (opt_params.flg_en_mpcConstraints) {
    compute_constraints_V2(local_path);
  }

  // Compose the gradient of the Lagrangian
  extract_grad_L();

  // Check progress relative to iteration im1
  int size_grad_L = grad_L.rows();
  int size_grad_L_im1 = grad_L_im1.rows();

  if (size_grad_L == size_grad_L_im1) {
    // Compute change in slope
    Eigen::MatrixXf dx_im1 = grad_L_im1.transpose() * ds_opt_im1;
    Eigen::MatrixXf dx_i = grad_L.transpose() * ds_opt_im1;

    if (dx_i.rows() != 1 || dx_i.cols() != 1) {
      LOG(WARNING) << "Not singleton.";
    }

    float f_dx_im1 = fabs(dx_im1(0, 0));
    float f_dx_i = fabs(dx_i(0, 0));
    slope_step1.push_back(f_dx_i);
    slope_step2.push_back(f_dx_im1);

    slope_step.push_back(f_dx_i / f_dx_im1);

    int steps_for_done = 6;
    if (iterations_small_step < steps_for_done) {
      if (f_dx_im1 < 1) {
        iterations_small_step = iterations_small_step + 1;
      } else {
        iterations_small_step = 0;
      }
    } else {
      iterations_small_step = iterations_small_step + 1;
    }
  }

  if (iterations_small_step < 3) {
    // Compose the Jacobian of the gradient of the Lagrangian
    extract_J_grad_L(iteration);

    // Update the value of mu (used in constrained optimization)
    update_mu_value(mu_value, mu_index);
  }
}

void MpcSolverXUopt::populate_mtx(Eigen::SparseMatrix<float, 0> &mtx_sm, mtx_triplet_list_t &triplet_list) {

  int num_elements = triplet_list.size();
  mtx_sm.reserve(num_elements);

  for (int element = 0; element < num_elements; element++) {
    if (triplet_list[element].i >= mtx_sm.rows() || triplet_list[element].j >= mtx_sm.cols()) {
      LOG(INFO) << "Solver.populate_mtx:  Trying to add mtx triplet outside size of mtx.";
    } else {
      mtx_sm.insert(triplet_list[element].i, triplet_list[element].j) = triplet_list[element].v_ij;
    }
  }
}

MpcNominalModel::model_state_t *MpcSolverXUopt::select_x_pred(int index) {
  MpcNominalModel::model_state_t *x_pred_curr_index = &x_opt[index];
  return x_pred_curr_index;
}

MpcNominalModel::model_trajectory_t *MpcSolverXUopt::select_x_pred(void) {
  MpcNominalModel::model_trajectory_t *x_pred_ptr = &x_opt;
  return x_pred_ptr;
}

void MpcSolverXUopt::set_desired_speed_ctrl(int &index, const double &speed, const double &ctrl) {

  if (fabs(s_bar(v_offset_index + index, 0)) > 0 || fabs(s_bar(v_offset_index + index, 0)) > 0) {
    LOG(WARNING) << "Overwriting initial (v,w) commands in MPC Solver.";
  }

  // Overwrite speed
  v_desired(index, 0) = (float) speed;
  s_bar(v_offset_index + index, 0) = (float) speed;
  x_opt[index].command_k[0] = s_bar(v_offset_index + index, 0);

  // Overwrite ctrl
  s_bar(u_offset_index + index, 0) = (float) ctrl;
  x_opt[index].command_k[1] = s_bar(u_offset_index + index, 0);
}

void MpcSolverXUopt::set_desired_speed(int &index, const double &speed) {
  v_desired(index, 0) = (float) speed;
  x_opt[index].command_k[0] = s_bar(v_offset_index + index, 0);
}

void MpcSolverXUopt::get_desired_ctrl(int &index) {
  x_opt[index].command_k[1] = s_bar(u_offset_index + index, 0);
}

void MpcSolverXUopt::post_process_x_pred(int &index) {
  x_opt[index + 1].var_x_k = x_pred[index + 1].var_x_k;
}

void MpcSolverXUopt::extract_x(const local_path_t &local_path) {

  int state_index;

  for (int index = 0; index < lookahead; index++) {
    state_index = size_x * index;
    x_target.block(state_index, 0, size_x, 1) = x_pred[index + 1].x_k;
    x_desired.block(state_index, 0, size_x, 1) = local_path.x_des_interp.block(0, index + 1, size_x, 1);
    if (opt_params.flg_en_mpcConstraints) {
      x_lb_lims.block(0, index, 2, 1) = local_path.x_lb_interp.block(0, index + 1, 2, 1);
      x_ub_lims.block(0, index, 2, 1) = local_path.x_ub_interp.block(0, index + 1, 2, 1);
    }
  }
}

void compute_C_0_k(Eigen::Matrix2f &C_0_k, const float &th_des) {
  /** Define the rotation mtx **/
  C_0_k << cos(th_des), -sin(th_des),
      sin(th_des), cos(th_des);
}

Eigen::MatrixXf compute_linear_abc(const Eigen::MatrixXf &point_1,
                                   const Eigen::MatrixXf &point_2,
                                   const Eigen::MatrixXf &des_point,
                                   const float &norm_dist) {
  Eigen::MatrixXf abc(3, 1), abc_temp(3, 1);
  /** a*x + b*y + c = 0 **/

  /** Compute the constants describing the line **/
  abc_temp << (point_2(1, 0) - point_1(1, 0)),
      (point_1(0, 0) - point_2(0, 0)),
      (point_2(0, 0) * point_1(1, 0) - point_1(0, 0) * point_2(1, 0));

  float test_out = abc_temp(0, 0) * des_point(0, 0) + abc_temp(1, 0) * des_point(1, 0) + abc_temp(2, 0);
  bool test_out_gt_zero = test_out > 0.0f;

  if (!test_out_gt_zero) {
    abc = -abc_temp;
  } else {
    abc = abc_temp;
  }
  abc_temp = abc;

  /** Normalize the constants such that at norm_dist from the line, ax+by+c = 1 **/
  // 1) Compute a point norm_dist perpendicular from the line connecting point1 and point2
  float th_line = std::atan2(point_2(1, 0) - point_1(1, 0), point_2(0, 0) - point_1(0, 0));
  Eigen::MatrixXf C_line(2, 2);
  C_line << cos(th_line), -sin(th_line),
      sin(th_line), cos(th_line);
  Eigen::MatrixXf norm_point_des(2, 1);
  norm_point_des << 0, fabs(norm_dist);
  Eigen::MatrixXf norm_point_lineFrame = point_1 + C_line * norm_point_des;

  // 2) Compute the value of abc(norm_point_lineFrame), then divide the abc constants
  if (fabs(norm_dist) > 0) {
    float
        norm_value = fabs(abc(0, 0) * norm_point_lineFrame(0, 0) + abc(1, 0) * norm_point_lineFrame(1, 0) + abc(2, 0));
    abc = abc / fabs(norm_value);
  } else {
    LOG(WARNING) << "Path tracker XU Solver: Cannot normalize at dist 0.";
  }

  if (fabs(abc(0, 0)) > 5000 || fabs(abc(1, 0)) > 5000 || fabs(abc(2, 0)) > 5000) {
    LOG(WARNING) << "Computation of abc results in large values (p_1,p_2,abc_temp/abc)";
    Eigen::MatrixXf test_out(2, 6);
    test_out << point_1, point_2, norm_point_lineFrame;
    test_out.block(0, 3, 1, 3) = abc_temp.transpose();
    test_out.block(1, 3, 1, 3) = abc.transpose();
    // print_mtx_str(test_out,0,0,2,6,false);
  }
  return abc;
}

Eigen::MatrixXf compute_bound_point(Eigen::MatrixXf &p_in, float &y_offset) {
  // Point (x,y coords) in frame of p_in
  Eigen::Matrix2f C_0_k;
  compute_C_0_k(C_0_k, p_in(2, 0));
  Eigen::MatrixXf p_offset(2, 1);
  p_offset << 0, y_offset;
  Eigen::MatrixXf p_out = p_in.block(0, 0, 2, 1) + C_0_k * p_offset;
  return p_out;
}

void MpcSolverXUopt::compute_constraints_V2(const local_path_t &local_path) {
  /** Prepare constraint coefficients
    * Lateral:
    *   a_lb*x + b_lb*y + c_lb > 0
    *   a_ub*x + b_ub*y + c_ub > 0
    *
    * Heading:
    *   (th_des - th) - eh_lb > 0
    *  -(th_des - th) + eh_ub > 0
    *
    **/

  // Initialize the constraints
  x_ub_abc.resize(5, lookahead);
  x_lb_abc.resize(5, lookahead);

  // Set to inf = Valgrind, Texas-style
  x_ub_abc = Eigen::MatrixXf::Ones(5, lookahead) / 0.0f;
  x_lb_abc = Eigen::MatrixXf::Ones(5, lookahead) / 0.0f;

  float c_mult = 1.0f;
  if (!opt_params.flg_en_robustMpcConstraints) {
    c_mult = 0.0f;
  }

  /** Check if nom/uncert pose meets constraints **/
  result_flgs.flg_nominal_pose_grossly_fails_constraints = false;
  if (x_ub_lims(0, 0) - x_opt[0].x_k[1] < -0.05 || x_opt[0].x_k[1] - x_lb_lims(0, 0) < -0.05) {
    result_flgs.flg_nominal_pose_grossly_fails_constraints = true;
  }

  result_flgs.flg_nominal_pose_fails_constraints = false;
  if (x_ub_lims(0, 0) - x_opt[0].x_k[1] < 0 || x_opt[0].x_k[1] - x_lb_lims(0, 0) < 0) {
    result_flgs.flg_nominal_pose_fails_constraints = true;
  }

  float ub_lim = x_ub_lims(0, 0) - c_mult * x_opt[0].lateral_uncertainty;
  float lb_lim = x_lb_lims(0, 0) + c_mult * x_opt[0].lateral_uncertainty;

  result_flgs.flg_uncertain_pose_fails_constraints = false;
  if (ub_lim - x_opt[0].x_k[1] < 0 || x_opt[0].x_k[1] - lb_lim < 0) {
    result_flgs.flg_uncertain_pose_fails_constraints = true;
  }

  /** Initialize temporary variables **/
  int k_i = 0; // index variable
  //float distance = 0;
  float d_max = 1.2; //max distance between constraint vertices, m
  Eigen::MatrixXf pose_1(3, 1), pose_2(3, 1); //init varibles representing constraint vertices

  /** Compute point 1 and offsets from point 1 (ub, lb, mid) **/
  pose_1 << -getSign(v_desired(0, 0)) * (d_max + 0.001), 0, 0; //initialize pose_1 behind the robot
  float c_ub = x_ub_lims(0, 0) - c_mult * x_opt[0].lateral_uncertainty;
  float c_lb = x_lb_lims(0, 0) + c_mult * x_opt[0].lateral_uncertainty;
  float c_mid_1 = (c_ub + c_lb) / 2.0f;
  float c_mid_2 = c_mid_1;

  /** Ensure constraints at point 1 are a minimum of 0.05m apart **/
  float c_min = 0.05f;
  if (c_ub - c_lb < c_min) {
    c_ub = c_mid_1 + c_min / 2.0f;
    c_lb = c_mid_1 - c_min / 2.0f;
  }

  /** Compute constraint points **/
  Eigen::MatrixXf p_1_ub(2, 1), p_1_lb(2, 1), p_1_in(2, 1), p_2_ub(2, 1), p_2_lb(2, 1);
  p_1_ub = compute_bound_point(pose_1, c_ub);
  p_1_lb = compute_bound_point(pose_1, c_lb);
  p_1_in = compute_bound_point(pose_1, c_mid_1);

  test_out_ = Eigen::MatrixXf::Zero(8, lookahead + 1);
  test_out_(0, 0) = pose_1(0, 0);
  test_out_(1, 0) = pose_1(1, 0);
  test_out_(2, 0) = p_1_ub(0, 0);
  test_out_(3, 0) = p_1_ub(1, 0);
  test_out_(4, 0) = p_1_lb(0, 0);
  test_out_(5, 0) = p_1_lb(1, 0);
  test_out_(6, 0) = p_1_in(0, 0);
  test_out_(7, 0) = p_1_in(1, 0);

  Eigen::MatrixXf x_lb_abc_fixed(3, 1), x_ub_abc_fixed(3, 1);

  /**
        For the following:
        1) Compute pose 2 (pose_2)
        2) Compute offsets from pose 2 (c_ub, c_lb, c_mid_2)
        3) Compute constraint points (p_2_ub, p_2_lb)
        4) Compute upper limit represented by lines:
            - line a*x + b*y + c = 0, goes between p_1_ub, p_2_ub
            - select sign of a,b,c such that when evaluated at p_1_in, a*x+b*y+c > 0
            - repeat for lower limit
        5) Advance variables so that p_1... becomes the values from p_2
        6) Repeat
    **/

  int num_advances = 0;
  for (int i = 0; i < lookahead; i++) {
    // Increase c_min
    if (i == 1) {
      c_min = -0.3f;
    }

    // Get distance from last point
    pose_2 = local_path.x_des_interp.block(0, i + 1, 3, 1);

    c_ub = x_ub_lims(0, i) - c_mult * x_opt[i].lateral_uncertainty;
    c_lb = x_lb_lims(0, i) + c_mult * x_opt[i].lateral_uncertainty;
    c_mid_2 = (c_ub + c_lb) / 2.0f;

    Eigen::Matrix2f C_0_k;
    compute_C_0_k(C_0_k, pose_2(2, 0));
    Eigen::MatrixXf d_pose_k = C_0_k.transpose() * (pose_2.block(0, 0, 2, 1) - pose_1.block(0, 0, 2, 1));

    bool flg_ovrlap_uncert = false;
    if (c_ub - c_lb < c_min) {
      c_ub = c_mid_2 + c_min / 2.0f;
      c_lb = c_mid_2 - c_min / 2.0f;
      flg_ovrlap_uncert = true;
    }

    if (fabs(d_pose_k(0, 0)) > d_max || i == lookahead - 1 || flg_ovrlap_uncert) {

      test_out_(0, i + 1) = pose_2(0, 0);
      test_out_(1, i + 1) = pose_2(1, 0);

      if (fabs(d_pose_k(0, 0)) > 0.01) {
        // Compute point2 (ub and lb)
        p_2_lb = compute_bound_point(pose_2, c_lb);
        p_2_ub = compute_bound_point(pose_2, c_ub);
        test_out_(2, i + 1) = p_2_ub(0, 0);
        test_out_(3, i + 1) = p_2_ub(1, 0);
        test_out_(4, i + 1) = p_2_lb(0, 0);
        test_out_(5, i + 1) = p_2_lb(1, 0);
        test_out_(6, i + 1) = p_1_in(0, 0);
        test_out_(7, i + 1) = p_1_in(1, 0);

        // Compute constraints
        x_lb_abc_fixed = compute_linear_abc(p_1_lb, p_2_lb, p_1_in, opt_params.barrier_norm);
        x_ub_abc_fixed = compute_linear_abc(p_1_ub, p_2_ub, p_1_in, opt_params.barrier_norm);
      } else {
        x_lb_abc_fixed << 0.0f, 1.0f, -(x_lb_lims(0, i) + c_mult * x_opt[i].lateral_uncertainty);
        x_ub_abc_fixed << 0.0f, -1.0f, (x_ub_lims(0, i) - c_mult * x_opt[i].lateral_uncertainty);
        test_out_(2, i + 1) = x_lb_abc_fixed(2, 0);
        test_out_(3, i + 1) = x_ub_abc_fixed(2, 0);
        test_out_(4, i + 1) = 9.99;
        test_out_(5, i + 1) = 9.99;
        test_out_(6, i + 1) = 9.99;
        test_out_(7, i + 1) = 9.99;
      }

      // Check if we need to fill all remaining points
      bool flg_fill_rem_points = false;
      int term_pose = i;
      if (c_ub < c_lb) {
        flg_fill_rem_points = true;
        term_pose = lookahead - 1;
      }

      for (int j = k_i; j <= term_pose; j++) {

        // Copy over xy constraints
        x_lb_abc.block(0, j, 3, 1) = x_lb_abc_fixed;
        x_ub_abc.block(0, j, 3, 1) = x_ub_abc_fixed;

        // Copy over theta constraints
        x_lb_abc(3, j) = 1.0f;
        x_lb_abc(4, j) = -local_path.x_des_interp(2, i + 1) - x_lb_lims(1, i);

        x_lb_abc(3, j) = 0.0f;
        x_lb_abc(4, j) = 3.0f;

        x_ub_abc(3, j) = -1.0f;
        x_ub_abc(4, j) = local_path.x_des_interp(2, i + 1) + x_ub_lims(1, i);

        x_ub_abc(3, j) = 0.0f;
        x_ub_abc(4, j) = 3.0f;

        k_i = k_i + 1;
      }

      if (flg_fill_rem_points) {
        break;
      } else {
        // Advance points
        num_advances = num_advances + 1;
        pose_1 = pose_2;
        c_mid_1 = c_mid_2;
        p_1_lb = p_2_lb;
        p_1_ub = p_2_ub;
        p_1_in = compute_bound_point(pose_1, c_mid_1);
      }
    }
  }

  /** Check that the final pose_2 is far enough forward **/
  float d2_xk = fabs(pose_2(0, 0) - x_opt[0].x_k[0]);
  float min_lookahead = 0.3;
  if (false && d2_xk < min_lookahead && num_advances <= 1) {
    LOG(INFO) << "Overwriting constraints for high uncert / slow speed.";
    // Compute pose 1
    pose_1 << -getSign(v_desired(0, 0)) * (d_max + 0.001), 0, 0; //initialize pose_1 behind the robot
    c_mid_1 = (x_ub_lims(0, 0) + x_lb_lims(0, 0)) / 2;
    c_ub = c_mid_1 + 0.05;
    c_lb = c_mid_1 - 0.05;
    p_1_ub = compute_bound_point(pose_1, c_ub);
    p_1_lb = compute_bound_point(pose_1, c_lb);
    p_1_in = compute_bound_point(pose_1, c_mid_1);

    pose_2(0, 0) = x_opt[0].x_k[0] + getSign(v_desired(0, 0)) * min_lookahead;
    c_mid_2 = (x_ub_lims(0, lookahead - 1) + x_lb_lims(0, lookahead - 1)) / 2;
    c_ub = c_mid_2 + 0.01;
    c_lb = c_mid_2 - 0.01;
    p_2_lb = compute_bound_point(pose_2, c_lb);
    p_2_ub = compute_bound_point(pose_2, c_ub);

    // Compute constraints
    x_lb_abc_fixed = compute_linear_abc(p_1_lb, p_2_lb, p_1_in, opt_params.barrier_norm);
    x_ub_abc_fixed = compute_linear_abc(p_1_ub, p_2_ub, p_1_in, opt_params.barrier_norm);

    for (int j = 0; j < lookahead; j++) {
      x_lb_abc.block(0, j, 3, 1) = x_lb_abc_fixed;
      x_ub_abc.block(0, j, 3, 1) = x_ub_abc_fixed;
    }
  }

  bool flg_nan = false;
  for (int col = 0; col < lookahead; col++) {
    for (int row = 0; row < 5; row++) {
      if (std::isnan(x_lb_abc(row, col)) || std::isnan(x_ub_abc(row, col)) || fabs(x_lb_abc(row, col)) > 5000
          || fabs(x_ub_abc(row, col)) > 5000) {
        flg_nan = true;
      }
    }
  }

  if (flg_nan) {
    LOG(WARNING) << "Computed constraints include nan or large value.";
    // print_mtx_str(x_lb_abc,0,0,5,lookahead,false);
    // print_mtx_str(x_ub_abc,0,0,5,lookahead,false);
  }
}

void MpcSolverXUopt::extract_grad_L(void) {

  grad_L.setZero(s_bar.rows(), 1);

  // dels_J
  Eigen::MatrixXf delx_J = weight_Q * (s_bar.block(x_offset_index, 0, size_x_vec, 1) - x_desired);
  Eigen::MatrixXf delu_J = weight_Ru_diag.cwiseProduct(s_bar.block(u_offset_index, 0, size_u_vec, 1))
      + dI_t_Rdu_dI * s_bar.block(u_offset_index, 0, size_u_vec, 1) + weight_u0;
  Eigen::MatrixXf delv_J = weight_Rv_diag.cwiseProduct(s_bar.block(v_offset_index, 0, size_v_vec, 1) - v_desired)
      + dI_t_Rdv_dI * s_bar.block(v_offset_index, 0, size_v_vec, 1) + weight_v0;

  if (delu_J.rows() != lookahead * size_u || delu_J.cols() != 1) {
    LOG(WARNING)
        << "cwiseProduct failed to produce mtx with proper dimensions (path_tracker_mpc_solver_XUopt_implementation.hpp).";
  }

  // dels_yTh
  int state_index, state_indexM1, state_indexM2, u_index, u_indexM1, v_index, v_indexM1, z_lb_index, z_ub_index;
  Eigen::MatrixXf delx_yTh, delx_zTg, delu_yTh, delv_yTh, delv_zTg;
  delx_yTh.setZero(size_x_vec, 1);
  delu_yTh.setZero(size_u_vec, 1);
  delv_yTh.setZero(size_v_vec, 1);
  delx_zTg.setZero(size_x_vec, 1);
  delv_zTg.setZero(size_v_vec, 1);

  Eigen::MatrixXf y_i;
  y_i.setZero(size_x, 1);

  /**
    Notes
    State/Input: a_k = (x_k, x_km1, u_k, u_km1, v_k, v_km1) where x_k in R^n
    (i.e. x_k = (x_1, ... , x_n))

    Process model: x_kp1 = f(a_k)

    State optimization variables: (x_kp1, ... , x_kpKp1)

    Constraints: h_k = x_kp1 - f(a_k) = 0, k = 0..K-1

    Lagrange multipliers: y_k, k = 0..K-1

    Finally: yTh = y.transpose()*h;
    **/

  // index is the index of the mpc look-ahead
  for (int look_ahead_index = 0; look_ahead_index < lookahead; look_ahead_index++) {

    state_index = size_x * look_ahead_index;
    state_indexM1 = size_x * (look_ahead_index - 1);
    state_indexM2 = size_x * (look_ahead_index - 2);
    u_index = size_u * (look_ahead_index);  // does not include offset index because it's going into temp vec first
    u_indexM1 = size_u * (look_ahead_index - 1);
    v_index = size_v * (look_ahead_index);
    v_indexM1 = size_v * (look_ahead_index - 1);
    z_lb_index = z_lb_offset_index + 1 + 2 * look_ahead_index;
    z_ub_index = z_ub_offset_index + 2 * look_ahead_index;

    // Get y_i
    y_i = s_bar.block(y_offset_index + state_index, 0, size_x, 1);

    delx_yTh.block(state_index, 0, size_x, 1) = y_i;
    delu_yTh.block(u_index, 0, size_u, 1) -= x_opt[look_ahead_index].grad_u * y_i;
    delv_yTh.block(v_index, 0, size_u, 1) -= x_opt[look_ahead_index].grad_v * y_i;

    if (look_ahead_index > 0) {
      delx_yTh.block(state_indexM1, 0, size_x, 1) -= x_opt[look_ahead_index].grad_x * y_i;
      delu_yTh.block(u_indexM1, 0, size_u, 1) -= x_opt[look_ahead_index].grad_ukm1 * y_i;
      delv_yTh.block(v_indexM1, 0, size_v, 1) -= x_opt[look_ahead_index].grad_vkm1 * y_i;
    }

    if (look_ahead_index > 1) {
      delx_yTh.block(state_indexM2, 0, size_x, 1) -= x_opt[look_ahead_index].grad_xkm1 * y_i;
    }

    if (opt_params.flg_en_mpcConstraints) {
      // grad_x
      delx_zTg(state_index, 0) =
          s_bar(z_lb_index, 0) * x_lb_abc(0, look_ahead_index) + s_bar(z_ub_index, 0) * x_ub_abc(0, look_ahead_index);
      delx_zTg(state_index + 1, 0) =
          s_bar(z_lb_index, 0) * x_lb_abc(1, look_ahead_index) + s_bar(z_ub_index, 0) * x_ub_abc(1, look_ahead_index);
      delx_zTg(state_index + 2, 0) = s_bar(z_lb_index + 1, 0) * x_lb_abc(3, look_ahead_index)
          + s_bar(z_ub_index + 1, 0) * x_ub_abc(3, look_ahead_index);
    }
  }

  Eigen::MatrixXf x_opt = s_bar.block(x_offset_index, 0, size_x_vec, 1);

  grad_L.block(x_offset_index, 0, size_x_vec, 1) = delx_J - delx_yTh - delx_zTg;
  grad_L.block(u_offset_index, 0, size_u_vec, 1) = delu_J - delu_yTh;
  grad_L.block(v_offset_index, 0, size_v_vec, 1) = delv_J - delv_yTh; // - delv_zTg;
  x_minus_fx_ = x_opt - x_target;
  grad_L.block(y_offset_index, 0, size_y_vec, 1) = -(x_minus_fx_);

  if (opt_params.flg_en_mpcConstraints) {

    int state_K = size_x * (lookahead - 1);
    int z_K = (size_x - 1) * lookahead;

    // grad_x, init and term constraints
    grad_L(x_offset_index, 0) -= -s_bar(z_lb_offset_index, 0);     // x > -50, hard-coded
    grad_L(x_offset_index + state_K, 0) -= -s_bar(z_ub_offset_index + z_K, 0); // x_opt(0,0) > -50, hard-coded

    // grad_z, init and term constraints
    grad_L(z_lb_offset_index, 0) =
        -(-s_bar(0, 0) + 50.0 - s_bar(w_lb_offset_index, 0));     // x_opt(0,0) > -50, hard-coded
    grad_L(z_ub_offset_index + z_K, 0) =
        -(-s_bar(state_K, 0) + 50.0 - s_bar(w_ub_offset_index + z_K, 0)); // x_opt(0,0) > -50, hard-coded

    int el_index_lb, eh_index_lb, el_index_ub, eh_index_ub, x_index;

    for (int index = 0; index < lookahead; index++) {
      x_index = index * size_x;
      el_index_lb = 1 + 2 * index;
      eh_index_lb = 1 + 2 * index + 1;
      el_index_ub = 2 * index;
      eh_index_ub = 2 * index + 1;

      float x_act = s_bar(x_index, 0);
      float y_act = s_bar(x_index + 1, 0);
      float th_act = s_bar(x_index + 2, 0);

      // grad_z, lower bound
      grad_L(z_lb_offset_index + el_index_lb, 0) =
          -(x_lb_abc(0, index) * x_act + x_lb_abc(1, index) * y_act + x_lb_abc(2, index)
              - s_bar(w_lb_offset_index + el_index_lb, 0));
      grad_L(z_lb_offset_index + eh_index_lb, 0) =
          -(x_lb_abc(3, index) * th_act + x_lb_abc(4, index) - s_bar(w_lb_offset_index + eh_index_lb, 0));

      // grad_z, upper bound
      grad_L(z_ub_offset_index + el_index_ub, 0) =
          -(x_ub_abc(0, index) * x_act + x_ub_abc(1, index) * y_act + x_ub_abc(2, index)
              - s_bar(w_ub_offset_index + el_index_ub, 0));
      grad_L(z_ub_offset_index + eh_index_ub, 0) =
          -(x_ub_abc(3, index) * th_act + x_ub_abc(4, index) - s_bar(w_ub_offset_index + eh_index_ub, 0));
    }

    Eigen::MatrixXf z_vec = s_bar.block(z_lb_offset_index, 0, size_z_vec, 1);
    Eigen::MatrixXf w_vec = s_bar.block(w_lb_offset_index, 0, size_w_vec, 1);

    // grad_w
    if (USE_LOG_BARRIER_FUNC == true) {
      grad_L.block(w_lb_offset_index, 0, size_w_vec, 1) = (z_vec.cwiseProduct(w_vec).array() - mu_value);

    } else if (USE_LIN_BARRIER_FUNC == true) {

      float a_lin = 400;
      float b_lin = 100;
      Eigen::MatrixXf b_w = b_lin * w_vec;

      for (int i = 0; i < size_w_vec; i++) {
        // Above and below 50, the sigmoid becomes NAN due to computation of exp(+-50)
        b_w(i, 0) = std::min(10.0f, std::max(-10.0f, b_w(i, 0)));
      }

      exp_bw = (b_w).array().exp();
      Eigen::MatrixXf temp = Eigen::MatrixXf::Ones(size_w_vec, 1);
      f_1 = (temp + exp_bw).cwiseInverse();

      temp = f_1.array().pow(2);
      df_1 = -b_lin * exp_bw.cwiseProduct(temp);
      d2f_1 = -b_lin * b_lin * exp_bw.cwiseProduct(temp);
      d2f_1 -= 2 * b_lin * exp_bw.cwiseProduct(f_1.cwiseProduct(df_1));

      // Linear: cost = a_lin*w(i,0)*sigmoid(b_lin*w_vec(i,0))
      //grad_L.block(w_lb_offset_index,0, size_w_vec  ,1) =    z_vec + (-a_lin*f_1-a_lin*w_vec.cwiseProduct(df_1));
      //d2_cost_w = -2*a_lin*df_1 - a_lin*w_vec.cwiseProduct(d2f_1);

      // Square: cost = a_lin*w(i,0)^2*sigmoid(b_lin*w_vec(i,0))
      temp = w_vec.array().square();
      grad_L.block(w_lb_offset_index, 0, size_w_vec, 1) =
          z_vec + 2 * a_lin * w_vec.cwiseProduct(f_1) + a_lin * temp.cwiseProduct(df_1);
      d2_cost_w = 2 * a_lin * f_1 + 4 * a_lin * w_vec.cwiseProduct(df_1) + a_lin * temp.cwiseProduct(d2f_1);

      for (int i = 0; i < size_w_vec; i++) {
        //if (b_lin*w_vec(i,0) > 10){
        if (b_lin * w_vec(i, 0) > 0) {
          grad_L(w_lb_offset_index + i, 0) = z_vec(i, 0);
          d2_cost_w(i, 0) = 0.0f;
        } else if (b_lin * w_vec(i, 0) < -10) {
          grad_L(w_lb_offset_index + i, 0) = z_vec(i, 0) + 2 * a_lin * w_vec(i, 0);
          d2_cost_w(i, 0) = 2 * a_lin;
        }
      }

      // check for nan/inf
      bool flg_nan = false;
      for (int i = 0; i < size_w_vec; i++) {
        if (std::isnan(d2_cost_w(i, 0)) || std::isnan(grad_L(w_lb_offset_index + i, 0))) {
          flg_nan = true;
        }
      }
      if (flg_nan) {
        LOG(WARNING) << "Detected nan in computation of barrier function, grad_L, d2_cost_w:";
        // print_mtx_str(grad_L,w_lb_offset_index,0,size_w_vec,1,true);
        // print_mtx_str(d2_cost_w,0,0,size_w_vec,1,true);
      }
    } else {
      Eigen::MatrixXf exp_c_in_w = (-c_in * w_vec).array().exp();
      grad_L.block(w_lb_offset_index, 0, size_w_vec, 1) = z_vec - c_out * c_in * exp_c_in_w;
    }
  }
}

void MpcSolverXUopt::extract_J_grad_L(int iteration) {

  J_grad_L_triplet_list.clear();

  Eigen::MatrixXf y_i;
  y_i.setZero(3, 1);

  int x_index, x_indexM1, x_indexM2, u_index, u_indexM1, v_index, v_indexM1,
      y_index; //, z_index_lb, z_index_ub, w_index_lb, w_index_ub;

  int size_ds_bar = std::max(0, (int) s_bar.rows() - size_z_vec - size_w_vec);
  Eigen::MatrixXf J_grad_L;
  J_grad_L = Eigen::MatrixXf::Zero(size_ds_bar, size_ds_bar);

  mtx_triplet_list_t identity_mtx_3x3;
  identity_mtx_3x3.push_back(mtx_triplet(0, 0, 1.0f));
  identity_mtx_3x3.push_back(mtx_triplet(1, 1, 1.0f));
  identity_mtx_3x3.push_back(mtx_triplet(2, 2, 1.0f));

  bool transpose_true = true;

  // Create list of entries for the Jacobian
  for (int index = 0; index < lookahead; index++) {
    x_index = x_offset_index + size_x * index;
    x_indexM1 = x_offset_index + size_x * (index - 1);
    x_indexM2 = x_offset_index + size_x * (index - 2);
    u_index = u_offset_index + size_u * index;
    u_indexM1 = u_offset_index + size_u * (index - 1);
    v_index = v_offset_index + size_v * index;
    v_indexM1 = v_offset_index + size_v * (index - 1);
    y_index = y_offset_index + size_x * index;

    y_i = s_bar.block(y_offset_index + index * STATE_SIZE, 0, STATE_SIZE, 1);

    if (index > 0) {
      for (int j = 0; j < size_x; j++) {
        // Jx_(-gx)
        insert_triplet_list(J_grad_L, x_opt[index].Jx_gx[j], y_i(j, 0), x_indexM1, x_indexM1);

        // Jv_(-gx)
        insert_triplet_list(J_grad_L, x_opt[index].Jv_gx[j], y_i(j, 0), x_indexM1, v_index);

        // Jx_(-gv)
        insert_triplet_list(J_grad_L, x_opt[index].Jx_gv[j], y_i(j, 0), v_index, x_indexM1);
      }
    }

    //Jy_(-gx)
    float mult = 1.0f;
    insert_triplet_list(J_grad_L, identity_mtx_3x3, mult, x_index, y_index, -1.0f);
    if (index > 0) {
      insert_mtx(J_grad_L, x_opt[index].grad_x, x_indexM1, y_index);
    }
    if (index > 1) {
      insert_mtx(J_grad_L, x_opt[index].grad_xkm1, x_indexM2, y_index);
    }

    //Jy_(-gu)
    insert_mtx(J_grad_L, x_opt[index].grad_u, u_index, y_index);
    if (index > 0) {
      insert_mtx(J_grad_L, x_opt[index].grad_ukm1, u_indexM1, y_index);
    }

    //Jy_(-gv)
    insert_mtx(J_grad_L, x_opt[index].grad_v, v_index, y_index);
    if (index > 0) {
      insert_mtx(J_grad_L, x_opt[index].grad_vkm1, v_indexM1, y_index);
    }

    // Jx_(-gy)
    float sign = 1.0f;
    insert_triplet_list(J_grad_L, identity_mtx_3x3, mult, y_index, x_index, -1.0f);
    if (index > 0) {
      insert_mtx(J_grad_L, x_opt[index].grad_x, y_index, x_indexM1, sign, transpose_true);
    }
    if (index > 1) {
      insert_mtx(J_grad_L, x_opt[index].grad_xkm1, y_index, x_indexM2, sign, transpose_true);
    }

    // Ju_(-gy)
    insert_mtx(J_grad_L, x_opt[index].grad_u, y_index, u_index, sign, transpose_true);
    if (index > 0) {
      insert_mtx(J_grad_L, x_opt[index].grad_ukm1, y_index, u_indexM1, sign, transpose_true);
    }

    // Jv_(-gy)
    insert_mtx(J_grad_L, x_opt[index].grad_v, y_index, v_index, sign, transpose_true);
    if (index > 0) {
      insert_mtx(J_grad_L, x_opt[index].grad_vkm1, y_index, v_indexM1, sign, transpose_true);
    }
  }

  float mult = 1.0f;
  insert_triplet_list(J_grad_L, weight_Q_triplet, mult, x_offset_index, x_offset_index);
  insert_triplet_list(J_grad_L, weight_Ru_triplet, mult, u_offset_index, u_offset_index);
  insert_triplet_list(J_grad_L, weight_Rv_triplet, mult, v_offset_index, v_offset_index);

  insert_mtx(J_grad_L, dI_t_Rdu_dI, u_offset_index, u_offset_index);
  insert_mtx(J_grad_L, dI_t_Rdv_dI, v_offset_index, v_offset_index);

  Eigen::MatrixXf A_1, b_1, diag_A2, b_2, b_3;

  /** Compute dz and dw through back-substitution
        *  dw = A_1*dx + b_1
        *  dz = A_2*dw + b_2
        *     = A_2*A_1*dx + (A_2*b_1 + b_2)
        *     = A_3*dx + b_3
        *
        * where:
        *   A_1 = -Jx_gz
        *   b_1 = -gz
        *   A_2 = -(Jz_gw)^-1*(Jw_gw)
        *   b_2 = -(Jz_gw)^-1*(gw)
        */

  if (opt_params.flg_en_mpcConstraints) {

    /** Compute A_1 (for-loop) and b_1 **/
    A_1 = Eigen::MatrixXf::Zero(size_z_vec, size_x_vec);
    b_1 = -grad_L.block(z_lb_offset_index, 0, size_z_vec, 1);

    int index_z_lb, index_z_ub;
    Eigen::MatrixXf Jx_gz_k_lb(2, 3), Jx_gz_k_ub(2, 3);
    for (int index = 0; index < lookahead; index++) {

      x_index = size_x * index;
      index_z_lb = 1 + 2 * index;
      index_z_ub = size_z_vec / 2 + 2 * index;

      A_1(index_z_lb, x_index) = x_lb_abc(0, index);
      A_1(index_z_lb, x_index + 1) = x_lb_abc(1, index);
      A_1(index_z_lb + 1, x_index + 2) = x_lb_abc(3, index);

      A_1(index_z_ub, x_index) = x_ub_abc(0, index);
      A_1(index_z_ub, x_index + 1) = x_ub_abc(1, index);
      A_1(index_z_ub + 1, x_index + 2) = x_ub_abc(3, index);
    }

    // Initial and terminal conditions
    A_1(0, 0) = -1.0f;
    A_1(size_z_vec / 2 + lookahead * (size_x - 1), (lookahead - 1) * size_x) = -1.0f;

    /** Compute A_2 and b_2 **/
    if (USE_LOG_BARRIER_FUNC == true) {
      Eigen::MatrixXf diag_Jz_gw_INV = s_bar.block(w_lb_offset_index, 0, size_w_vec, 1).cwiseInverse();
      diag_A2 = -diag_Jz_gw_INV.cwiseProduct(s_bar.block(z_lb_offset_index, 0, size_w_vec, 1));
      b_2 = -diag_Jz_gw_INV.cwiseProduct(grad_L.block(w_lb_offset_index, 0, size_w_vec, 1));

    } else if (USE_LIN_BARRIER_FUNC == true) {
      diag_A2 = -d2_cost_w;
      b_2 = -grad_L.block(w_lb_offset_index, 0, size_w_vec, 1);

    } else { // Use exponential barrier function
      Eigen::MatrixXf w_vec = s_bar.block(w_lb_offset_index, 0, size_w_vec, 1);
      Eigen::MatrixXf exp_c_in_w = (-c_in * w_vec).array().exp();
      diag_A2 = -c_out * c_in * c_in * exp_c_in_w;
      b_2 = -grad_L.block(w_lb_offset_index, 0, size_w_vec, 1);
    }

    b_3 = diag_A2.cwiseProduct(b_1) + b_2;

    /**
           * Now: dz = A_2*A_1*dx + A_2*b_1 + b_2
           *         = A_2*A_1*dx + b_3
           * And: Jx_gx*dx + Ju_gx*du + Jy_gx*dy + Jz_gx*dz + gx = 0
           *
           **/
    // Update Jz_gx
    float sign_pos = 1.0f;
    Eigen::MatrixXf Jx_gx_update = -A_1.transpose() * diag_A2.asDiagonal() * A_1;
    insert_mtx(J_grad_L, Jx_gx_update, x_offset_index, x_offset_index, sign_pos);

    // Update gx
    grad_L.block(0, 0, size_x_vec, 1) += -A_1.transpose() * b_3;
  }

  size_ds_bar = std::max(0, (int) s_bar.rows() - size_z_vec - size_w_vec);
  Eigen::MatrixXd ds_bar_dbl(size_ds_bar, 1);

  Eigen::SparseMatrix<double> J_grad_L_mtx_sm;
  J_grad_L_mtx_sm.resize(size_ds_bar, size_ds_bar);

  // Once triplet list is created, reset the index list that keeps track of non-zero entries.
  reset_index_list(J_grad_L_triplet_list);

  // Convert triplet list to sparse matrix
  //populate_mtx(J_grad_L_mtx_sm, J_grad_L_triplet_list);

  //  Solve the newton step
  Eigen::MatrixXd J_grad_dm = J_grad_L.cast<double>();

  // Full piv LU
  Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(J_grad_dm);
  ds_bar_dbl = lu_decomp.solve(-1.0f * grad_L.block(0, 0, size_ds_bar, 1).cast<double>());

#ifdef COMPUTE_DW_EXTERNALLY
  // Copy out ds_bar
  Eigen::MatrixXf ds_opt(s_bar.rows(), 1);
  ds_opt.block(0, 0, ds_bar_dbl.rows(), 1) = ds_bar_dbl.cast<float>();

  // Compute dw and dz
  if (opt_params.flg_en_mpcConstraints) {
    ds_opt.block(w_lb_offset_index, 0, size_w_vec, 1) = A_1 * ds_opt.block(0, 0, size_x_vec, 1) + b_1;
    ds_opt.block(z_lb_offset_index, 0, size_z_vec, 1) =
        diag_A2.asDiagonal() * ds_opt.block(w_lb_offset_index, 0, size_w_vec, 1) + b_2;
  }
#else
  // Copy out ds_bar
  Eigen::MatrixXf ds_opt = ds_bar_dbl.cast<float>();
#endif

  step_size_ = 0.2;

  //  Compute step-size
  for (int index = 0; index < lookahead; index++) {
    compute_limiting_step_size(step_size_,
                               s_bar(u_offset_index + index, 0),
                               ds_opt(u_offset_index + index, 0),
                               -1.5f,
                               1.5f);
    compute_limiting_step_size(step_size_,
                               s_bar(v_offset_index + index, 0),
                               ds_opt(v_offset_index + index, 0),
                               -1.5f,
                               1.5f);
  }
  float w_min, z_min;
  if (USE_LOG_BARRIER_FUNC == true) {
    z_min = 0.0f;
    w_min = 0.001f;
  } else {
    z_min = 0.0f;
    w_min = -5000.0f;
  }

  if (opt_params.flg_en_mpcConstraints) {
    for (int index = 0; index < size_z_vec; index++) {
      compute_limiting_step_size(step_size_,
                                 s_bar(w_lb_offset_index + index, 0),
                                 ds_opt(w_lb_offset_index + index, 0),
                                 w_min,
                                 5000.0f);
      compute_limiting_step_size(step_size_,
                                 s_bar(z_lb_offset_index + index, 0),
                                 ds_opt(z_lb_offset_index + index, 0),
                                 z_min,
                                 5000.0f);
    }
  }

  if (iteration < 3) {
    step_size_ = 0.02;
  } else if (iteration < 6) {
    step_size_ = 0.05;
  } else {
    step_size_ = 0.2;
  }

  // Compute cos_th between gradient and ds_opt
  Eigen::MatrixXf norm2_grad_L = grad_L.transpose() * grad_L;
  Eigen::MatrixXf norm2_ds_opt = ds_opt.transpose() * ds_opt;
  Eigen::MatrixXf grad_L_ds_opt = grad_L.transpose() * ds_opt;

  if (norm2_grad_L.rows() != 1 || norm2_grad_L.cols() != 1) {
    LOG(WARNING) << "Not scalar.";
  }

  float f_norm_grad = norm2_grad_L(0, 0);
  float f_norm_ds = norm2_ds_opt(0, 0);
  float f_grad_L_ds_opt = grad_L_ds_opt(0, 0);
  float descent = f_grad_L_ds_opt / (f_norm_grad * f_norm_ds);

  norm_grad_L_ds_opt_vec.push_back(descent);

  Eigen::MatrixXf s_bar_init = s_bar;
  s_bar = s_bar + step_size_ * ds_opt;

  grad_L_im1 = grad_L;
  ds_opt_im1 = ds_opt;

  // Constrain updated values
  int theta_ind;
  for (int i = 0; i < lookahead; i++) {
    theta_ind = (i * 3) + 2;
    if (s_bar(theta_ind, 0) > 3.14159f) {
      s_bar(theta_ind, 0) = s_bar(theta_ind, 0) - 2 * 3.14159f;
    } else if (s_bar(theta_ind, 0) < -3.14159f) {
      s_bar(theta_ind, 0) = s_bar(theta_ind, 0) + 2 * 3.14159f;
    }
  }

  bool sign_flip = false;
  for (int i = 0; i < lookahead * CONTROL_SIZE; i++) {
    float v_max = fabs(v_desired(i));
    float v_min = 0.05;
    float w_max = opt_params.w_max;

    float w_cmd = s_bar(u_offset_index + i, 0);
    float v_cmd = s_bar(v_offset_index + i, 0);

    sign_flip = false;

    if (sign_flip) {
      // All the rest are incorrect?
      v_cmd = 0;
      w_cmd = 0;
      LOG(WARNING) << "MPC solver setting commands to zero. This should never happen.";
    } else if (getSign(v_cmd * v_desired(i)) < 0) {
      // Solution suggests driving in opposite direction
      v_cmd = getSign(v_desired(i)) * v_min;
      sign_flip = true;
      LOG_EVERY_N(10, WARNING)
          << "MPC solution suggested direction switch when none was requested. Setting speed to slow.";
      //w_cmd = -w_cmd;
    } else if (fabs(v_cmd) > v_min) {
      float des_curv = fabs(w_cmd / v_cmd);
      float lim_curv = fabs(w_max / v_max);

      if (des_curv > lim_curv && fabs(w_cmd) > w_max) {
        w_cmd = getSign(w_cmd) * w_max;
        v_cmd = getSign(v_cmd) * w_max / des_curv;
      } else if (des_curv <= lim_curv && fabs(v_cmd) > v_max) {
        v_cmd = getSign(v_cmd) * v_max;
        w_cmd = getSign(w_cmd) * v_max * des_curv;
      }
    }

    if (fabs(v_cmd) < 0.05) {
      v_cmd = getSign(v_desired(i)) * 0.05;
    }
    s_bar(u_offset_index + i, 0) = w_cmd;
    s_bar(v_offset_index + i, 0) = v_cmd;
  }

  if (sign_flip) {
    result_flgs.flg_des_vel_one_point_turn = true;
  }

  if (opt_params.flg_en_mpcConstraints) {
    for (int i = 0; i < size_z_vec; i++) {
      s_bar(w_lb_offset_index + i, 0) = std::max(w_min, (float) s_bar(w_lb_offset_index + i, 0));
      s_bar(z_lb_offset_index + i, 0) = std::max(z_min, (float) s_bar(z_lb_offset_index + i, 0));
    }
  }

  Eigen::MatrixXf delta = s_bar_init - s_bar;
  delta = delta.cwiseAbs();

  norm_ds_opt.push_back(delta.block(0, 0, size_x_vec + size_u_vec + size_v_vec, 1).sum());
  norm_ds.push_back(delta.block(0, 0, size_x_vec + size_u_vec + size_v_vec, 1).sum());

  /** Compute delta between x_pred and x_opt
    *  1) We optimize a cost function J(x_opt,u_opt), where x_opt and u_opt are sequences of states/inputs respectively
    *  2) We compute a sequence x_pred using the learned process model, where x_pred_kp1 = f(x_opt_k, u_opt_k), k = 0..K-1
    *  3) Through constraints, x_opt should equal x_pred
    **/

  if (size_x_vec > 0) {
    float max_delta = 0;
    float delta = 0;
    float delta_x_avg = 0;
    for (int i = 0; i < size_x_vec; i++) {
      delta = std::abs(s_bar(x_offset_index + i, 0) - x_target(i, 0));
      max_delta = std::max(delta, max_delta);

      delta_x_avg = delta_x_avg + delta;
    }
    delta_x_avg = delta_x_avg / size_x_vec;
    result_flgs.delta_x_pred_opt = delta_x_avg;

    if (delta_x_avg > 0.3f) {
      result_flgs.flg_x_opt_and_pred_dont_match = true;
    } else {
      result_flgs.flg_x_opt_and_pred_dont_match = false;
    }

  }

  // Copy x back into x_opt
  for (int i = 0; i < lookahead; i++) {
    int x_index = size_x * i;
    x_opt[i + 1].x_k = s_bar.block(x_index, 0, size_x, 1);
  }

}

void MpcSolverXUopt::compute_limiting_step_size(float &step_size, float &s_km1, float &ds, float lb, float ub) {

  float step_size_new;
  if (ds > 0) {
    float delta = s_km1 - lb;
    step_size_new = std::min(0.95f, std::max(0.1f, delta / ds));
  } else {
    float delta = s_km1 - ub;
    step_size_new = std::min(0.95f, std::max(0.1f, delta / ds));
  }
  step_size = std::min(step_size, step_size_new);
}

void MpcSolverXUopt::insert_mtx(mtx_triplet_list_t &triplet_list_out,
                                const Eigen::MatrixXf &gradients_in,
                                int &i,
                                int &j,
                                float sign /* = 1.0f */,
                                bool transpose /* = false */) {

  int num_rows = gradients_in.rows();
  int num_cols = gradients_in.cols();
  for (int col = 0; col < num_cols; col++) {
    for (int row = 0; row < num_rows; row++) {
      if (!transpose) {
        insert_triplet(triplet_list_out, row + i, col + j, gradients_in(row, col), sign);
      } else {
        insert_triplet(triplet_list_out, col + i, row + j, gradients_in(row, col), sign);
      }
    }
  }
}

void MpcSolverXUopt::insert_triplet_list(mtx_triplet_list_t &triplet_list_out,
                                         mtx_triplet_list_t &triplet_list_in,
                                         float &mult,
                                         int &i,
                                         int &j,
                                         float sign) {

  int i_new, j_new;
  float v_ij_new;

  for (unsigned triplet_ind = 0; triplet_ind < triplet_list_in.size(); triplet_ind++) {

    i_new = triplet_list_in[triplet_ind].i + i;
    j_new = triplet_list_in[triplet_ind].j + j;
    v_ij_new = triplet_list_in[triplet_ind].v_ij * mult;

    insert_triplet(triplet_list_out, i_new, j_new, v_ij_new, sign);
  }
}

void MpcSolverXUopt::insert_triplet(mtx_triplet_list_t &triplet_list_out,
                                    const int &i,
                                    const int &j,
                                    float v_ij,
                                    float sign /* = 1.0f */) {

  int index = s_bar.size() * i + j;
  int num_elements = s_bar.size() * s_bar.size();

  if (index > num_elements - 1 || index < 0) {
    LOG(INFO) << "Solver.insert_mtx:  Trying to add to (" << i << ", " << j << ") but mtx is of size ("
              << (int) s_bar.size() << ", " << (int) s_bar.size() << ").";
    LOG(INFO) << "Solver.insert_mtx:  Trying to add to element " << (int) index << " but num_elements = "
              << num_elements;
  }

  if (J_grad_L_mtx_entries_flg[index]) {
    if (sign > 0) {
      v_ij = triplet_list_out[J_grad_L_mtx_indices[index]].v_ij + v_ij;
    } else {
      v_ij = triplet_list_out[J_grad_L_mtx_indices[index]].v_ij - v_ij;
    }
    triplet_list_out[J_grad_L_mtx_indices[index]].v_ij = v_ij;

  } else {
    J_grad_L_mtx_entries_flg[index] = true;
    J_grad_L_mtx_indices[index] = triplet_list_out.size();
    if (sign > 0) {
      triplet_list_out.push_back(mtx_triplet(i, j, v_ij));
    } else {
      triplet_list_out.push_back(mtx_triplet(i, j, -v_ij));
    }
  }
}

void MpcSolverXUopt::insert_mtx(Eigen::MatrixXf &mtx,
                                const Eigen::MatrixXf &gradients_in,
                                int &i,
                                int &j,
                                float sign /* = 1.0f */,
                                bool transpose /* = false */) {

  if (!transpose) {
    if (sign > 0) {
      mtx.block(i, j, gradients_in.rows(), gradients_in.cols()) += gradients_in;
    } else {
      mtx.block(i, j, gradients_in.rows(), gradients_in.cols()) -= gradients_in;
    }
  } else {
    if (sign > 0) {
      mtx.block(i, j, gradients_in.cols(), gradients_in.rows()) += gradients_in.transpose();
    } else {
      mtx.block(i, j, gradients_in.cols(), gradients_in.rows()) -= gradients_in.transpose();
    }
  }
}

void MpcSolverXUopt::insert_triplet_list(Eigen::MatrixXf &mtx,
                                         mtx_triplet_list_t &triplet_list_in,
                                         float &mult,
                                         int &i,
                                         int &j,
                                         float sign) {

  int i_new, j_new;
  float v_ij_new;

  for (unsigned triplet_ind = 0; triplet_ind < triplet_list_in.size(); triplet_ind++) {

    i_new = triplet_list_in[triplet_ind].i + i;
    j_new = triplet_list_in[triplet_ind].j + j;
    v_ij_new = triplet_list_in[triplet_ind].v_ij * mult;

    insert_triplet(mtx, i_new, j_new, v_ij_new, sign);
  }
}

void MpcSolverXUopt::insert_triplet(Eigen::MatrixXf &mtx,
                                    const int &i,
                                    const int &j,
                                    float v_ij,
                                    float sign /* = 1.0f */) {

  int index = s_bar.size() * i + j;
  int num_elements = s_bar.size() * s_bar.size();

  if (index > num_elements - 1 || index < 0) {
    LOG(INFO) << "Solver.insert_mtx:  Trying to add to (" << i << ", " << j << ") but mtx is of size ("
              << (int) s_bar.size() << ", " << (int) s_bar.size() << ").";
    LOG(INFO) << "Solver.insert_mtx:  Trying to add to element " << (int) index << " but num_elements = "
              << num_elements;
  }

  if (sign > 0) {
    mtx(i, j) += v_ij;
  } else {
    mtx(i, j) -= v_ij;
  }
}

void MpcSolverXUopt::reset_index_list(mtx_triplet_list_t &triplet_list_in) {
  int index;
  for (unsigned i = 0; i < triplet_list_in.size(); i++) {
    index = s_bar.size() * triplet_list_in[i].i + triplet_list_in[i].j;
    J_grad_L_mtx_entries_flg[index] = false;
  }
}

void MpcSolverXUopt::update_mu_value(float &mu_value_in, int &mu_index_in) {
  mu_value_in = std::max(0.0001f, 0.25f * mu_value_in);
}

void MpcSolverXUopt::get_u(Eigen::MatrixXf &u_mat) {
  u_mat = s_bar.block(u_offset_index, 0, size_u_vec, 1);
}

void MpcSolverXUopt::get_v(Eigen::MatrixXf &v_mat) {
  v_mat = s_bar.block(v_offset_index, 0, size_v_vec, 1);
}

void MpcSolverXUopt::get_w(Eigen::MatrixXf &w_mat) {
  w_mat = s_bar.block(w_lb_offset_index, 0, size_w_vec, 1);
}

} // path_tracker
} // vtr
