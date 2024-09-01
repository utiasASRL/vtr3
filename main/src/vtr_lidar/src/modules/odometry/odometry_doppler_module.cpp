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
 * \file odometry_doppler_module.cpp
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/odometry/odometry_doppler_module.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"

#include <random>

namespace vtr {
namespace lidar {

namespace {

template <class PointT>
void cart2pol(pcl::PointCloud<PointT> &point_cloud) {
  for (auto &p : point_cloud) {
    p.rho = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    p.theta = std::atan2(std::sqrt(p.x * p.x + p.y * p.y), p.z);
    p.phi = std::atan2(p.y, p.x);
  }
}
}  // namespace

using namespace tactic;
using namespace steam;
using namespace steam::se3;
using namespace steam::traj;
using namespace steam::vspace;

auto OdometryDopplerModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // motion compensation
  config->use_trajectory_estimation = node->declare_parameter<bool>(param_prefix + ".use_trajectory_estimation", config->use_trajectory_estimation);
  config->traj_num_extra_states = node->declare_parameter<int>(param_prefix + ".traj_num_extra_states", config->traj_num_extra_states);
  config->traj_lock_prev_pose = node->declare_parameter<bool>(param_prefix + ".traj_lock_prev_pose", config->traj_lock_prev_pose);
  config->traj_lock_prev_vel = node->declare_parameter<bool>(param_prefix + ".traj_lock_prev_vel", config->traj_lock_prev_vel);
  const auto qcd = node->declare_parameter<std::vector<double>>(param_prefix + ".traj_qc_diag", std::vector<double>());
  if (qcd.size() != 6) {
    std::string err{"Qc diagonal malformed. Must be 6 elements!"};
    CLOG(ERROR, "lidar.odometry_doppler") << err;
    throw std::invalid_argument{err};
  }
  config->traj_qc_diag << qcd[0], qcd[1], qcd[2], qcd[3], qcd[4], qcd[5];

  // 
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  config->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", false);
  config->max_iterations = (unsigned int)node->declare_parameter<int>(param_prefix + ".max_iterations", 1);

  // doppler odom parameters
  config->num_sensors = node->declare_parameter<int>(param_prefix + ".num_sensors", config->num_sensors);
  config->ransac_seed = node->declare_parameter<long int>(param_prefix + ".ransac_seed", config->ransac_seed);
  config->ransac_gyro = node->declare_parameter<bool>(param_prefix + ".ransac_gyro", config->ransac_gyro);
  config->ransac_max_iter = node->declare_parameter<int>(param_prefix + ".ransac_max_iter", config->ransac_max_iter);
  config->ransac_min_range = node->declare_parameter<double>(param_prefix + ".ransac_min_range", config->ransac_min_range);
  config->ransac_thres = node->declare_parameter<double>(param_prefix + ".ransac_thres", config->ransac_thres);
  config->integration_steps = node->declare_parameter<int>(param_prefix + ".integration_steps", config->integration_steps);
  config->zero_vel_tol = node->declare_parameter<double>(param_prefix + ".zero_vel_tol", config->zero_vel_tol);

  config->min_dist = node->declare_parameter<double>(param_prefix + ".min_dist_lidar_center", config->min_dist);
  config->max_dist = node->declare_parameter<double>(param_prefix + ".max_dist_lidar_center", config->max_dist);
  
  const auto p0_inv = node->declare_parameter<std::vector<double>>(param_prefix + ".P0inv", std::vector<double>());
  if (p0_inv.size() != 6) {
    std::string err{"P0inv malformed. Must be 6 elements!"};
    CLOG(ERROR, "lidar.odometry_doppler") << err;
    throw std::invalid_argument{err};
  }
  config->P0inv = Eigen::DiagonalMatrix<double, 6>(p0_inv[0], p0_inv[1], p0_inv[2], p0_inv[3], p0_inv[4], p0_inv[5]);

  const auto qz_inv = node->declare_parameter<std::vector<double>>(param_prefix + ".Qzinv", std::vector<double>());
  if (qz_inv.size() != 6) {
    std::string err{"Qzinv diagonal malformed. Must be 6 elements!"};
    CLOG(ERROR, "lidar.odometry_doppler") << err;
    throw std::invalid_argument{err};
  }
  config->Qzinv = Eigen::DiagonalMatrix<double, 6>(qz_inv[0], qz_inv[1], qz_inv[2], qz_inv[3], qz_inv[4], qz_inv[5]);

  const auto qk_inv = node->declare_parameter<std::vector<double>>(param_prefix + ".Qkinv", std::vector<double>());
  if (qk_inv.size() != 6) {
    std::string err{"Qkinv diagonal malformed. Must be 6 elements!"};
    CLOG(ERROR, "lidar.odometry_doppler") << err;
    throw std::invalid_argument{err};
  }
  config->Qkinv = Eigen::DiagonalMatrix<double, 6>(qk_inv[0], qk_inv[1], qk_inv[2], qk_inv[3], qk_inv[4], qk_inv[5]);

  // clang-format on
  return config;
}

OdometryDopplerModule::OdometryDopplerModule(const Config::ConstPtr &config,
                      const std::shared_ptr<tactic::ModuleFactory> &module_factory,
                      const std::string &name)
                      : tactic::BaseModule(module_factory, name), config_(config) {  

  // precompute wnoa lhs
  Eigen::Matrix<double, 12, 6> temp_wnoa;
  temp_wnoa.topRows<6>() = -Eigen::Matrix<double, 6, 6>::Identity();
  temp_wnoa.bottomRows<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  wnoa_lhs_ = temp_wnoa * config_->Qkinv * temp_wnoa.transpose();

  // ransac generator
  random_engine_ = std::mt19937_64(size_t(config_->ransac_seed));
}

std::vector<Eigen::MatrixXd> OdometryDopplerModule::next_gyro(const double &start_time, const double &end_time, const std::vector<Eigen::MatrixXd> &gyro) {
  double dt = 0.1;
  std::vector<Eigen::MatrixXd> output;

  for (unsigned sensorid = 0; sensorid < gyro.size(); ++sensorid) {
    std::vector<int> inds; inds.clear();
    for (int r = 0; r < gyro[sensorid].rows(); ++r) {
      double meas_time = gyro[sensorid](r, 0) - dt;
      if (meas_time >= start_time && meas_time < end_time) {
        inds.push_back(r);
      }
    } // end for r

    if (inds.size() == 0) {
      // no measurements
      output.push_back(Eigen::Matrix<double, 1, 1>());  // 1x1 zero matrix
      continue;
    }

    Eigen::MatrixXd temp_gyro(inds.size(), 4);
    for (unsigned r = 0; r < inds.size(); ++r) {
      temp_gyro(r, 0) = gyro[sensorid](inds[r], 0) - dt; // timestamp
      temp_gyro.row(r).rightCols<3>() = gyro[sensorid].row(inds[r]).rightCols<3>();
    }
    output.push_back(temp_gyro);
  } // end for sensorid
  return output;
}

void OdometryDopplerModule::run_(QueryCache &qdata0, OutputCache &,
                             const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (!qdata.sliding_map_odo) {
    CLOG(INFO, "lidar.odometry_icp") << "First frame, simply return.";
    const Eigen::Matrix4d& identityMatrix = Eigen::Matrix4d::Identity();  
    // clang-format off
    // undistorted preprocessed point cloud
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.preprocessed_point_cloud);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;
    //
    qdata.timestamp_odo.emplace(*qdata.stamp);
    qdata.T_r_m_odo.emplace(EdgeTransform(identityMatrix, Eigen::Matrix<double, 6, 6>::Identity()));
    qdata.w_m_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    //
    *qdata.odo_success = true;
    // clang-format on
    EdgeTransform T_next(identityMatrix, Eigen::Matrix<double, 6, 6>::Identity());
    frame_count = 0;
    return;
  }

  CLOG(DEBUG, "lidar.odometry_doppler") << "Retrieve input data and setup evaluators.";

  frame_count += 1;

  // Inputs
  const auto &query_stamp = *qdata.stamp;
  const auto &query_points = *qdata.preprocessed_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &curr_gyro = *qdata.gyro;
  const auto &first_frame_micro = *qdata.initial_timestamp;
  const auto &next_state_micro = *qdata.next_state_time;
  const auto &timestamp_odo = *qdata.timestamp_odo; 
  const auto &T_r_m_odo = *qdata.T_r_m_odo;
  const auto &w_m_r_in_r_odo = *qdata.w_m_r_in_r_odo;
  
  /// Parameters
  // gyro noise
  gyro_invcov_.resize(config_->num_sensors);
  gyro_invcov_ = *qdata.gyro_invcov;

  // gyro bias
  const_gyro_bias_.resize(config_->num_sensors);
  const_gyro_bias_ = *qdata.const_gyro_bias;
  
  // clang-format off
  /// Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r); // at rear axle

  T_sv_.resize(config_->num_sensors);
  T_sv_[0] = T_s_r.matrix();
  // adjoint
  adT_sv_top3rows_.resize(config_->num_sensors);
  adT_sv_top3rows_[0] = lgmath::se3::tranAd(T_sv_[0]).topRows<3>();

  T_s_r_var->locked() = true;

  // current state time
  auto query_time = static_cast<int64_t>(query_stamp);
  // save last frame state
  auto prev_time = static_cast<int64_t>(timestamp_odo);
  auto prev_T_r_m_odo = *qdata.T_r_m_odo; 
  auto prev_w_m_r_in_r_odo = *qdata.w_m_r_in_r_odo;
  // next state time
  auto next_time = static_cast<int64_t>(next_state_micro);

  /// last frame state
  std::vector<SE3StateVar::Ptr> T_r_m_extp_vec;
  std::vector<VSpaceStateVar<6>::Ptr> w_m_r_in_r_vec;

  const_vel::Interface::Ptr trajectory = nullptr;
  if (config_->use_trajectory_estimation) {
    trajectory = const_vel::Interface::MakeShared(config_->traj_qc_diag);
    
    Time t_prev_time(static_cast<int64_t>(timestamp_odo));
    auto prev_T_r_m_var = SE3StateVar::MakeShared(T_r_m_odo);
    auto prev_w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(w_m_r_in_r_odo);
    if (config_->traj_lock_prev_pose) prev_T_r_m_var->locked() = true;
    if (config_->traj_lock_prev_vel) prev_w_m_r_in_r_var->locked() = true;
    trajectory->add(t_prev_time, prev_T_r_m_var, prev_w_m_r_in_r_var); 
  }

  if (prev_time == query_time && frame_count > 1) {
    CLOG(WARNING, "lidar.odometry_doppler") << "Skipping point cloud with duplicate stamp";
    *qdata.odo_success = false;
    return;
  }

  using Stopwatch = common::timing::Stopwatch<>;
  std::vector<std::unique_ptr<Stopwatch>> timer;
  std::vector<std::string> clock_str;
  clock_str.push_back("Ransac ............ ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Solve ............. ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Integration ....... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Undistortion ...... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));


  CLOG(DEBUG, "lidar.odometry_doppler") << "Starting Doppler Odometry";
  std::vector<double> total_time_timer(clock_str.size(), 0.0);

  auto compare_times = [](const auto &a, const auto &b) { return a.timestamp < b.timestamp; };
  auto min_max_time = std::minmax_element(query_points.begin(), query_points.end(), compare_times);
  auto first_frame_sec = first_frame_micro / 1e6;

  auto query_time_sec = (query_time / 1e9) - (first_frame_micro / 1e6);       // curr state start time in [s] 
  auto next_time_sec = (next_time / 1e9) - (first_frame_micro / 1e6);         // next state start time in [s]

  // *********** RANSAC ***********
  timer[0]->start();
  // precompute for full solve
  Eigen::Matrix<double, Eigen::Dynamic,6> ransac_precompute_all = Eigen::Matrix<double, Eigen::Dynamic, 6>(query_points.size(), 6);
  Eigen::Matrix<double, Eigen::Dynamic,1> meas_precompute_all = Eigen::Matrix<double, Eigen::Dynamic, 1>(query_points.size());

  // loop over each point to precompute
  for (size_t i = 0; i < query_points.size(); ++i) {
    // get x,y,z vector for solve
    Eigen::Vector3d point(query_points[i].x, query_points[i].y, query_points[i].z); 
    // the 'C' in y = C*x 
    ransac_precompute_all.row(i) = point.transpose()/query_points[i].rho * adT_sv_top3rows_[0];  // currently hardcoded for 1 sensor; to do: use active_lidars
    // the 'y' in y = C*x
    meas_precompute_all(i) = query_points[i].radial_velocity;
  } 

  // initialize uniform distribution
  std::uniform_int_distribution<int> uni_dist(0, query_points.size() - 1);

  // setup gyro (optional)
  Eigen::Matrix3d lhs_gyro = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs_gyro = Eigen::Vector3d::Zero();
  if (config_->ransac_gyro) {
    for (int i = 0; i < curr_gyro.size(); ++i) {
      if (curr_gyro[i].rows() <= 1 && curr_gyro[i].cols() <= 1)
        continue; // no data

      Eigen::Matrix3d R_sv = T_sv_[0].topLeftCorner<3, 3>();

      // loop over each gyro measurement
      for (int j = 0; j < curr_gyro[i].rows(); ++j) {
        
        double gy = (R_sv.transpose() * curr_gyro[i].row(j).rightCols<3>().transpose())(2); // rotate measurement to vehicle frame, extract z dim
        double gyvar = (R_sv.transpose() * gyro_invcov_[i] * R_sv)(2, 2); // rotate covariance, extract z dim
        lhs_gyro(2, 2) += gyvar;
        rhs_gyro(2) += gy / gyvar;
      }
    } // end for i
  }

  // ransac
  Eigen::Matrix3d lhs_ransac;
  Eigen::Vector3d rhs_ransac;
  Eigen::Vector3d G_ransac;
  int max_inliers = 0;

  Eigen::Array<bool, Eigen::Dynamic,1> inliers;       
  Eigen::Array<bool, Eigen::Dynamic,1> best_inliers;  
  Eigen::Vector3d curr_varpi;   // for debugging
  for (int iter = 0; iter < config_->ransac_max_iter; ++iter) {
    // setup linear system
    lhs_ransac.setZero();
    rhs_ransac.setZero();

    // gyro contribution (can be zero, i.e., not turned on)
    lhs_ransac += lhs_gyro;
    rhs_ransac += rhs_gyro;

    // sample until we satisfy min. range condition for ransac
    int sample1, sample2;
    for (int k = 0; k < 1e3; ++k) { // 1e3 is safety measure to prevent infinite loop
      sample1 = uni_dist(random_engine_);
      if (query_points[sample1].rho > config_->ransac_min_range)
        break;
    }
    
    for (int k = 0; k < 1e3; ++k) { // 1e3 is safety measure to prevent infinite loop
      sample2 = uni_dist(random_engine_);
      if (query_points[sample2].rho > config_->ransac_min_range)
        break;
    }

    // sample 1
    G_ransac(0) = ransac_precompute_all(sample1, 0);
    G_ransac(1) = ransac_precompute_all(sample1, 1);
    G_ransac(2) = ransac_precompute_all(sample1, 5);
    lhs_ransac += G_ransac * G_ransac.transpose();
    rhs_ransac += G_ransac * meas_precompute_all(sample1);

    // sample 2
    G_ransac(0) = ransac_precompute_all(sample2, 0);
    G_ransac(1) = ransac_precompute_all(sample2, 1);
    G_ransac(2) = ransac_precompute_all(sample2, 5);
    lhs_ransac += G_ransac * G_ransac.transpose();
    rhs_ransac += G_ransac * meas_precompute_all(sample2);

    // 2 DOF solve
    Eigen::Matrix2d lhs2d;
    lhs2d << lhs_ransac(0, 0), lhs_ransac(0, 2), lhs_ransac(2, 0), lhs_ransac(2, 2);
    if (fabs(lhs2d.determinant()) < 1e-4)
      continue; // not invertible

    Eigen::Vector2d rhs2d;
    rhs2d << rhs_ransac(0), rhs_ransac(2);

    Eigen::Vector2d varpi2d = lhs2d.inverse()*rhs2d;  // inverse should be fast for 2x2
    curr_varpi << varpi2d(0), 0.0, varpi2d(1);

    // calculate error and inliers 
    auto errors = meas_precompute_all - ransac_precompute_all.col(0)*curr_varpi(0) 
                                      - ransac_precompute_all.col(1)*curr_varpi(1) 
                                      - ransac_precompute_all.col(5)*curr_varpi(2);
    inliers = errors.array().abs() < config_->ransac_thres;
    int num_inliers = inliers.count();

    // check for improvement in number of inliers
    if (num_inliers > max_inliers) {
      max_inliers = num_inliers;
      best_inliers = inliers;
    }
  }

  // create new frame with only inliers
  pcl::PointCloud<PointWithInfo> inlier_frame; // check this
  inlier_frame.reserve(max_inliers);

  // precompute for full solve
  Eigen::Matrix<double, Eigen::Dynamic, 6> ransac_precompute_(max_inliers, 6);
  Eigen::Matrix<double, Eigen::Dynamic, 1> meas_precompute_(max_inliers, 1);
  Eigen::Matrix<double, Eigen::Dynamic, 1> alpha_precompute_(max_inliers, 1);
  Eigen::Matrix<double, Eigen::Dynamic, 1> malpha_precompute_(max_inliers, 1);
  Eigen::Matrix<double, Eigen::Dynamic, 1> ivariance_precompute_(max_inliers, 1);

  // loop over each measurement
  int k = 0;
  for (size_t i = 0; i < query_points.size(); ++i) {
    if (!best_inliers(i))
      continue; // skip since it's not an inlier

    inlier_frame.push_back(query_points[i]);
    ransac_precompute_.row(k) = ransac_precompute_all.row(i);
    meas_precompute_[k] = meas_precompute_all[i];
    alpha_precompute_[k] = std::min(1.0, std::max(0.0, (((query_points[i].timestamp / 1e9) - first_frame_sec - query_time_sec) / (next_time_sec - query_time_sec))));
    malpha_precompute_[k] = std::max(0.0, 1.0 - alpha_precompute_[k]);
    ivariance_precompute_[k] = query_points[i].ivariance;
    ++k;
  }
  timer[0]->stop();
  // end ransac

  timer[1]->start();
  // *********** SOLVE FRAME *********** 
  // we build a 12x12 linear system and marginalize to a 6x6 system to solve for the latest vehicle velocity 
  Eigen::Matrix<double, 12, 12> lhs = Eigen::Matrix<double, 12, 12>::Zero(); 
  Eigen::Matrix<double, 12, 1> rhs = Eigen::Matrix<double, 12, 1>::Zero();

  if (frame_count == 1) {
    // prior
    lhs.topLeftCorner<6, 6>() += config_->P0inv;
  } else {
    lhs.topLeftCorner<6, 6>() += last_lhs_;
    rhs.topLeftCorner<6, 1>() += last_rhs_;
  }

  // wnoa prior
  lhs += wnoa_lhs_;

  // zero velocity prior
  lhs.bottomRightCorner<6, 6>() += config_->Qzinv;

  // IMU measurements
  for (int i = 0; i < curr_gyro.size(); ++i) {
    if (curr_gyro[i].rows() <= 1 && curr_gyro[i].cols() <= 1){
      continue; // no data
    }
    Eigen::Matrix<double,3,6> Cgyro = Eigen::Matrix<double, 3, 6>::Zero();
    Cgyro.rightCols<3>() = T_sv_[0].topLeftCorner<3, 3>();
    Eigen::Matrix<double,3,12> Ggyro = Eigen::Matrix<double, 3, 12>::Zero();
    // loop over each gyro measurement
    for (int j = 0; j < curr_gyro[i].rows(); ++j) {
      double alpha = std::min(1.0, std::max(0.0, (curr_gyro[i](j, 0) - query_time_sec)/(next_time_sec - query_time_sec))); // times in [s]
      Ggyro.leftCols<6>() = (1.0 - alpha)*Cgyro;
      Ggyro.rightCols<6>() = alpha*Cgyro;

      lhs += Ggyro.transpose() * gyro_invcov_[i] * Ggyro;
      rhs += Ggyro.transpose() * gyro_invcov_[i] * (curr_gyro[i].row(j).rightCols<3>().transpose());
    }
  } // end for i

  // doppler measurements
  Eigen::Matrix<double, Eigen::Dynamic, 12> G(inlier_frame.size(), 12); // N x 12
  G.leftCols<6>() = ransac_precompute_.array().colwise() * malpha_precompute_.array();
  G.rightCols<6>() = ransac_precompute_.array().colwise() * alpha_precompute_.array();
  lhs += G.transpose() * (G.array().colwise() * ivariance_precompute_.array()).matrix();   
  rhs += G.transpose() * (meas_precompute_.array() * ivariance_precompute_.array()).matrix();

  // marginalize
  Eigen::Matrix<double, 6, 6> temp = lhs.bottomLeftCorner<6,6>()*lhs.topLeftCorner<6,6>().inverse();
  Eigen::Matrix<double, 6, 6> lhs_new = lhs.bottomRightCorner<6,6>() - temp*lhs.topRightCorner<6,6>();
  Eigen::Matrix<double, 6, 1> rhs_new = rhs.tail<6>() - temp*rhs.head<6>();

  // solve
  Eigen::Matrix<double, 6, 1> w_temp = lhs_new.llt().solve(rhs_new);  
  last_lhs_ = lhs_new;
  last_rhs_ = rhs_new;
  // end solve frame
  timer[1]->stop();

  timer[2]->start();
  // *********** NUM INTEGRATE ***********
  const Eigen::Matrix<double, 6, 6>& zeroMatrix = Eigen::Matrix<double, 6, 6>::Zero();
  const Eigen::Matrix4d& identityMatrix = Eigen::Matrix4d::Identity();  
  // get velocity knots
  Eigen::Matrix<double,6,1> knot1 = Eigen::Matrix<double,6,1>::Zero();
  double dt = 0.1;
  if (frame_count > 1) {
    knot1 = w_next;
    dt = (query_time - prev_time) / 1e9;
  }
  Eigen::Matrix<double,6,1> knot2 = w_temp;

  // mask to zero when stationary
  if (std::fabs(knot1(0)) < config_->zero_vel_tol)
    knot1 = Eigen::Matrix<double,6,1>::Zero();
  if (std::fabs(knot2(0)) < config_->zero_vel_tol)
    knot2 = Eigen::Matrix<double,6,1>::Zero();

  // integrate between the knots
  double dtt = dt/static_cast<double>(config_->integration_steps);
  Eigen::Matrix4d T_21 = Eigen::Matrix4d::Identity(); 
  for (int s = 1; s <= config_->integration_steps; ++s) {
    double t = s*dtt;
    double alpha = t/dt;
    Eigen::Matrix<double,6,1> vinterp = (1.0-alpha)*knot1 + alpha*knot2;
    T_21 = lgmath::se3::vec2tran(dtt*vinterp)*T_21;
  }

  EdgeTransform T_temp;
  // output latest pose
  if (frame_count == 1) {
    // T_temp = EdgeTransform(identityMatrix, lhs_new);
    T_temp = EdgeTransform(identityMatrix, Eigen::Matrix<double, 6, 6>::Identity());
  } else {
    // T_temp = EdgeTransform(T_21 * T_next, lhs_new);
    T_temp = EdgeTransform(T_21 * T_next, Eigen::Matrix<double, 6, 6>::Identity());
  }

  const auto T_r_m_eval = SE3StateVar::MakeShared(T_next);
  // compound transform for alignment (sensor to point map transform)
  const auto T_m_s_eval = inverse(compose(T_s_r_var, T_r_m_eval));

  *qdata.T_r_m_odo = T_r_m_eval->value();
  *qdata.w_m_r_in_r_odo = w_next;

  // end num integrate
  timer[2]->stop();

  // outputs - create shallow copy
  pcl::PointCloud<PointWithInfo> aligned_points(inlier_frame);
  const auto query_mat = inlier_frame.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto query_norms_mat = inlier_frame.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  timer[3]->start();

  if (frame_count > 1) {
    auto &sliding_map_odo = *qdata.sliding_map_odo;
    //auto &point_map = sliding_map_odo.point_cloud();
    if (config_->use_trajectory_estimation && (query_stamp != next_state_micro)) {
      Time knot_time(static_cast<int64_t>(query_stamp));
      const auto T_r_m_var = SE3StateVar::MakeShared(*qdata.T_r_m_odo);
      const auto w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(*qdata.w_m_r_in_r_odo);
      trajectory->add(knot_time, T_r_m_var, w_m_r_in_r_var);

      Time knot2_time(static_cast<int64_t>(next_state_micro));
      const auto T2_r_m_var = SE3StateVar::MakeShared(T_temp);
      const auto w2_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(w_temp);
      trajectory->add(knot2_time, T2_r_m_var, w2_m_r_in_r_var);   

#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
      for (unsigned i = 0; i < inlier_frame.size(); i++) {
        const auto &qry_time = inlier_frame[i].timestamp;
        const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
        const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
        const auto T_m_s = T_m_s_intp_eval->evaluate().matrix().cast<float>();
                
        aligned_mat.block<4, 1>(0, i) = T_m_s * query_mat.block<4, 1>(0, i);
        aligned_norms_mat.block<4, 1>(0, i) = T_m_s * query_norms_mat.block<4, 1>(0, i);
      }

    } else {
      const auto T_m_s = T_m_s_eval->evaluate().matrix().cast<float>();
      aligned_mat = T_m_s * query_mat;
      aligned_norms_mat = T_m_s * query_norms_mat;
    }

    // undistort the preprocessed pointcloud
    const auto T_s_m = T_m_s_eval->evaluate().matrix().inverse().cast<float>();
    aligned_mat = T_s_m * aligned_mat;
    aligned_norms_mat = T_s_m * aligned_norms_mat;
    
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(aligned_points);
    cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
    qdata.undistorted_point_cloud = undistorted_point_cloud;

    EdgeTransform T_r_v_dop(T_r_m_eval->value(), Eigen::Matrix<double, 6, 6>::Identity());

    *qdata.T_r_v_odo = T_r_v_dop * sliding_map_odo.T_vertex_this().inverse();

    /// \todo double check that we can indeed treat m same as v for velocity
    if (config_->use_trajectory_estimation)
      *qdata.w_v_r_in_r_odo = *qdata.w_m_r_in_r_odo;

    *qdata.odo_success = true;

  } else {
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(inlier_frame);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;
  }
  timer[3]->stop();

  *qdata.timestamp_odo = query_stamp;

  T_next = T_temp;
  w_next = w_temp;

  /// Dump timing info
  // timing for current frame
  CLOG(DEBUG, "lidar.odometry_doppler") << "Frame " << frame_count << "\nDump timing info inside loop: ";
  for (size_t i = 0; i < clock_str.size(); i++) {
    CLOG(DEBUG, "lidar.odometry_doppler") << "  " << clock_str[i] << timer[i]->count() / 1e6;
  }
  // elapsed timing
  CLOG(DEBUG, "lidar.odometry_doppler") << "Total timing info: ";
  for (size_t i = 0; i < timer.size(); ++i) {
    auto count = timer[i]->count();
    tot_timers[i] += count;
  }
  for (size_t i = 0; i < clock_str.size(); i++) {
      CLOG(DEBUG, "lidar.odometry_doppler") << "  " << clock_str[i] << tot_timers[i] / 1e6 / frame_count;
  }

  // clang-format on
}

}  // namespace lidar
}  // namespace vtr