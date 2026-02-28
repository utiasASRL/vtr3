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
 * \file odometry_gt_module.cpp
 * \author Daniil Lisus, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/odometry_gt_module.hpp"

#include "vtr_radar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace radar {

namespace {

template <class PointT>
void cart2pol(pcl::PointCloud<PointT> &point_cloud) {
  for (auto &p : point_cloud) {
    p.rho = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    p.theta = std::atan2(std::sqrt(p.x * p.x + p.y * p.y), p.z);
    p.phi = std::atan2(p.y, p.x);
  }
}

Eigen::Vector<double, 3> vec3Dto2D(const Eigen::Matrix<double, 6, 1> &se3_vec) {
  Eigen::Vector3d se2_vec;
  se2_vec << se3_vec(0), se3_vec(1), se3_vec(5);
  return se2_vec;
}

Eigen::Vector<double, 6> vec2Dto3D(const Eigen::Matrix<double, 3, 1> &se2_vec) {
  Eigen::Vector<double, 6> se3_vec = Eigen::Vector<double, 6>::Zero();
  se3_vec << se2_vec(0), se2_vec(1), 0, 0, 0, se2_vec(2);
  return se3_vec;
}

}  // namespace

using namespace tactic;
using namespace steam;
using namespace steam::se2;
using namespace steam::traj;
using namespace steam::vspace;

auto OdometryGTModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();

  // Undistortion trajectory parameters
  const auto qcd = node->declare_parameter<std::vector<double>>(param_prefix + ".traj_qc_diag", std::vector<double>());
  if (qcd.size() != 3) {
    std::string err{"Qc diagonal malformed. Must be 3 elements!"};
    CLOG(ERROR, "radar.odometry_icp") << err;
    throw std::invalid_argument{err};
  }
  config->traj_qc_diag << qcd[0], qcd[1], qcd[2];
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);

  return config;
}

void OdometryGTModule::run_(QueryCache &qdata0, OutputCache &,
                             const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  // Do nothing if qdata does not contain any radar data (was populated by gyro)
  if(!qdata.radar_data)
  {
    return;
  }

  if (!qdata.T_s_world_gt || !qdata.v_s_gt) {
    CLOG(ERROR, "radar.odometry_gt")
        << "Groundtruth pose or velocity not provided!";
    throw std::runtime_error("Groundtruth pose or velocity not provided!");
  }

  if (!qdata.sliding_map_odo) {
    // Initialize all variables
    CLOG(INFO, "radar.odometry_gt") << "First frame, simply return.";
    // clang-format off
    // undistorted preprocessed point cloud
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.preprocessed_point_cloud);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;
    //
    qdata.T_r_m_odo.emplace(EdgeTransform(true));
    qdata.T_r_m_odo_radar.emplace(EdgeTransform(true));

    // Initialize groundtruth variables
    qdata.T_s_world_gt_prev.emplace(*qdata.T_s_world_gt);
    qdata.v_s_gt_prev.emplace(*qdata.v_s_gt);

    // Initialize prior values
    qdata.T_r_m_odo_prior.emplace(lgmath::se3::Transformation());
    qdata.timestamp_prior.emplace(*qdata.stamp);

    *qdata.odo_success = true;
    // clang-format on

    // This is the first odometry frame
    if(qdata.first_frame)
      *qdata.first_frame = true;
    else
      qdata.first_frame.emplace(true); // reset first frame - this is the first frame! Gyro could have run before though

    return;
  }

  CLOG(DEBUG, "radar.odometry_gt")
      << "Retrieve input data and setup evaluators.";

  // Inputs (these are all 3D to be consistent with other pipelines)
  const auto &scan_stamp = *qdata.stamp;
  const auto &query_points = *qdata.preprocessed_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_r_m_odo_prev = *qdata.T_r_m_odo_prior;
  const auto &beta = *qdata.beta;
  const auto &T_s_world_gt = *qdata.T_s_world_gt;
  const auto &v_s_gt = *qdata.v_s_gt;
  const auto &T_s_world_gt_prev = *qdata.T_s_world_gt_prev;
  const auto &v_s_gt_prev = *qdata.v_s_gt_prev;
  const auto &timestamp_prev = *qdata.timestamp_prior;

  // Compute velocity in robot frame
  const auto Ad_T_r_s = lgmath::se2::tranAd(T_s_r.inverse().toSE2().matrix());
  const auto v_s_gt_2d = vec3Dto2D(v_s_gt);
  const auto v_r_gt_2d = Ad_T_r_s * v_s_gt_2d;
  const auto v_s_gt_prev_2d = vec3Dto2D(v_s_gt_prev);
  const auto v_r_gt_prev_2d = Ad_T_r_s * v_s_gt_prev_2d;

  // Compute odometry change from groundtruth
  auto T_r_s = T_s_r.inverse();
  auto del_T_r = (T_r_s * T_s_world_gt * T_s_world_gt_prev.inverse() * T_s_r).toSE2().toSE3();

  // Compute new odometry at radar scan time
  auto T_r_m_new = del_T_r * T_r_m_odo_prev;

  // Create trajectory for undistortion
  const_vel_se2::Interface::Ptr trajectory = const_vel_se2::Interface::MakeShared(config_->traj_qc_diag);
  // Add poses and velocities to trajectory
  trajectory->add(Time(timestamp_prev), SE2StateVar::MakeShared(T_r_m_odo_prev.toSE2()),
                  VSpaceStateVar<3>::MakeShared(v_r_gt_prev_2d));
  trajectory->add(Time(scan_stamp), SE2StateVar::MakeShared(T_r_m_new.toSE2()),
                  VSpaceStateVar<3>::MakeShared(v_r_gt_2d));
  
  // Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE2StateVar::MakeShared(T_s_r.toSE2());
  T_s_r_var->locked() = true;

  // Compound transform for alignment (sensor to point map transform)
  Evaluable<lgmath::se2::Transformation>::ConstPtr T_r_m_eval = trajectory->getPoseInterpolator(scan_stamp);
  const auto T_m_s_eval = inverse(compose(T_s_r_var, T_r_m_eval));

  // Initialize aligned points for matching (Deep copy of targets)
  pcl::PointCloud<PointWithInfo> aligned_points(query_points);

  // Eigen matrix of original data (only shallow copy of ref clouds)
  const auto query_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto query_norms_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  /// perform initial alignment
  CLOG(DEBUG, "radar.odometry_gt") << "Start initial alignment.";
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
  for (unsigned i = 0; i < query_points.size(); ++i) {
    aligned_mat.block<4, 1>(0, i) = query_mat.block<4, 1>(0, i);
  }
  if (beta != 0) {
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (unsigned i = 0; i < query_points.size(); ++i) {
      const auto &qry_time = query_points[i].timestamp;
      const auto &up_chirp = query_points[i].up_chirp;
      const auto w_m_r_in_r_intp_eval = trajectory->getVelocityInterpolator(Time(qry_time));
      const auto w_m_s_in_s_intp_eval = compose_velocity(T_s_r_var, w_m_r_in_r_intp_eval);
      const auto w_m_s_in_s = w_m_s_in_s_intp_eval->evaluate().matrix().cast<float>();
      // Still create 3D v_m_s_in_s but just with a 0 z component
      const Eigen::Vector3f v_m_s_in_s = Eigen::Vector3f(w_m_s_in_s(0), w_m_s_in_s(1), 0.0f);
      Eigen::Vector3f abar = aligned_mat.block<3, 1>(0, i);
      abar.normalize();
      // If up chirp azimuth, subtract Doppler shift
      if (up_chirp) {
        aligned_mat.block<3, 1>(0, i) -= beta * abar * abar.transpose() * v_m_s_in_s;
      } else {
        aligned_mat.block<3, 1>(0, i) += beta * abar * abar.transpose() * v_m_s_in_s;
      }
    }
  }
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
  for (unsigned i = 0; i < query_points.size(); i++) {
    const auto &qry_time = query_points[i].timestamp;
    const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
    const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
    // Transform to 3D for point manipulation
    const auto T_m_s = T_m_s_intp_eval->evaluate().toSE3().matrix().cast<float>();
    aligned_mat.block<4, 1>(0, i) = T_m_s * aligned_mat.block<4, 1>(0, i);
    aligned_norms_mat.block<4, 1>(0, i) = T_m_s * query_norms_mat.block<4, 1>(0, i);
  }

  // undistort the preprocessed pointcloud to eval state (at query timestamp)
  const auto T_s_m = T_m_s_eval->evaluate().toSE3().matrix().inverse().cast<float>();
  aligned_mat = T_s_m * aligned_mat;
  aligned_norms_mat = T_s_m * aligned_norms_mat;

  auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(aligned_points);
  cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
  qdata.undistorted_point_cloud = undistorted_point_cloud;

  // Update results
  *qdata.T_r_v_odo = del_T_r * *qdata.T_r_v_odo;
  // Set small diagonal covariance
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
  qdata.T_r_v_odo->setCovariance(cov);
  *qdata.T_r_m_odo = T_r_m_new;
  *qdata.T_r_m_odo_radar = T_r_m_new;
  *qdata.w_v_r_in_r_odo = vec2Dto3D(v_r_gt_2d);

  // Update groundtruth previous values
  *qdata.T_s_world_gt_prev = T_s_world_gt;
  *qdata.v_s_gt_prev = v_s_gt;

  // Update prior values for next iteration
  *qdata.T_r_m_odo_prior = T_r_m_new;
  *qdata.timestamp_prior = scan_stamp;

  //
  *qdata.odo_success = true;
  CLOG(DEBUG, "radar.odometry_gt") << "T_r_v_odo: " << *qdata.T_r_v_odo;
}

}  // namespace radar
}  // namespace vtr