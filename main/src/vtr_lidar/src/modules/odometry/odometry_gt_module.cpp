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
#include "vtr_lidar/modules/odometry/odometry_gt_module.hpp"

#include "vtr_lidar/utils/nanoflann_utils.hpp"

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

auto OdometryGTModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();

  // Undistortion trajectory parameters
  const auto qcd = node->declare_parameter<std::vector<double>>(param_prefix + ".traj_qc_diag", std::vector<double>());
  if (qcd.size() != 6) {
    std::string err{"Qc diagonal malformed. Must be 6 elements!"};
    CLOG(ERROR, "lidar.odometry_gt") << err;
    throw std::invalid_argument{err};
  }
  config->traj_qc_diag << qcd[0], qcd[1], qcd[2], qcd[3], qcd[4], qcd[5];
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);

  return config;
}

void OdometryGTModule::run_(QueryCache &qdata0, OutputCache &,
                             const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (!qdata.T_s_world_gt || !qdata.v_s_gt) {
    CLOG(ERROR, "lidar.odometry_gt")
        << "Groundtruth pose or velocity not provided!";
    throw std::runtime_error("Groundtruth pose or velocity not provided!");
  }

  if (!qdata.sliding_map_odo) {
    CLOG(INFO, "lidar.odometry_gt") << "First frame, simply return.";
    // clang-format off
    // undistorted preprocessed point cloud
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.preprocessed_point_cloud);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;
    //
    qdata.T_r_m_odo.emplace(EdgeTransform(true));

    // Initialize groundtruth variables
    qdata.T_s_world_gt_prev.emplace(*qdata.T_s_world_gt);
    qdata.v_s_gt_prev.emplace(*qdata.v_s_gt);

    // Initialize prior values
    qdata.T_r_m_odo_prior.emplace(lgmath::se3::Transformation());
    qdata.timestamp_prior.emplace(*qdata.stamp);

    *qdata.odo_success = true;
    // clang-format on
    return;
  }

  CLOG(DEBUG, "lidar.odometry_gt")
      << "Retrieve input data and setup evaluators.";

  // Inputs

  const auto &scan_stamp = *qdata.stamp;
  const auto &query_points = *qdata.preprocessed_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_r_m_odo_prev = *qdata.T_r_m_odo_prior;
  const auto &T_s_world_gt = *qdata.T_s_world_gt;
  const auto &v_s_gt = *qdata.v_s_gt;
  const auto &T_s_world_gt_prev = *qdata.T_s_world_gt_prev;
  const auto &v_s_gt_prev = *qdata.v_s_gt_prev;
  const auto &timestamp_prev = *qdata.timestamp_prior;

  // Compute velocity in robot frame
  const auto Ad_T_r_s = lgmath::se3::tranAd(T_s_r.inverse().matrix());
  const auto v_r_gt = Ad_T_r_s * v_s_gt;
  const auto v_r_gt_prev = Ad_T_r_s * v_s_gt_prev;

  // Compute odometry change from groundtruth
  auto T_r_s = T_s_r.inverse();
  auto del_T_r = T_r_s * T_s_world_gt * T_s_world_gt_prev.inverse() * T_s_r;

  // Compute new odometry at radar scan time
  auto T_r_m_new = del_T_r * T_r_m_odo_prev;

  // Create trajectory for undistortion
  const_vel::Interface::Ptr trajectory = const_vel::Interface::MakeShared(config_->traj_qc_diag);
  // Add poses and velocities to trajectory
  trajectory->add(Time(timestamp_prev), SE3StateVar::MakeShared(T_r_m_odo_prev), VSpaceStateVar<6>::MakeShared(v_r_gt_prev));
  trajectory->add(Time(scan_stamp), SE3StateVar::MakeShared(T_r_m_new), VSpaceStateVar<6>::MakeShared(v_r_gt));

  // Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r);
  T_s_r_var->locked() = true;

  // Compound transform for alignment (sensor to point map transform)
  Evaluable<lgmath::se3::Transformation>::ConstPtr T_r_m_eval = trajectory->getPoseInterpolator(scan_stamp);
  const auto T_m_s_eval = inverse(compose(T_s_r_var, T_r_m_eval));

  /// Initialize aligned points for matching (Deep copy of targets)
  pcl::PointCloud<PointWithInfo> aligned_points(query_points);

  /// Eigen matrix of original data (only shallow copy of ref clouds)
  const auto query_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto query_norms_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  /// perform initial alignment
  CLOG(DEBUG, "lidar.odometry_gt") << "Start initial alignment.";
  #pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
  for (unsigned i = 0; i < query_points.size(); i++) {
    const auto &qry_time =  query_points[i].timestamp;
    const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
    const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
    const auto T_m_s = T_m_s_intp_eval->evaluate().matrix().cast<float>();
    aligned_mat.block<4, 1>(0, i) = T_m_s * query_mat.block<4, 1>(0, i);
    aligned_norms_mat.block<4, 1>(0, i) = T_m_s * query_norms_mat.block<4, 1>(0, i);
  }

  // undistort the preprocessed pointcloud
  const auto T_s_m = T_m_s_eval->evaluate().matrix().inverse().cast<float>();
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
  *qdata.w_v_r_in_r_odo = v_r_gt;

  // Update groundtruth previous values
  *qdata.T_s_world_gt_prev = T_s_world_gt;
  *qdata.v_s_gt_prev = v_s_gt;

  // Reuse the prior variables to pass previous groundtruth
  *qdata.T_r_m_odo_prior = T_r_m_new;
  *qdata.timestamp_prior = scan_stamp;

  //
  *qdata.odo_success = true;
  CLOG(DEBUG, "lidar.odometry_gt") << "T_r_v_odo: " << *qdata.T_r_v_odo;
}

}  // namespace lidar
}  // namespace vtr