// Copyright 2026, Autonomous Space Robotics Lab (ASRL)
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
 * \file localization_icp_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/localization/direct_localization_module.hpp"

#include <algorithm>
#include <cmath>

#include "vtr_common/conversions/se2_to_se3.hpp"
#include "vtr_radar/dr_ba/scans/local_map_scan.hpp"

namespace vtr {
namespace radar {

using namespace tactic;
using namespace common::conversions;


auto DirectLocalizationModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                            const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->use_pose_prior = node->declare_parameter<bool>(param_prefix + ".use_pose_prior", config->use_pose_prior);
  std::vector<double> prior_cov_diag = node->declare_parameter<std::vector<double>>(param_prefix + ".prior_cov_diag", {});
  if (config->use_pose_prior && prior_cov_diag.size() != 3) {
    CLOG(ERROR, static_name) << "Pose prior covariance did not have three values as required";
  }
  config->prior_cov << Eigen::Map<Eigen::Vector3d>(prior_cov_diag.data());

  config->max_iter = node->declare_parameter<long>(param_prefix + ".max_iterations", config->max_iter);

  config->alpha = node->declare_parameter<double>(param_prefix + ".alpha", config->alpha);
  config->conv_tol = node->declare_parameter<double>(param_prefix + ".convergence_tol", config->conv_tol);
  config->max_dist = node->declare_parameter<double>(param_prefix + ".max_dist", config->max_dist);

  config->ba_opts.meas_std = node->declare_parameter<double>(param_prefix + ".meas_std", config->ba_opts.meas_std);
  config->ba_opts.range_factor = node->declare_parameter<double>(param_prefix + ".range_factor", config->ba_opts.range_factor);

  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);
  // clang-format on
  return config;
}

void DirectLocalizationModule::run_(QueryCache &qdata0, OutputCache &output,
                                 const Graph::Ptr &,
                                 const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  if(!qdata.smoothed_scan || !qdata.smoothed_scan_res)
  {
    return;
  }

  // Inputs
  const auto &local_scan = *qdata.smoothed_scan;
  const auto scan_res = *qdata.smoothed_scan_res;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_r_v = *qdata.T_r_v_loc;  // used as prior
  const auto &T_v_m = *qdata.T_v_m_loc;
  auto &point_map = qdata.submap_loc->point_cloud();

  // Convert the local scan to a float matrix for the DRL optimization
  cv::Mat prepared;
  local_scan.convertTo(prepared, CV_32F, 1.0 / 255.0);

  Eigen::MatrixXf local_map;
  cv::cv2eigen(prepared, local_map);

  // Force SE2 to avoid complaining about SE3 conversions
  const auto T_m_s_init = (T_s_r * T_r_v * T_v_m).inverse().toSE2().toSE3();

  CLOG(DEBUG, static_name) << "Initial guess\n" << T_m_s_init;
  ba::LocalMapScan scan(*qdata.stamp, 0, scan_res, config_->ba_opts, T_m_s_init, tactic::EdgeTransform(), local_map);

  // Crop the map once around the initial guess, which the estimate shoudl
  // stay within over the optimization
  std::vector<size_t> active_points;
  active_points.reserve(point_map.size());
  {
    const Eigen::Vector3d r_m_s = T_m_s_init.r_ab_inb();
    const double max_dist_sq = std::pow(config_->max_dist, 2);
    for (size_t i = 0; i < point_map.size(); ++i) {
      const double dx = point_map[i].x - r_m_s(0);
      const double dy = point_map[i].y - r_m_s(1);
      if (config_->max_dist <= 0.0 || dx * dx + dy * dy <= max_dist_sq)
        active_points.push_back(i);
    }
  }
  CLOG(DEBUG, static_name) << "Using " << active_points.size() << " of " << point_map.size() << " map voxels";

  // Optimization loop
  double cost = 0.0;
  int num_iters = 0;
  bool converged = false;
  bool solve_failed = false;

  for (int iter = 0; iter < config_->max_iter; ++iter) {
      Eigen::Matrix3d lhs = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      cost = 0.0;
      int num_voxels_used = 0;

      // Parallel voxel loop with per-thread accumulation
      std::vector<Eigen::Matrix3d> lhs_threads(config_->num_threads, Eigen::Matrix3d::Zero());
      std::vector<Eigen::Vector3d> rhs_threads(config_->num_threads, Eigen::Vector3d::Zero());
      std::vector<double> cost_threads(config_->num_threads, 0.0);
      std::vector<int> count_threads(config_->num_threads, 0);

      #pragma omp parallel for num_threads(config_->num_threads)
      for (size_t i = 0; i < active_points.size(); ++i) {
          int tid = omp_get_thread_num();
          const auto& point = point_map[active_points[i]];
          double voxel_x = point.x;
          double voxel_y = point.y;
          double vox_intensity = point.intensity;

          std::optional<ba::Scan::Measurement> interp_meas = scan.interpolate(voxel_x, voxel_y);
          if (!interp_meas.has_value()) {
              continue;
          }

          double I_meas = interp_meas->intensity;
          double meas_cov = interp_meas->covariance;
          double err_weight = 1.0 / meas_cov;

          double err = (vox_intensity - I_meas);
          double err_weight_sqrt = std::sqrt(err_weight);

          Eigen::Matrix<double, 1, 3> d_beta_d_T = - interp_meas->jacobian * err_weight_sqrt;
          err *= err_weight_sqrt;

          lhs_threads[tid] += d_beta_d_T.transpose() * d_beta_d_T;
          rhs_threads[tid] += d_beta_d_T.transpose() * err;
          cost_threads[tid] += 0.5 * std::pow(err, 2);
          count_threads[tid]++;
      }

      // Merge thread results
      for (int t = 0; t < config_->num_threads; ++t) {
          lhs += lhs_threads[t];
          rhs += rhs_threads[t];
          cost += cost_threads[t];
          num_voxels_used += count_threads[t];
      }
      num_iters = iter + 1;

      if(config_->use_pose_prior) {
        //Pose prior cost
        const auto xi_total = (T_m_s_init.inverse() * scan.pose()).toSE2().vec();
        const auto J = lgmath::se2::vec2jac(xi_total);
        lhs += J.transpose() * J;
        cost += 0.5 * config_->prior_cov.transpose() * xi_total.array().square().matrix();
        rhs += J * (config_->prior_cov.array().sqrt() * xi_total.array()).matrix();
      }
     
      if (num_voxels_used == 0 || lhs.determinant() == 0.0) {
          solve_failed = true;
          break;
      }

      Eigen::Vector3d delta = - config_->alpha * lhs.inverse() * rhs;
      scan.update_pose(delta);

      if (iter != 0 && delta.norm() < config_->conv_tol) {
          converged = true;
          break;
      }
  }

  // Per-frame summary: whether the step norm dropped below conv_tol, how many
  // iterations that took, and how far the solve pulled away from the prior
  {
    const auto T_moved = (T_m_s_init.inverse() * scan.pose()).toSE2();
    CLOG(DEBUG, static_name)
        << "DRL converged: " << (converged ? "true" : "false") << " after "
        << num_iters << "/" << config_->max_iter << " iters | cost " << cost
        << " | moved " << T_moved.r_ab_inb().norm() << " m "
        << T_moved.vec()(2) * 180.0 / M_PI << " deg from prior";
  }

  CLOG(INFO, static_name) << "Final pose\n" << scan.pose();

  /// Outputs
  if (!solve_failed) {
    // update map to robot transform
    *qdata.T_r_v_loc = (T_v_m * scan.pose() * T_s_r).inverse();
    // set success
    *qdata.loc_success = true;
  } else {
    CLOG(WARNING, static_name) << "DRL solve failed.";
    // no update to map to robot transform
    // set success
    *qdata.loc_success = false;
  }
  // clang-format on
}

}  // namespace radar
}  // namespace vtr