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

  config->gauss_blur_sigma = node->declare_parameter<int>(param_prefix + ".gauss_blur_sigma", config->gauss_blur_sigma);
  config->gauss_blur_ksize = node->declare_parameter<int>(param_prefix + ".gauss_blur_ksize", config->gauss_blur_ksize);

  config->max_iter = node->declare_parameter<long>(param_prefix + ".max_iterations", config->max_iter);

  config->alpha = node->declare_parameter<double>(param_prefix + ".alpha", config->alpha);
  config->conv_tol = node->declare_parameter<double>(param_prefix + ".convergence_tol", config->conv_tol);

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

  if(!qdata.radar_data)
  {
    // This works if we have a last value - assuming we localized at least once
    // If not, just let loc_success be the default, which should be set to false by the pipeline
    return;
  }

  // Inputs
  const auto &local_scan = *qdata.smoothed_scan;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_r_v = *qdata.T_r_v_loc;  // used as prior
  const auto &T_v_m = *qdata.T_v_m_loc;
  auto &point_map = qdata.submap_loc->point_cloud();

  int ksize = config_->gauss_blur_ksize;
  cv::GaussianBlur(local_scan, local_scan, cv::Size(ksize, ksize), config_->gauss_blur_sigma);

  Eigen::MatrixXf local_map;
  cv::cv2eigen(local_scan, local_map);

  ba::LocalMapScan scan(0, 0, 0.1, config_->ba_opts, tactic::EdgeTransform(), tactic::EdgeTransform(), local_map);

  // Optimization loop
  double cost = 0.0;
  int final_iter = 0;
  int total_voxels_used = 0;

  for (int iter = 0; iter < config_->max_iter; ++iter) {
      final_iter = iter;
      Eigen::Matrix3d lhs = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      cost = 0.0;
      int num_voxels_used = 0;

      auto t_interp = std::chrono::high_resolution_clock::now();

      // Parallel voxel loop with per-thread accumulation
      std::vector<Eigen::Matrix3d> lhs_threads(config_->num_threads, Eigen::Matrix3d::Zero());
      std::vector<Eigen::Vector3d> rhs_threads(config_->num_threads, Eigen::Vector3d::Zero());
      std::vector<double> cost_threads(config_->num_threads, 0.0);
      std::vector<int> count_threads(config_->num_threads, 0);

      #pragma omp parallel for num_threads(config_->num_threads)
      for (size_t i = 0; i < point_map.size(); ++i) {
          int tid = omp_get_thread_num();
          const auto& point = point_map[i];
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
      total_voxels_used += num_voxels_used;

      if (num_voxels_used == 0 || lhs.determinant() == 0.0) {
          break;
      }

      Eigen::Vector3d delta = - config_->alpha * lhs.inverse() * rhs;
      scan.update_pose(delta);

      if (iter != 0 && delta.norm() < config_->conv_tol) break;
  }

  CLOG(INFO, static_name) << "Final pose\n" << scan.pose();

  /// Outputs
  if (final_iter < config_->max_iter) {
    // update map to robot transform
    *qdata.T_r_v_loc = T_s_r.inverse() * scan.pose() * T_v_m.inverse();
    // set success
    *qdata.loc_success = true;
  } else {
    CLOG(WARNING, static_name)
        << "DRL ran to maximum iterations. Localization is considered failed.";
    // no update to map to robot transform
    // set success
    *qdata.loc_success = false;
  }
  // clang-format on
}

}  // namespace radar
}  // namespace vtr