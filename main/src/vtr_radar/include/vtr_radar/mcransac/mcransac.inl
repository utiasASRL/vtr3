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
 * \file mcransac.inl
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 * \brief Motion-compensated RANSAC implementation
 */
#pragma once

#include "vtr_radar/mcransac/mcransac.hpp"

#include "vtr_common/utils/hash.hpp"

namespace vtr {
namespace radar {

namespace mcransac {

inline int get_closest(const double &x, const std::vector<double> &v) {
  const auto low = std::lower_bound(v.begin(), v.end(), x);
  int idx = low - v.begin();
  if (idx == 0) return idx;
  if ((size_t)idx >= v.size()) idx = v.size() - 1;
  double d = std::fabs(x - v[idx]);
  if (std::fabs(x - v[idx - 1]) < d) return idx - 1;
  return idx;
}

}  // namespace mcransac

template <class PointT>
std::set<int> MCRansac<PointT>::random_subset(const int &max_size) {
  size_t subset_size = std::min(max_size, subset_size_);

  std::set<int> subset;
  if (subset_size < ((size_t)max_size / 5)) {
    std::uniform_int_distribution<int> uniform_dist(0, max_size - 1);
    while (subset.size() < subset_size) subset.insert(uniform_dist(rng_));
  } else {
    std::vector<int> indices(max_size);
    std::iota(std::begin(indices), std::end(indices), 0);
    std::shuffle(std::begin(indices), std::end(indices), rng_);
    for (size_t i = 0; i < subset_size; ++i) subset.insert(indices[i]);
  }
  return subset;
}
// clang-format off
template <class PointT>
Eigen::VectorXd MCRansac<PointT>::get_motion_parameters(
    const pcl::PointCloud<PointT> &pc1, const pcl::PointCloud<PointT> &pc2,
    const std::set<int> &subset) {
  Eigen::VectorXd wbar = Eigen::VectorXd::Zero(6, 1);
  // Run Gauss-Newton optimization
  double lastError = 0;
  for (int it = 0; it < max_gn_iterations_; ++it) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(6);
    for (const auto& sample_idx: subset) {
      const double dt = pc2.at(sample_idx).time - pc1.at(sample_idx).time;
      const Eigen::MatrixXd T_1_2 = lgmath::se3::vec2tran(dt * wbar);  // dt * wbar = xi_2_1
      const Eigen::Vector4d p1 = {pc1.at(sample_idx).x, pc1.at(sample_idx).y, pc1.at(sample_idx).z, 1};
      const Eigen::Vector4d p2 = {pc2.at(sample_idx).x, pc2.at(sample_idx).y, pc2.at(sample_idx).z, 1};
      const Eigen::VectorXd gbar = T_1_2 * p2;
      const Eigen::MatrixXd G = dt * lgmath::se3::point2fs(gbar.block<3, 1>(0, 0));  // dt * circledot(gbar)
      const Eigen::VectorXd ebar = p1 - gbar;
      A += G.transpose() * G;
      b += G.transpose() * ebar;
    }
    const Eigen::VectorXd delta_w = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    // Line search for best update
    double minError = std::numeric_limits<double>::infinity();
    double bestAlpha = 1.0;
    for (double alpha = 0.1; alpha <= 1.0; alpha += 0.1) {
      double e = 0;
      const Eigen::VectorXd wbar_temp = wbar + alpha * delta_w;
      for (const auto& sample_idx: subset) {
        const double dt = pc2.at(sample_idx).time - pc1.at(sample_idx).time;
        const Eigen::Vector4d p1 = {pc1.at(sample_idx).x, pc1.at(sample_idx).y, pc1.at(sample_idx).z, 1};
        const Eigen::Vector4d p2 = {pc2.at(sample_idx).x, pc2.at(sample_idx).y, pc2.at(sample_idx).z, 1};
        const Eigen::MatrixXd T_1_2 = lgmath::se3::vec2tran(dt * wbar_temp);  // dt * wbar = xi_2_1
        const Eigen::VectorXd ebar = p1 - T_1_2 * p2;
        e += ebar.squaredNorm();
      }
      if (e < minError) {
        minError = e;
        bestAlpha = alpha;
      }
    }
    wbar = wbar + bestAlpha * delta_w;
    if (delta_w.squaredNorm() < epsilon_converge_)
      break;
    if (it > 0 && fabs((lastError - minError) / lastError) < error_converge_)
      break;
    lastError = minError;
  }

  return wbar;
}

// pc1, pc2 inlcude x,y,z,t
// returns inlier ratio of MCRANSAC
template <class PointT>
double MCRansac<PointT>::run(const pcl::PointCloud<PointT> &pc1,
                             const pcl::PointCloud<PointT> &pc2,
                             Eigen::VectorXd &w_best,
                             std::vector<int> &best_inliers) {
  if (pc1.size() != pc2.size())
    throw std::invalid_argument("matched point cloud size does not match.");

  const int N = pc1.size();

  // use unordered set to ensure uniqueness of subsets
  std::unordered_set<std::set<int>> unique_subsets;
  int max_iterations = std::min(iterations_, (int)std::pow(N, subset_size_));
  int max_inliers = 0;
  w_best = Eigen::VectorXd::Zero(6, 1);
  int i = 0;
  for (i = 0; i < max_iterations; ++i) {
    const auto subset = [&]{
      // We try 10000 times to get a unique subset, if we fail, we just return
      // an empty set
      for(int iter=0; iter<10000; ++iter) {
        const auto ret = unique_subsets.insert(random_subset(N));
        if (ret.second) return *ret.first;
      }
      return std::set<int>();
    }();
    if (subset.empty()) {
      CLOG(WARNING, "radar.mcransac") << "Failed to get another unique subset, reached max number of iteration.";
      break;
    }
    Eigen::VectorXd wbar = get_motion_parameters(pc1, pc2, subset);
    // Check number of inliers (RANSAC)
    const auto num_inliers = get_inliers(pc1, pc2, wbar).size();
    if ((int)num_inliers > max_inliers) {
      max_inliers = num_inliers;
      w_best = wbar;
    }
    if ((double(num_inliers) / N) > inlier_ratio_)
      break;
  }
  best_inliers = get_inliers(pc1, pc2, w_best);
  w_best = get_motion_parameters(pc1, pc2, std::set<int>(best_inliers.begin(), best_inliers.end()));
  std::cout << "mcransac: " << i << std::endl;
  return double(best_inliers.size()) / N;
}

template <class PointT>
std::vector<int> MCRansac<PointT>::get_inliers(
                                   const pcl::PointCloud<PointT> &pc1,
                                   const pcl::PointCloud<PointT> &pc2,
                                   const Eigen::VectorXd &w_2_1) {
  const int N = pc1.size();
  double min_delta = std::numeric_limits<double>::max();
  double max_delta = std::numeric_limits<double>::lowest();
  for (int i = 0; i < N; ++i) {
    const auto dt = pc2[i].time - pc1[i].time;
    if (dt < min_delta) min_delta = dt;
    if (dt > max_delta) max_delta = dt;
  }
  double delta_diff = (max_delta - min_delta) / (num_transforms_ - 1);
  // only compute a finite number of transforms (much faster)
  std::vector<double> delta_vec;
  std::vector<Eigen::Matrix4d> T_1_2_vec;
  delta_vec.reserve(num_transforms_);
  T_1_2_vec.reserve(num_transforms_);
  for (int i = 0; i < num_transforms_; ++i) {
    delta_vec.emplace_back(min_delta + i * delta_diff);
    T_1_2_vec.emplace_back(lgmath::se3::vec2tran(delta_vec.back() * w_2_1));
  }

  std::vector<int> inliers;
  for (int i = 0; i < N; ++i) {
    const double dt = pc2[i].time - pc1[i].time;
    const Eigen::Vector4d p1 = {pc1.at(i).x, pc1.at(i).y, pc1.at(i).z, 1};
    const Eigen::Vector4d p2 = {pc2.at(i).x, pc2.at(i).y, pc2.at(i).z, 1};
    const Eigen::Vector4d error = p1 - T_1_2_vec[mcransac::get_closest(dt, delta_vec)] * p2;
    if (error.squaredNorm() < tolerance_)
      inliers.emplace_back(i);
  }
  return inliers;
}

}  // namespace radar
}  // namespace vtr