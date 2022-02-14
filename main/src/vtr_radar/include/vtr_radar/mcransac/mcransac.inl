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

namespace vtr {
namespace radar {

struct vector_hash {
  size_t operator()(const vector<int> 
                    &myVector) const {
    std::hash<int> hasher;
    size_t answer = 0;
    for (int i : myVector) {
      answer ^= hasher(i) + 0x9e3779b9 + 
                (answer << 6) + (answer >> 2);
    }
    return answer;
  }
};

static void random_subset(int max_index, int subset_size,
  std::vector<int>& subset) {
  subset.clear();
  if (max_index < 0 || subset_size < 0)
    return subset;
  if (max_index < subset_size)
    subset_size = max_index;
  subset = std::vector<int>(subset_size, -1);
  for (uint i = 0; i < subset.size(); i++) {
    while (subset[i] < 0) {
      int idx = std::rand() % max_index;
      if (std::find(subset.begin(), subset.begin() + i, idx) == subset.begin() + i)
        subset[i] = idx;
    }
  }
}

template <class PointT>
void MCRANSAC::get_motion_parameters(const pcl::PointCloud<PointT> &pc1,
  const pcl::PointCloud<PointT> &pc2, const std::vector<int> &subset,
  Eigen::VectorXd &wbar) {
  const int N = pc1.size();
  const auto p1map = pc1.getMatrixXfMap(3, PointT::size(), PointT::cartesian_offset());
  const auto p2map = pc2.getMatrixXfMap(3, PointT::size(), PointT::cartesian_offset());
  wbar = Eigen::VectorXd::Zero(6, 1);
  // Run Gauss-Newton optimization
  double lastError = 0;
  for (int it = 0; it < max_gn_iterations_; ++it) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(6);
    for (uint m = 0; m < subset.size(); ++m) {
      const int sample_idx = subset[m];
      const double delta_t = pc2[sample_idx].t - pc1[sample_idx].t;
      const Eigen::MatrixXd T_1_2 = lgmath::se3::vec2tran(delta_t * wbar);  // delta_t * wbar = xi_2_1
      const auto &p1 = p1mat.block<3, 1>(0, sample_idx).cast<double>();
      const auto &p2 = p2mat.block<3, 1>(0, sample_idx).cast<double>();
      const Eigen::VectorXd gbar = T_1_2 * p2;
      const Eigen::MatrixXd G = delta_t * lgmath::se3::point2fs(gbar.block(0, 0, 3, 1));  // delta_t * circledot(gbar)
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
      for (uint m = 0; m < subset.size(); ++m) {
        const int sample_idx = subset[m];
        const double delta_t = pc2[sample_idx].t - pc1[sample_idx].t;
        const auto &p1 = p1mat.block<3, 1>(0, sample_idx).cast<double>();
        const auto &p2 = p2mat.block<3, 1>(0, sample_idx).cast<double>();
        const Eigen::MatrixXd T_1_2 = lgmath::se3::vec2tran(delta_t * wbar_temp);  // delta_t * wbar = xi_2_1
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
}

// pc1, pc2 inlcude x,y,z,t
// returns inlier ratio of MCRANSAC
template <class PointT>
double MCRANSAC<PointT>::run(const pcl::PointCloud<PointT> &pc1,
  const pcl::PointCloud<PointT> &pc2, Eigen::VectorXd &w_best,
  std::vector<int> &best_inliers) {
  assert(pc1.size() == pc2.size());
  const int N = pc1.size();
  // use unordered set to ensure uniqueness of subsets
  std::unordered_set<std::vector<int>>, vector_hash> unique_subsets;
  int max_iterations = std::min(iterations_, pow(N, subset_size_));
  int max_inliers = 0;
  int prev_size = 0;
  Eigen::VectorXd w_best = Eigen::VectorXd::Zero(6, 1);
  for (int i = 0; i < max_iterations; ++i) {
    // Retrieve a unique subset of point indices
    while (unique_subsets.size() == prev_size) {
      std::vector<int> subset;
      random_subset(N, subset_size_, subset);
      unique_subsets.insert(subset);
    }
    prev_size = unique_subsets.size();
    Eigen::VectorXd wbar;
    get_motion_parameters(pc1, pc2, unique_subsets[i], wbar);
    // Check number of inliers (RANSAC)
    int inliers = get_num_inliers(pc1, pc2, wbar);
    if (inliers > max_inliers) {
      max_inliers = inliers;
      w_best = wbar;
    }
    if (double(inliers) / N > inlier_ratio)
      break;
  }
  best_inliers.clear();
  get_inliers(pc1, pc2, w_best, best_inliers);
  get_motion_parameters(pc1, pc2, best_inliers, w_best);
  return double(best_inliers.size()) / N;
}

static int get_closest(const double x, const std::vector<double> &v) {
    const auto low = std::lower_bound(v.begin(), v.end(), x);
    int idx = low - v.begin();
    if (idx == 0)
      return idx;
    if (idx >= v.size())
      idx = v.size() - 1;
    double d = fabs(x - v[idx]);
    if (fabs(x - v[idx - 1]) < d)
      return idx - 1;
    return idx;
}

template <class PointT>
int MCRANSAC<PointT>::get_num_inliers(const pcl::PointCloud<PointT> &pc1,
  const pcl::PointCloud<PointT> &pc2, const Eigen::VectorXd &w_2_1) {
  std::vector<int> inliers;
  get_inliers(pc1, pc2, w_2_1, inliers);
  return inliers.size();
}

template <class PointT>
void MCRANSAC<PointT>::get_inliers(const pcl::PointCloud<PointT> &pc1,
  const pcl::PointCloud<PointT> &pc2, const Eigen::VectorXd &w_2_1,
  std::vector<int> &inliers) {
  inliers.clear();
  const int N = pc1.size();
  const auto p1map = pc1.getMatrixXfMap(3, PointT::size(), PointT::cartesian_offset());
  const auto p2map = pc2.getMatrixXfMap(3, PointT::size(), PointT::cartesian_offset());
  double min_delta = std::numeric_limits<double>::max();
  double max_delta = std::numeric_limits<double>::lowest();
  for (int i = 0; i < N; ++i) {
    const delta_t = pc2[i].t - pc1[i].t;
    if (delta_t < min_delta)
      min_delta = delta_t;
    if (delta_t > max_delta)
      max_delta = delta_t;
  }
  double delta_diff = (max_delta - min_delta) / (num_transforms_ - 1);
  // only compute a finite number of transforms (much faster)
  std::vector<double> delta_vec(num_transforms_);
  for (int i = 0; i < num_transforms_; ++i) {
    delta_vec[i] = min_delta + i * delta_diff;
  }
  std::vector<Eigen::MatrixXd> transforms_1_2(num_transforms);
  for (int i = 0; i < num_transforms_; ++i) {
    transforms_1_2[i] = vec2tran(delta_vec[i] * w_2_1);
  }
  for (int i = 0; i < N; ++i) {
    const double delta_t = pc2[i].t - pc1[i].t;
    const auto &p1 = p1mat.block<3, 1>(0, i).cast<double>();
    const auto &p2 = p2mat.block<3, 1>(0, i).cast<double>();
    Eigen::VectorXd error = p1 - transforms_1_2[get_closest(delta_t, delta_vec)] * p2;
    if (error.squaredNorm() < tolerance_)
      inliers.push_back(i);
  }
}

}  // namespace radar
}  // namespace vtr