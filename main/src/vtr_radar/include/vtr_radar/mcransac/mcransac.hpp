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
 * \file mcransac.hpp
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 * \brief Motion-compensated RANSAC implementation
 */
#pragma once

#include <random>

#include "lgmath.hpp"

#include "vtr_radar/types.hpp"

namespace vtr {
namespace radar {

//* MotionDistortedRansac
/**
 * \brief This class estimates the linear velocity and angular velocity of the
 * sensor in the body-frame.
 *
 * Assuming constant velocity, the motion vector can be used to estimate the
 * transform between any two pairs of points if the delta_t between those points
 * is known.
 *
 * A single transform between the two pointclouds can also be retrieved.
 *
 * All operations are done in SE(3) even if the input is 2D. The output motion
 * and transforms are in 3D.
 */
template <class PointT>
class MCRansac {
 public:
  MCRansac() : rng_(rd_()) {}
  MCRansac(const double tolerance, const double inlier_ratio,
           const int iterations, const int max_gn_iterations,
           const double epsilon_converge, const int subset_size)
      : tolerance_(tolerance),
        inlier_ratio_(inlier_ratio),
        iterations_(iterations),
        max_gn_iterations_(max_gn_iterations),
        epsilon_converge_(epsilon_converge),
        subset_size_(subset_size),
        rng_(rd_()) {}

  /**
   * \brief Computes the body-centric 6x1 velocity vector w_2_1. This can be
   * converted to a 4x4 homogeneous transform with:
   *   T_1_2 = vec2tran(delta_t * w_2_1) where delta_t = t_2 - t_1
   * \param[in] pc1
   * \param[in] pc2
   * \param[out] w_2_1
   * \param[out] best_inliers
   * \return inlier ratio of MCRANSAC
   */
  double run(const pcl::PointCloud<PointT> &pc1,
             const pcl::PointCloud<PointT> &pc2, Eigen::VectorXd &w_2_1,
             std::vector<int> &best_inliers);

 private:
  /**
   * \brief Given two matched pointclouds (pc1, pc2), this function computes the
   * motion of the sensor (linear and angular velocity) in the body frame using
   * nonlinear least squares.
   * \pre It's very important that the delt_t_local is accurate. Note that each
   * azimuth in the radar scan is time stamped, this should be used to get the
   * more accurate time differences.
   * \return motion estimate w_2_1
   */
  Eigen::VectorXd get_motion_parameters(const pcl::PointCloud<PointT> &pc1,
                                        const pcl::PointCloud<PointT> &pc2,
                                        const std::set<int> &subset);

  /** \brief Retrieve inliers corresponding to body motion vec wbar. (6 x 1) */
  std::vector<int> get_inliers(const pcl::PointCloud<PointT> &pc1,
                               const pcl::PointCloud<PointT> &pc2,
                               const Eigen::VectorXd &w_2_1);

 private:
  /** \brief Returns a random subset of size subset_size_ from [0, max_index] */
  std::set<int> random_subset(const int &max_index);

 private:
  const double tolerance_ = 0.1225;
  const double inlier_ratio_ = 0.9;
  const int iterations_ = 100;
  const int max_gn_iterations_ = 10;
  const double epsilon_converge_ = 0.0001;
  const int subset_size_ = 2;
  const double error_converge_ = 0.01;
  const double r_observable_sq = 0.0625;
  const int num_transforms_ = 21;

 private:
  std::random_device rd_;
  std::mt19937 rng_;
};

}  // namespace radar
}  // namespace vtr

#include "vtr_radar/mcransac/mcransac.inl"