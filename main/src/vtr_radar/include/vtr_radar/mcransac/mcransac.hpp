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

#include "lgmath.hpp"
#include "vtr_radar/types.hpp"

#include <math.h>

namespace vtr {
namespace radar {

//* MotionDistortedRansac
/**
* \brief This class estimates the linear velocity and angular velocity of the sensor in the body-frame.
*
* Assuming constant velocity, the motion vector can be used to estimate the transform between any two pairs of points
* if the delta_t between those points is known.
*
* A single transform between the two pointclouds can also be retrieved.
*
* All operations are done in SE(3) even if the input is 2D. The output motion and transforms are in 3D.
*/
template <class PointT>
class MCRansac {
 public:
	MCRansac() = default;
  MCRansac(double tolerance, double inlier_ratio, int iterations,
  	int max_gn_iterations, double epsilon_converge, int subset_size) :
  		: tolerance_(tolerance), inlier_ratio_(inlier_ratio), iterations_(iterations),
  		max_gn_iterations_(max_gn_iterations), epsilon_converge_(epsilon_converge),
  		subset_size_(subset_size) { }

    /*!
       \brief Computes the body-centric 6x1 velocity vector w_2_1
				This can be converted to a 4x4 homogeneous transform with:
				T_1_2 = vec2tran(delta_t * w_2_1) where delta_t = t_2 - t_1
    */
    double run(const pcl::PointCloud<PointT> &pc1,
		  const pcl::PointCloud<PointT> &pc2, Eigen::VectorXd &w_2_1,
		  std::vector<int> &best_inliers);

private:
    std::vector<double> delta_ts;
    double tolerance_ = 0.1225;
    double inlier_ratio_ = 0.9;
    int iterations_ = 100;
    int max_gn_iterations_ = 10;
    double epsilon_converge_ = 0.0001;
    int subset_size_ = 2;
    double error_converge_ = 0.01;
    double r_observable_sq = 0.0625;
    int num_transforms_ = 21;

    /*!
       \brief Given two matched pointclouds (pc1, pc2), this function computes the motion of the sensor
       (linear and angular velocity) in the body frame using nonlinear least squares.
       \pre It's very important that the delt_t_local is accurate. Note that each azimuth in the radar scan is time
       stamped, this should be used to get the more accurate time differences.
    */
    void get_motion_parameters(const pcl::PointCloud<PointT> &pc1,
  		const pcl::PointCloud<PointT> &pc2, const std::vector<int> &subset,
  		Eigen::VectorXd &w_2_1);

    /*!
       \brief Retrieve the number of inliers corresponding to body motion vector wbar. (6 x 1)
    */
    int get_num_inliers(const pcl::PointCloud<PointT> &pc1,
  		const pcl::PointCloud<PointT> &pc2, const Eigen::VectorXd &w_2_1);

    void get_inliers(const pcl::PointCloud<PointT> &pc1,
  		const pcl::PointCloud<PointT> &pc2, const Eigen::VectorXd &w_2_1,
  		std::vector<int> &inliers);

};

}  // namespace radar
}  // namespace vtr