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
 * \file polar_processing.hpp
 * \brief Polar processing utility functions
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <set>

#define _USE_MATH_DEFINES
#include <math.h>

#include <Eigen/Dense>

#include <vtr_lidar/cloud/cloud.hpp>
#include <vtr_lidar/nanoflann/nanoflann.hpp>
#include <vtr_lidar/pointmap/pointmap.hpp>

namespace vtr {
namespace lidar {

// KDTree type definition
using PointXYZ_KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3>;

void cart2Pol_(std::vector<PointXYZ>& xyz);
PointXYZ cart2pol(const PointXYZ& p);

void pca_features(std::vector<PointXYZ>& points,
                  std::vector<float>& eigenvalues,
                  std::vector<PointXYZ>& eigenvectors);

void detect_outliers(std::vector<PointXYZ>& rtp, std::vector<float>& scores,
                     int lidar_n_lines, float lidar_angle_res, float minTheta,
                     int n_pass, float threshold);

float get_lidar_angle_res(std::vector<PointXYZ>& rtp, float& minTheta,
                          float& maxTheta, int lidar_n_lines);

void scaleAndLogRadius(std::vector<PointXYZ>& rtp, float r_scale);

void scaleHorizontal(std::vector<PointXYZ>& rtp, float h_scale);

void extract_features_multi_thread(std::vector<PointXYZ>& points,
                                   std::vector<PointXYZ>& normals,
                                   std::vector<float>& planarity,
                                   std::vector<float>& linearity,
                                   int lidar_n_lines, float h_scale,
                                   float r_scale, int verbose);
/**
 * \brief todo
 * \param[in] points point cloud in cartesian
 * \param[in] polar_pts point cloud in polar
 * \param[in] normals point cloud normal
 * \param[in] r0 ideal distance for estimating the normal
 * \param[in] theta0 maximum incidence angle for estimating the normal
 * \param[out] scores a score for each point based on normal direction
 */
void smartNormalScore(const std::vector<PointXYZ>& points,
                      const std::vector<PointXYZ>& polar_pts,
                      const std::vector<PointXYZ>& normals, const float& r0,
                      const float& theta0, std::vector<float>& scores);

void smartICPScore(std::vector<PointXYZ>& polar_pts,
                   std::vector<float>& scores);

void compare_map_to_frame(std::vector<PointXYZ>& frame_points,
                          std::vector<PointXYZ>& map_points,
                          std::vector<PointXYZ>& map_normals,
                          std::unordered_map<VoxKey, size_t>& map_samples,
                          Eigen::Matrix3d R_d, Eigen::Vector3d T_d,
                          float theta_dl, float phi_dl, float map_dl,
                          std::vector<float>& movable_probs,
                          std::vector<int>& movable_counts);

void extractNormal(const std::vector<PointXYZ>& points,
                   const std::vector<PointXYZ>& polar_pts,
                   const std::vector<PointXYZ>& queries,
                   const std::vector<PointXYZ>& polar_queries,
                   const float polar_r, const int parallel_threads,
                   std::vector<PointXYZ>& normals,
                   std::vector<float>& norm_scores);

}  // namespace lidar
}  // namespace vtr