// Copyright 2023, Autonomous Space Robotics Lab (ASRL)
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
 * \file perspective_image.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/filters/perspective_image.hpp"


namespace vtr {
namespace lidar {

/// \todo Parallelize  
void generate_intensity_image(const pcl::PointCloud<PointWithInfo>& point_cloud, cv::Mat& intensity_image, cv::Mat& idx_image, PerspectiveImageParams params) {

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    auto& point = point_cloud[i];
    if (point.z > params.max_range || point.z < params.min_range)
      continue;
    
    int u = (int)round(params.f_u() * point.x / point.z) + params.c_u();
    int v = (int)round(params.f_v() * point.y / point.z) + params.c_v();

    if (0 <= u && u < params.width && 0 <= v && v < params.height) {
      intensity_image.at<u_int8_t>(u, v) = point.intensity;
      idx_image.at<u_int32_t>(u, v) = i;
    }
  }

}

void generate_depth_image(const pcl::PointCloud<PointWithInfo>& point_cloud, cv::Mat& depth_image, cv::Mat& idx_image, PerspectiveImageParams params) {

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    auto& point = point_cloud[i];
    if (point.z > params.max_range || point.z < params.min_range)
      continue;
    
    int u = (int)round(params.f_u() * point.x / point.z) + params.c_u();
    int v = (int)round(params.f_v() * point.y / point.z) + params.c_v();

    if (0 <= u && u < params.width && 0 <= v && v < params.height) {
      cv::Vec3b &hsv = depth_image.at<cv::Vec3b>(v, u);

      if (hsv[0] == 0 ||  hsv[0] > abs(point.z) * UINT8_MAX / params.max_range) {
        hsv[2] = sqrt(point.flex23) * 15.5 < UINT8_MAX ? sqrt(point.intensity) * 15.5 : UINT8_MAX;
        hsv[1] = UINT8_MAX - 1;
        hsv[0] = abs(point.z) * UINT8_MAX / params.max_range;

        idx_image.at<u_int32_t>(v, u) = i;
      }

    }
  }

}

} //lidar
} //vtr