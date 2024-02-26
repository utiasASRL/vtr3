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

  params.width = intensity_image.cols;
  params.height = intensity_image.rows;

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    auto& point = point_cloud[i];
    if (point.z > params.crop_range || point.z < params.min_range)
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

  params.width = depth_image.cols;
  params.height = depth_image.rows;

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    auto& point = point_cloud[i];
    if (point.z > params.crop_range || point.z < params.min_range)
      continue;
    
    int u = (int)round(params.f_u() * point.x / point.z) + params.c_u();
    int v = (int)round(params.f_v() * point.y / point.z) + params.c_v();

    if (0 <= u && u < params.width && 0 <= v && v < params.height) {
      depth_image.at<u_int8_t>(u, v, 2) = point.intensity;
      depth_image.at<u_int8_t>(u, v, 0) = point.z;

      idx_image.at<u_int32_t>(u, v) = i;
    }
  }

}

} //lidar
} //vtr