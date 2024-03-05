// Copyright 2024, Autonomous Space Robotics Lab (ASRL)
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
 * \file range_image.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/data_types/point.hpp"
#include "math.h"
#include "opencv2/core/mat.hpp"


namespace vtr {
namespace lidar{

struct PerspectiveImageParams {
  double v_fov;    //In radians
  double h_fov;  //In radians
  double max_range = 1000.0; //m
  double min_range = 0.1; //m
  int width;
  int height;

  double f_u(){
    return (double)(width) / 2.0 / tan(h_fov / 2);
  }
  
  unsigned c_u() { return width / 2; }

  double f_v(){
    return (double)(height) / 2.0 / tan(v_fov / 2);
  }
  
  unsigned c_v() { return height / 2; }
};

//Greyscale image with intensity
void generate_intensity_image(const pcl::PointCloud<PointWithInfo>& point_cloud, cv::Mat& intesity_image, cv::Mat& idx_image, PerspectiveImageParams params);

//HSV image. Hue is depth Saturation is constant. Value is intensity
void generate_depth_image(const pcl::PointCloud<PointWithInfo>& point_cloud, cv::Mat& depth_image, cv::Mat& idx_image, PerspectiveImageParams params);

void unproject_data_image(pcl::PointCloud<PointWithInfo>& point_cloud, const cv::Mat& depth_image, const cv::Mat& idx_image);

void interpolate_hsv_image(cv::Mat& depth_image);

void mask_to_pointcloud(const cv::Mat& mask, const cv::Mat& index_img, pcl::PointCloud<PointWithInfo>& point_cloud);

} // namespace lidar
} // vtr