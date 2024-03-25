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
 * \file range_image.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "pcl/point_cloud.h"
#include "sensor_msgs/msg/image.hpp"
#include "vtr_lidar/data_types/point.hpp"

namespace vtr {
namespace lidar{

using RowMatrixXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RowMatrixXuI8 = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

struct RangeImageParams {
  double fov_up;    //In radians
  double fov_down;  //In radians
  double crop_range = 1000.0; //m

  double getFOV() { return abs(fov_up - fov_down);}
};

void generate_range_image(const pcl::PointCloud<PointWithInfo>& point_cloud, RowMatrixXf& range_image, RangeImageParams params);
void generate_range_image(const pcl::PointCloud<PointWithInfo>& point_cloud, RowMatrixXf& range_image, Eigen::MatrixXi& idx_image, RangeImageParams params);
void unproject_range_image(pcl::PointCloud<PointWithInfo>& point_cloud, const RowMatrixXf& data_image, const Eigen::MatrixXi& idx_image);
sensor_msgs::msg::Image range_to_image(const RowMatrixXf& data_image);



} // namespace lidar
} // vtr
