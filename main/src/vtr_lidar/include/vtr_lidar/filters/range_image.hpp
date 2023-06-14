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
#include "vtr_lidar/data_types/point.hpp"

namespace vtr {
namespace lidar{

struct RangeImageParams {
  double fov_up;    //In radians
  double fov_down;  //In radians

  double getFOV() { return abs(fov_down) + abs(fov_up);}
};

/// \todo Parallelize  
void generate_range_image(const pcl::PointCloud<PointWithInfo>& point_cloud, Eigen::MatrixXf& range_image, RangeImageParams params) {

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    auto& point = point_cloud[i];
    double proj_x = 0.5 * (point.theta / M_PI + 1.0);
    double proj_y = 1.0 - (point.phi + abs(params.fov_down)) / params.getFOV();

    proj_x = (proj_x < 0.0) ? 0.0 : proj_x;
    proj_x = (proj_x > 1.0) ? 1.0 : proj_x;

    proj_y = (proj_y < 0.0) ? 0.0 : proj_y;
    proj_y = (proj_y > 1.0) ? 1.0 : proj_y;

    proj_x *= range_image.cols() - 1;
    proj_y *= range_image.rows() - 1;

    range_image((int)proj_y, (int)proj_x) = point.rho;
  }
}

} // namespace lidar
} // vtr
