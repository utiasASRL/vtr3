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
 * \file range_image.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/filters/range_image.hpp"

namespace vtr {
namespace lidar {

void generate_range_image(const pcl::PointCloud<PointWithInfo>& point_cloud, RowMatrixXf& range_image, RangeImageParams params) {
  Eigen::MatrixXi temp_idx = Eigen::MatrixXi::Constant(range_image.rows(), range_image.cols(), -1);
  generate_range_image(point_cloud, range_image, temp_idx, params);
}

/// \todo Parallelize  
void generate_range_image(const pcl::PointCloud<PointWithInfo>& point_cloud, RowMatrixXf& range_image, Eigen::MatrixXi& idx_image, RangeImageParams params) {

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    auto& point = point_cloud[i];
    if (point.rho > params.crop_range)
      continue;
    double proj_x = (remainder(point.phi, 2 * M_PI) / 2 / M_PI + 0.5);
    double proj_y = 1.0 - (M_PI/2 - point.theta  + abs(params.fov_down)) / params.getFOV();
    

    proj_x = (proj_x < 0.0) ? 0.0 : proj_x;
    proj_x = (proj_x > 1.0) ? 1.0 : proj_x;

    proj_y = (proj_y < 0.0) ? 0.0 : proj_y;
    proj_y = (proj_y > 1.0) ? 1.0 : proj_y;

    proj_x *= range_image.cols() - 1;
    proj_y *= range_image.rows() - 1;

    range_image((int)proj_y, (int)proj_x) = point.rho;
    idx_image((int)proj_y, (int)proj_x) = i;
  }
}

//Definitely parallelize, no reverse duplication
void unproject_range_image(pcl::PointCloud<PointWithInfo>& point_cloud, const RowMatrixXf& data_image, const Eigen::MatrixXi& idx_image) {

  for (int i = 0; i < idx_image.rows(); i++){
    //#pragma omp parallel for schedule(dynamic, 10) num_threads(num_threads)
    for (int j = 0; j < idx_image.cols(); j++) {
      int idx = idx_image(i, j);
      if (idx >= 0)
        point_cloud[idx].flex24 = data_image(i, j) + 1;
    }
  }

}

sensor_msgs::msg::Image range_to_image(const RowMatrixXf& data_image) {
  float max_range = data_image.maxCoeff();

  sensor_msgs::msg::Image ros_im;
  ros_im.width = data_image.cols();
  ros_im.height = data_image.rows();
  ros_im.encoding = "mono8";
  ros_im.step = data_image.cols();
  size_t size = data_image.size()*sizeof(uint8_t);
  ros_im.data.resize(size);

  RowMatrixXuI8 scaled_image = (data_image / max_range * 255).cast<uint8_t>();
  memcpy(&ros_im.data[0], scaled_image.data(), size);

  return ros_im;
}

} //namespace lidar
} //namespace vtr