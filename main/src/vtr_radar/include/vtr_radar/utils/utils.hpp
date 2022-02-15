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
 * \file utils.hpp
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 * \brief Utility functions for working with radar data
 */
#pragma once

#include <opencv2/opencv.hpp>

#include "vtr_radar/types.hpp"

namespace vtr {
namespace radar {

void load_radar(const std::string &path, std::vector<double> &timestamps,
                std::vector<double> &azimuths, cv::Mat &fft_data);

void load_radar(const cv::Mat &raw_data, std::vector<double> &timestamps,
                std::vector<double> &azimuths, cv::Mat &fft_data);

template <class PointT>
void pol2Cart2D(pcl::PointCloud<PointT> &pointcloud) {
  for (auto &point : pointcloud) {
    point.x = point.rho * std::cos(point.phi);
    point.y = point.rho * std::sin(point.phi);
    point.z = 0.0;
  }
}

/**
 * \brief Returns the cartesian image of a radar scan
 * \note This function changes fft_data when interpolate_crossover is true.
 */
cv::Mat radar_polar_to_cartesian(cv::Mat &fft_data,
                                 const std::vector<double> &azimuths,
                                 const float radar_resolution,
                                 const float cart_resolution,
                                 const int cart_pixel_width,
                                 const bool interpolate_crossover,
                                 const int output_type = CV_32F);

}  // namespace radar
}  // namespace vtr