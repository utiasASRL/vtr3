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
 */
#pragma once

#include <opencv2/opencv.hpp>

#include "vtr_radar/data_types/point.hpp"

namespace vtr {
namespace radar {

void load_radar(const std::string &path, std::vector<double> &timestamps,
                std::vector<double> &azimuths, cv::Mat &fft_data);

void load_radar(const cv::Mat &raw_data, std::vector<double> &timestamps,
                std::vector<double> &azimuths, cv::Mat &fft_data);

/** \brief Returns the cartesian image of a radar scan */
// clang-format off
void radar_polar_to_cartesian(const cv::Mat &fft_data,
                              const std::vector<double> &azimuths,
                              cv::Mat &cartesian,
                              const float radar_resolution,
                              const float cart_resolution,
                              const int cart_pixel_width,
                              const bool interpolate_crossover,
                              const int output_type = CV_8UC1);
// clang-format on

// vbar is 3x1 v_s_i_in_s where (s) is sensor, (i) is a static frame
// like an inertial frame or (pm) frame in this repository
// beta \approx f_transmission / (slope of modulation pattern dfdt)
template <class PointT>
void removeDoppler(pcl::PointCloud<PointT> &point_cloud,
                   const Eigen::VectorXd &vbar, const double beta) {
  for (auto &p : point_cloud) {
    Eigen::Vector3d abar = {p.x, p.y, p.z};
    abar.normalize();
    // relative velocity (> 0: point/sensor moving towards each other)
    const double v = abar.transpose() * vbar;
    const double delta_rho = beta * v;
    // correct radial distance using Doppler factor
    const double rho = sqrt(p.x * p.x + p.y * p.y + p.z * p.z) + delta_rho;
    const double theta = atan2(sqrt(p.x * p.x + p.y * p.y), p.z);
    const double phi = atan2(p.y, p.x);
    // Correct point locations
    p.x = rho * std::cos(phi) * std::sin(theta);
    p.y = rho * std::sin(phi) * std::sin(theta);
    p.z = rho * std::cos(theta);
  }
}

}  // namespace radar
}  // namespace vtr