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
 * \file utils.cpp
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 * \brief Utility functions for working with radar data
 */
#include "vtr_radar/utils/utils.hpp"

namespace vtr {
namespace radar {

void load_radar(const std::string &path, std::vector<int64_t> &timestamps,
                std::vector<double> &azimuths, std::vector<bool> &up_chirps, cv::Mat &fft_data, Eigen::Vector2d &vel_meas, double &yaw_meas) {
  const cv::Mat raw_data = cv::imread(path, cv::IMREAD_GRAYSCALE);
  load_radar(raw_data, timestamps, azimuths, up_chirps, fft_data, vel_meas, yaw_meas);
}

void load_radar(const cv::Mat &raw_data, std::vector<int64_t> &timestamps,
                std::vector<double> &azimuths, std::vector<bool> &up_chirps, cv::Mat &fft_data, Eigen::Vector2d &vel_meas, double &yaw_meas) {

  const int64_t time_convert = 1000;
  const double encoder_conversion = 2 * M_PI / 5600;
  const uint N = raw_data.rows;
  const uint M = raw_data.cols;
  timestamps = std::vector<int64_t>(N, 0);
  azimuths = std::vector<double>(N, 0);
  up_chirps = std::vector<bool>(N, true);

  bool doppler_metadata = false;
  bool velocity_metadata = false;
  if (M == 6859) {
    doppler_metadata = false;
  } else if (M == 6863) {
    doppler_metadata = true;
  } else if (M == 6871) {
    velocity_metadata = true;
  } else {
    throw std::runtime_error("Invalid radar data size");
  }

  const uint range_bins = M - 11 - (doppler_metadata ? 4 : 0) - (velocity_metadata ? 12 : 0);
  fft_data = cv::Mat::zeros(N, range_bins, CV_32F);

  // #pragma omp parallel
  for (uint i = 0; i < N; ++i) {
    const uchar *byteArray = raw_data.ptr<uchar>(i);
    timestamps[i] = *((int64_t *)(byteArray)) * time_convert;
    azimuths[i] = *((uint16_t *)(byteArray + 8)) * encoder_conversion;\
    // The 11th byte is used for up_chirp
    up_chirps[i] = *(byteArray + 10);
    for (uint j = 0; j < range_bins; ++j) {
      fft_data.at<float>(i, j) = (float)*(byteArray + 11 + j) / 255.0;
    }
    
    // For paper 1, the last 12 bytes contain fwd, side, and yaw measurements
    if (velocity_metadata && i == 0) {
      vel_meas(0) = *((float *)(byteArray + M - 12));
      vel_meas(1) = *((float *)(byteArray + M - 8));
      yaw_meas = *((float *)(byteArray + M - 4));
    }
  }
}

// clang-format off
namespace {

unsigned get_closest(const double &x, const std::vector<double> &v) {
  const auto low = std::lower_bound(v.begin(), v.end(), x);
  unsigned idx = low - v.begin();
  if (idx == 0) return idx;
  if (idx >= v.size()) idx = v.size() - 1;
  const double d = std::fabs(x - v[idx]);
  if (std::fabs(x - v[idx - 1]) < d) return idx - 1;
  return idx;
}

double get_azimuth_index(const std::vector<double> &azimuths,
  const double azimuth) {
  double closest = 0;
  const int M = azimuths.size();
  closest = get_closest(azimuth, azimuths);
  if (azimuths[closest] < azimuth) {
    double delta = 0;
    if (closest < M - 1)
        delta = (azimuth - azimuths[closest]) / (azimuths[closest + 1] - azimuths[closest]);
    closest += delta;
  } else if (azimuths[closest] > azimuth){
    double delta = 0;
    if (closest > 0)
        delta = (azimuths[closest] - azimuth) / (azimuths[closest] - azimuths[closest - 1]);
    closest -= delta;
  }
  return closest;
}

}
// clang-format on

void radar_polar_to_cartesian(const cv::Mat &fft_data,
                              const std::vector<double> &azimuths,
                              cv::Mat &cartesian, const float radar_resolution,
                              const float cart_resolution,
                              const int cart_pixel_width,
                              const bool interpolate_crossover,
                              const int output_type) {
  cv::Mat fft = fft_data.clone();
  float cart_min_range = (cart_pixel_width / 2) * cart_resolution;
  if (cart_pixel_width % 2 == 0)
    cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution;

  cv::Mat map_x = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);
  cv::Mat map_y = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);

  // #pragma omp parallel for collapse(2)
  for (int j = 0; j < map_y.cols; ++j) {
    for (int i = 0; i < map_y.rows; ++i) {
      map_y.at<float>(i, j) = -1 * cart_min_range + j * cart_resolution;
    }
  }
  // #pragma omp parallel for collapse(2)
  for (int i = 0; i < map_x.rows; ++i) {
    for (int j = 0; j < map_x.cols; ++j) {
      map_x.at<float>(i, j) = cart_min_range - i * cart_resolution;
    }
  }

  cv::Mat range = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);
  cv::Mat angle = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);
  // double azimuth_step = azimuths[1] - azimuths[0];
  // #pragma omp parallel for collapse(2)
  for (int i = 0; i < range.rows; ++i) {
    for (int j = 0; j < range.cols; ++j) {
      float x = map_x.at<float>(i, j);
      float y = map_y.at<float>(i, j);
      float r = (sqrt(pow(x, 2) + pow(y, 2)) - radar_resolution / 2) /
                radar_resolution;
      if (r < 0) r = 0;
      range.at<float>(i, j) = r;
      float theta = atan2f(y, x);
      if (theta < 0) theta += 2 * M_PI;
      // if (navtech_version == CIR204) {
      angle.at<float>(i, j) = get_azimuth_index(azimuths, theta);
      // } else {
      // angle.at<float>(i, j) = (theta - azimuths[0]) / azimuth_step;
      // }
    }
  }
  if (interpolate_crossover) {
    cv::Mat a0 = cv::Mat::zeros(1, fft.cols, CV_32F);
    cv::Mat aN_1 = cv::Mat::zeros(1, fft.cols, CV_32F);
    for (int j = 0; j < fft.cols; ++j) {
      a0.at<float>(0, j) = fft.at<float>(0, j);
      aN_1.at<float>(0, j) = fft.at<float>(fft.rows - 1, j);
    }
    cv::vconcat(aN_1, fft, fft);
    cv::vconcat(fft, a0, fft);
    angle = angle + 1;
  }

  cv::remap(fft, cartesian, range, angle, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
            cv::Scalar(0, 0, 0));
  if (output_type != CV_32F) {
    cartesian.convertTo(cartesian, output_type, 255.0);
  }
}

}  // namespace radar
}  // namespace vtr