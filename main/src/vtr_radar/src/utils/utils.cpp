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

void load_radar(const std::string &path, std::vector<double> &timestamps,
                std::vector<double> &azimuths, cv::Mat &fft_data) {
  const cv::Mat raw_data = cv::imread(path, cv::IMREAD_GRAYSCALE);
  load_radar(raw_data, timestamps, azimuths, fft_data);
}

void load_radar(const cv::Mat &raw_data, std::vector<double> &timestamps,
                std::vector<double> &azimuths, cv::Mat &fft_data) {
  const double time_convert = 1.0e-6;
  const double encoder_conversion = 2 * M_PI / 5600;
  const uint N = raw_data.rows;
  timestamps = std::vector<double>(N, 0);
  azimuths = std::vector<double>(N, 0);
  const uint range_bins = raw_data.cols - 11;
  fft_data = cv::Mat::zeros(N, range_bins, CV_32F);
#pragma omp parallel
  for (uint i = 0; i < N; ++i) {
    const uchar *byteArray = raw_data.ptr<uchar>(i);
    timestamps[i] = *((int64_t *)(byteArray)) * time_convert;
    azimuths[i] = *((uint16_t *)(byteArray + 8)) * encoder_conversion;
    // The 10th byte is reserved but unused
    for (uint j = 0; j < range_bins; ++j) {
      fft_data.at<float>(i, j) = (float)*(byteArray + 11 + j) / 255.0;
    }
  }
}

}  // namespace radar
}  // namespace vtr