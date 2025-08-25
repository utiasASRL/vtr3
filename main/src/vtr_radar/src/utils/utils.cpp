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
#include "vtr_radar/types.hpp"

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

// // Sam I can add a few dense helpers here (maybe later)
// using torch::indexing::Slice;
// torch::Tensor dopplerUpDown(const RadarFrame& rf) {
//     // Compute base path by removing last directory
//     auto pos = rf.sensor_root.find_last_of('/');
//     std::string base = (pos != std::string::npos)
//         ? rf.sensor_root.substr(0, pos)
//         : rf.sensor_root;
//     std::string doppler_path = base + "/radar/" + rf.frame;

//     cv::Mat img = cv::imread(doppler_path, cv::IMREAD_GRAYSCALE);
//     if (img.empty()) {
//         throw std::runtime_error("Failed to load image: " + doppler_path);
//     }

//     // Extract column 10 as 1D tensor
//     cv::Mat col = img.col(10).clone();
//     auto options = torch::TensorOptions().dtype(torch::kUInt8);
//     torch::Tensor t = torch::from_blob(
//         col.data, {col.rows}, options
//     ).to(torch::kInt32).clone();
//     return t;
// }

// bool checkChirp(const RadarFrame& rf) {
//     torch::Tensor up_chirps = dopplerUpDown(rf);
//     // Check even indices == 255 and odd indices == 0
//     auto evens = up_chirps.index({torch::arange(0, up_chirps.size(0), 2)});
//     auto odds  = up_chirps.index({torch::arange(1, up_chirps.size(0), 2)});
//     bool even_ok = evens.eq(255).all().item<bool>();
//     bool odd_ok  = odds.eq(0).all().item<bool>();
//     // If pattern invalid, return true to indicate chirp_up (needs flip)
//     return !(even_ok && odd_ok);
// }

torch::Tensor getGaussianKernel2D(int ksize_x, int ksize_y, double sigma_x, double sigma_y, torch::Device device) 
{
    int half_x = ksize_x / 2;
    int half_y = ksize_y / 2;

    auto x = torch::arange(-half_x, half_x + 1, torch::TensorOptions().dtype(torch::kFloat32).device(device));
    auto y = torch::arange(-half_y, half_y + 1, torch::TensorOptions().dtype(torch::kFloat32).device(device));

    auto xx = x.pow(2).div(2 * sigma_x * sigma_x).unsqueeze(1);  // shape (kx, 1)
    auto yy = y.pow(2).div(2 * sigma_y * sigma_y).unsqueeze(0);  // shape (1, ky)

    auto kernel = torch::exp(-(xx + yy));  // shape (kx, ky)
    kernel /= kernel.sum();  // normalize

    return kernel;
}

torch::Tensor applyGaussianBlur2D(const torch::Tensor& input, int kx, int ky, double sx, double sy) 
{
    torch::NoGradGuard no_grad;
    if (kx % 2 == 0 || ky % 2 == 0) {
        throw std::invalid_argument("Kernel size must be odd");
    }
    auto kernel = getGaussianKernel2D(kx, ky, sx, sy, input.device());
    kernel = kernel.view({1, 1, ky, kx});  // Conv2d expects [out_channels, in_channels, H, W]

    auto conv = torch::nn::Conv2d(torch::nn::Conv2dOptions(1, 1, {ky, kx}).padding({ky / 2, kx / 2}).bias(false));
    conv->weight.set_data(kernel.clone());  // clone to detach from computation graph
    conv->to(input.device());
    return conv->forward(input);
}

RadarDataTorch toTorch(const RadarData& src, const torch::Device& device) {
  RadarDataTorch dst;

  // 1. Convert timestamps: vector<int64_t> (ns) -> microseconds -> tensor (float64)
  std::vector<double> timestamps_us;
  timestamps_us.reserve(src.azimuth_times.size());
  for (auto ns : src.azimuth_times) {
    timestamps_us.push_back(static_cast<double>(ns) / 1e3);  // ns -> us
  }
  dst.timestamps = torch::from_blob(
      timestamps_us.data(),
      {(long)timestamps_us.size()},
      torch::TensorOptions().dtype(torch::kFloat64).device(torch::kCPU) // must start CPU
  ).clone().to(device);  // clone to own memory, then move to device

  // 2. Azimuths: vector<double> -> tensor (float32)
  dst.azimuths = torch::from_blob(
      (void*)src.azimuth_angles.data(),
      {(long)src.azimuth_angles.size()},
      torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU)
  ).clone().to(device);

  // 3. FFT scan cv::Mat -> tensor (float32, H x W')
  cv::Mat fft_float;
  src.fft_scan.convertTo(fft_float, CV_32F);
  dst.polar = torch::from_blob(
      fft_float.data,
      {fft_float.rows, fft_float.cols},
      torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU)
  ).clone().to(device);

  // // 4. Single timestamp: ns -> s
  // dst.timestamp = static_cast<double>(src.timestamp) / 1e9;

  return dst;
}



}  // namespace radar
}  // namespace vtr