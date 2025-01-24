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
 * \file offline_radar_conversion_module.cpp
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 * \brief OfflineRadarConversionModule class methods definition
 */
#include "vtr_radar/modules/preprocessing/conversions/offline_radar_conversion_module.hpp"
#include "cv_bridge/cv_bridge.h"
#include "vtr_radar/utils/utils.hpp"

namespace vtr {
namespace radar {

namespace {

template <class PointT>
void pol2Cart2D(pcl::PointCloud<PointT> &pointcloud) {
  for (auto &point : pointcloud) {
    point.x = point.rho * std::cos(point.phi);
    point.y = point.rho * std::sin(point.phi);
    point.z = 0.0;
  }
}

}  // namespace

using namespace tactic;

/// boreas navtech radar upgrade time
static constexpr int64_t upgrade_time = 1632182400000000000;

auto OfflineRadarConversionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->maxr = node->declare_parameter<double>(param_prefix + ".maxr", config->maxr);
  config->radar_resolution = node->declare_parameter<double>(param_prefix + ".radar_resolution", config->radar_resolution);
  config->cart_resolution = node->declare_parameter<double>(param_prefix + ".cart_resolution", config->cart_resolution);
  config->chirp_type = node->declare_parameter<std::string>(param_prefix + ".chirp_type", config->chirp_type);

  // clang-format on
  return config;
}

void OfflineRadarConversionModule::run_(QueryCache &qdata0, OutputCache &,
                                   const Graph::Ptr &,
                                   const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  /// Input
#if false
  auto scan = cv_bridge::toCvCopy(qdata.scan_msg.ptr(), "mono8")->image;
  scan.convertTo(scan, CV_32F);
#else
  const auto &scan = *qdata.scan;
#endif

  /// temp variables
  cv::Mat scan_use;
  cv::Mat fft_scan;
  cv::Mat cartesian;
  std::vector<int64_t> azimuth_times;
  std::vector<double> azimuth_angles;

  /// \note for now we retrieve radar resolution from load_radar function
#if false
  // Set radar resolution
  float radar_resolution = config_->radar_resolution;
#else
  // use the first timestamp to determine the resolution
  float radar_resolution = *qdata.stamp > upgrade_time ? 0.04381 : 0.0596;
#endif
  float cart_resolution = config_->cart_resolution;

  // Downsample scan based on desired chirp type
  if (config_->chirp_type == "up") {
    // Choose only every second row, starting from row 0
    scan_use = cv::Mat::zeros(scan.rows / 2, scan.cols, cv::IMREAD_GRAYSCALE);
    int j = 0;
    for (int i = 0; i < scan.rows; i+=2) {
      scan.row(i).copyTo(scan_use.row(j));
      j++;
    }
  } else if (config_->chirp_type == "down") {
    // Choose only every second row, starting from row 1
    scan_use = cv::Mat::zeros(scan.rows / 2, scan.cols, cv::IMREAD_GRAYSCALE);
    int j = 0;
    for (int i = 1; i < scan.rows; i+=2) {
      scan.row(i).copyTo(scan_use.row(i));
      j++;
    }
  } else{
    scan_use = scan;
  }

  // Load scan, times, azimuths from scan
  load_radar(scan_use, azimuth_times, azimuth_angles, fft_scan);

  // Convert to cartesian BEV image
  int cart_pixel_width = (2 * config_->maxr) / cart_resolution;
  radar_polar_to_cartesian(fft_scan, azimuth_angles, cartesian,
                           radar_resolution, cart_resolution, cart_pixel_width,
                           true, CV_32F);
  CLOG(DEBUG, "radar.pc_extractor")
      << "fft_scan has " << fft_scan.rows << " rows and " << fft_scan.cols
      << " cols with resolution " << radar_resolution;

  CLOG(DEBUG, "radar.pc_extractor") << "cartesian has " << cartesian.rows
                                         << " rows and " << cartesian.cols
                                         << " cols with resolution "
                                         << cart_resolution;
                                    
  CLOG(DEBUG, "radar.pc_extractor") << "azimuth_angles has " << azimuth_angles.size() << " elements";
  CLOG(DEBUG, "radar.pc_extractor") << "azimuth_times has " << azimuth_times.size() << " elements";

  qdata.radar_data.emplace();
  /// store them to the cache
  qdata.radar_data->fft_scan = fft_scan;
  qdata.radar_data->cartesian = cartesian;
  qdata.radar_data->azimuth_times = azimuth_times;
  qdata.radar_data->azimuth_angles = azimuth_angles;  

                                   }

}  // namespace radar
}  // namespace vtr