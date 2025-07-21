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
  config->cartesian_maxr = node->declare_parameter<double>(param_prefix + ".cartesian_maxr", config->cartesian_maxr);
  config->radar_resolution = node->declare_parameter<double>(param_prefix + ".radar_resolution", config->radar_resolution);
  config->cart_resolution = node->declare_parameter<double>(param_prefix + ".cart_resolution", config->cart_resolution);

  // clang-format on
  return config;
}

void OfflineRadarConversionModule::run_(QueryCache &qdata0, OutputCache &,
                                   const Graph::Ptr &,
                                   const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  if(!qdata.scan) return;

  /// Input
#if false
  auto scan = cv_bridge::toCvCopy(qdata.scan_msg.ptr(), "mono8")->image;
  scan.convertTo(scan, CV_32F);
#else
  const auto &scan = *qdata.scan;
#endif

  /// temp variables
  cv::Mat fft_scan;
  cv::Mat cartesian;
  std::vector<int64_t> azimuth_times;
  std::vector<double> azimuth_angles;
  std::vector<bool> up_chirps;
  Eigen::Vector2d vel_meas;
  double yaw_meas = -1000.0;

  /// \note for now we retrieve radar resolution from load_radar function
#if false
  // Set radar resolution
  float radar_resolution = config_->radar_resolution;
#else
  // use the first timestamp to determine the resolution
  float radar_resolution = *qdata.stamp > upgrade_time ? 0.04381 : 0.0596;
#endif
  float cart_resolution = config_->cart_resolution;

  // Load scan, times, azimuths from scan
  load_radar(scan, azimuth_times, azimuth_angles, up_chirps, fft_scan, vel_meas, yaw_meas);
  qdata.yaw_meas.emplace(yaw_meas);
  qdata.vel_meas.emplace(vel_meas);

  // Convert to cartesian BEV image
  int cart_pixel_width = (2 * config_->cartesian_maxr) / cart_resolution;
  radar_polar_to_cartesian(fft_scan, azimuth_angles, cartesian,
                           radar_resolution, cart_resolution, cart_pixel_width,
                           true, CV_32F);
  CLOG(DEBUG, "radar.conversion")
      << "fft_scan has " << fft_scan.rows << " rows and " << fft_scan.cols
      << " cols with resolution " << radar_resolution;

  CLOG(DEBUG, "radar.conversion") << "cartesian has " << cartesian.rows
                                         << " rows and " << cartesian.cols
                                         << " cols with resolution "
                                         << cart_resolution;
                                    
  CLOG(DEBUG, "radar.conversion") << "azimuth_angles has " << azimuth_angles.size() << " elements";
  CLOG(DEBUG, "radar.conversion") << "azimuth_times has " << azimuth_times.size() << " elements";

  qdata.radar_data.emplace();
  /// store them to the cache
  qdata.radar_data->fft_scan = fft_scan;
  qdata.radar_data->cartesian = cartesian;
  qdata.radar_data->azimuth_times = azimuth_times;
  qdata.radar_data->azimuth_angles = azimuth_angles;  
  qdata.radar_data->up_chirps = up_chirps;

                                   }

}  // namespace radar
}  // namespace vtr