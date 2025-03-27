// Copyright 2024, Autonomous Space Robotics Lab (ASRL)
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
 * \file online_radar_conversion_module.cpp
 * \author Sam Qiao, Autonomous Space Robotics Lab (ASRL)
 * \brief onlineRadarConversionModule class methods definition
 */
#include "vtr_radar/modules/preprocessing/conversions/online_radar_conversion_module.hpp"
#include "cv_bridge/cv_bridge.h"
#include "vtr_radar/utils/utils.hpp"
#include <pcl/common/common.h>
#include<vtr_radar/types.hpp>

namespace vtr {
namespace radar {

using namespace tactic;
// used for warthog RAS3 radar
auto OnlineRadarConversionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->cartesian_maxr = node->declare_parameter<double>(param_prefix + ".cartesian_maxr", config->cartesian_maxr);
  config->radar_resolution = node->declare_parameter<double>(param_prefix + ".radar_resolution", config->radar_resolution);
  config->cart_resolution = node->declare_parameter<double>(param_prefix + ".cart_resolution", config->cart_resolution);
  config->encoder_bin_size = node->declare_parameter<int>(param_prefix + ".encoder_bin_size", config->encoder_bin_size);

  // clang-format on
  return config;
}

void OnlineRadarConversionModule::run_(QueryCache &qdata0, OutputCache &,
                                   const Graph::Ptr &,
                                   const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  if(!qdata.scan_msg)
  {
    return;
  }
  /// Input radar message from the query cache
  // This is to convert a ROS image to an opencv image
  auto fft_scan = cv_bridge::toCvCopy(std::make_shared<ImageMsg>(qdata.scan_msg->b_scan_img))->image;

  fft_scan.convertTo(fft_scan, CV_32F); 

  // normalize to 0-1
  fft_scan = fft_scan/255.0;

  cv::Mat cartesian;
  Cache<Timestamp> qstamp = qdata.stamp;
  std::vector<bool> up_chirps;


  std::vector<int64_t> azimuth_times;
  for (const auto& time : qdata.scan_msg->timestamps) {
    azimuth_times.emplace_back(static_cast<int64_t>(time));
    up_chirps.emplace_back(false);
  }

  std::vector<double> azimuth_angles;
  CLOG(DEBUG, "radar.online_converter")<< "Total encoder size: " << config_->encoder_bin_size << " bins";

  for (const auto& encoder_value : qdata.scan_msg->encoder_values) {
    // log encoder bin size
    azimuth_angles.emplace_back(static_cast<double>(encoder_value)/config_->encoder_bin_size*2*M_PI); //16000 is for RAS3 radar
  }

  /// \note for now we retrieve radar resolution from config file
  float radar_resolution = config_->radar_resolution;
  float cart_resolution = config_->cart_resolution;

  // Convert to cartesian BEV image
  int cart_pixel_width = (2 * config_->cartesian_maxr) / cart_resolution;

  radar_polar_to_cartesian(fft_scan, azimuth_angles, cartesian,
                           radar_resolution, cart_resolution, cart_pixel_width,
                           true, CV_32F);

  
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