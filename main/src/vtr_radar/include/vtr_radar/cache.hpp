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
 * \file cache.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <opencv2/opencv.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_radar/data_structures/pointmap.hpp"
#include "vtr_radar/types.hpp"
#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace radar {

struct RadarQueryCache : public tactic::QueryCache {
  using Ptr = std::shared_ptr<RadarQueryCache>;

  // input
  tactic::Cache<sensor_msgs::msg::Image> scan_msg;  // from ROS
  tactic::Cache<cv::Mat> scan;                      // from cpp
  tactic::Cache<const tactic::EdgeTransform> T_s_r;

  // preprocessing
  tactic::Cache<float> radar_resolution;
  tactic::Cache<cv::Mat> fft_scan;
  tactic::Cache<std::vector<double>> azimuth_times;
  tactic::Cache<std::vector<double>> azimuth_angles;
  tactic::Cache<pcl::PointCloud<PointWithInfo>> raw_point_cloud;
};

struct RadarOutputCache : public tactic::OutputCache {
  using Ptr = std::shared_ptr<RadarOutputCache>;
};

}  // namespace radar
}  // namespace vtr