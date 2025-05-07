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
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "navtech_msgs/msg/radar_b_scan_msg.hpp"

#include "vtr_radar/data_types/point.hpp"
#include "vtr_radar/data_types/azimuth.hpp"
#include "vtr_radar/data_types/pointmap.hpp"
#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/types.hpp"

#include"vtr_radar/types.hpp"

namespace vtr {
namespace radar {

struct RadarQueryCache : virtual public tactic::QueryCache {
  PTR_TYPEDEFS(RadarQueryCache);
  // metadata
  tactic::Cache<std::string> seq_name;

  // radar input
  tactic::Cache<navtech_msgs::msg::RadarBScanMsg> scan_msg;  // from ROS
  tactic::Cache<cv::Mat> scan;                      // from cpp
  tactic::Cache<const tactic::EdgeTransform> T_s_r;

  // gyro input
  tactic::Cache<sensor_msgs::msg::Imu> gyro_msg;
  tactic::Cache<sensor_msgs::msg::Imu> prev_gyro_msg;
  tactic::Cache<const tactic::EdgeTransform> T_s_r_gyro;
  tactic::Cache<std::vector<sensor_msgs::msg::Imu>> gyro_msgs;

  // radar raw data
  tactic::Cache<RadarData> radar_data;

  // preprocessing
  tactic::Cache<float> beta;
  tactic::Cache<pcl::PointCloud<PointWithInfo>> raw_point_cloud;
  tactic::Cache<pcl::PointCloud<PointWithInfo>> preprocessed_point_cloud;

  // odometry & mapping
  tactic::Cache<const pcl::PointCloud<PointWithInfo>> undistorted_point_cloud;
#if false  /// store raw point cloud
  tactic::Cache<const pcl::PointCloud<PointWithInfo>> undistorted_raw_point_cloud;
#endif
  tactic::Cache<PointMap<PointWithInfo>> sliding_map_odo;
  tactic::Cache<tactic::EdgeTransform> T_r_m_odo;
  tactic::Cache<Eigen::Matrix<double, 6, 1>> w_m_r_in_r_odo;

  // save odometry variables from last radar update (to be used for next ICP)
  // this is necessary since the above variables might be overwritten by gyro in the meantime

  tactic::Cache<tactic::EdgeTransform> T_r_m_odo_radar;
  tactic::Cache<Eigen::Matrix<double, 6, 1>> w_m_r_in_r_odo_radar;
  tactic::Cache<tactic::Timestamp> timestamp_odo_radar;

  tactic::Cache<lgmath::se3::Transformation> T_r_m_odo_prior;
  tactic::Cache<Eigen::Matrix<double, 6, 1>> w_m_r_in_r_odo_prior;
  tactic::Cache<Eigen::Matrix<double, 12, 12>> cov_prior;
  tactic::Cache<int64_t> timestamp_prior;

  // localization
  tactic::Cache<const PointMap<PointWithInfo>> submap_loc;
  tactic::Cache<const bool> submap_loc_changed;
  tactic::Cache<const tactic::EdgeTransform> T_v_m_loc;

  // Doppler paper stuff
  tactic::Cache<Eigen::Vector2d> vel_meas;
  tactic::Cache<double> yaw_meas;
  tactic::Cache<DopplerScan> doppler_scan;
};

struct RadarOutputCache : virtual public tactic::OutputCache {
  PTR_TYPEDEFS(RadarOutputCache);
};

}  // namespace radar
}  // namespace vtr