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
 * \file navtech_extraction_module.cpp
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 * \brief NavtechExtractionModule class methods definition
 */
#include "vtr_radar/modules/preprocessing/conversions/navtech_extraction_module.hpp"

#include "cv_bridge/cv_bridge.h"

#include "vtr_radar/detector/detector.hpp"
#include "vtr_radar/utils/utils.hpp"

namespace vtr {
namespace radar {

using namespace tactic;

/// boreas navtech radar upgrade time
static constexpr int64_t upgrade_time = 1632182400000000000;

auto NavtechExtractionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->detector = node->declare_parameter<std::string>(param_prefix + ".detector", config->detector);
  config->kstrong = node->declare_parameter<int>(param_prefix + ".kstrong", config->kstrong);
  config->minr = node->declare_parameter<double>(param_prefix + ".minr", config->minr);
  config->maxr = node->declare_parameter<double>(param_prefix + ".maxr", config->maxr);
  config->zq = node->declare_parameter<double>(param_prefix + ".zq", config->zq);
  config->sigma = node->declare_parameter<int>(param_prefix + ".sigma", config->sigma);
  config->width = node->declare_parameter<int>(param_prefix + ".width", config->width);
  config->guard = node->declare_parameter<int>(param_prefix + ".guard", config->guard);
  config->kstat = node->declare_parameter<int>(param_prefix + ".kstat", config->kstat);
  config->threshold = node->declare_parameter<double>(param_prefix + ".threshold", config->threshold);
  config->threshold2 = node->declare_parameter<double>(param_prefix + ".threshold2", config->threshold2);
  config->radar_resolution = node->declare_parameter<double>(param_prefix + ".radar_resolution", config->radar_resolution);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void NavtechExtractionModule::run_(QueryCache &qdata0, OutputCache &,
                                   const Graph::Ptr &,
                                   const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    scan_pub_ = qdata.node->create_publisher<ImageMsg>("raw_scan", 5);
    fft_scan_pub_ = qdata.node->create_publisher<ImageMsg>("fft_scan", 5);
    pointcloud_pub_ = qdata.node->create_publisher<PointCloudMsg>("raw_point_cloud", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  /// Input
#if false
  auto scan = cv_bridge::toCvShare(qdata.scan_msg.ptr(), "mono8")->image;
  scan.convertTo(scan, CV_32F);
#else
  const auto &scan = *qdata.scan;
#endif

  /// Output
  auto &fft_scan = *qdata.fft_scan.emplace();
  auto &azimuth_times = *qdata.azimuth_times.emplace();
  auto &azimuth_angles = *qdata.azimuth_angles.emplace();
  auto &radar_resolution = *qdata.radar_resolution.emplace();
  auto &raw_point_cloud = *qdata.raw_point_cloud.emplace();

  /// \note for now we retrieve radar resolution from load_radar function
#if false
  // Set radar resolution
  radar_resolution = config_->radar_resolution;
#else
  // use the first timestamp to determine the resolution
  radar_resolution = *qdata.stamp > upgrade_time ? 0.04381 : 0.0596;
#endif

  // Load scan, times, azimuths from scan
  load_radar(scan, azimuth_times, azimuth_angles, fft_scan);
  CLOG(DEBUG, "radar.navtech_extractor")
      << "fft_scan has " << fft_scan.rows << " rows and " << fft_scan.cols
      << " cols with resolution " << radar_resolution;

  // Extract keypoints and times
  // const auto detector = [&]() -> std::unique_ptr<Detector<PointWithInfo>> {
  //   if (config_->detector == "cen2018")
  //     return std::make_unique<Cen2018<PointWithInfo>>(
  //         config_->zq, config_->sigma, config_->minr, config_->maxr);
  //   else if (config_->detector == "kstrongest")
  //     return std::make_unique<KStrongest<PointWithInfo>>(
  //         config_->kstrong, config_->threshold, config_->minr, config_->maxr);
  //   else if (config_->detector == "cacfar")
  //     return std::make_unique<KStrongest<PointWithInfo>>(
  //         config_->width, config_->guard, config_->threshold, config_->threshold2,
  //         config_->minr, config_->maxr);
  //   else if (config_->detector == "oscfar")
  //     return std::make_unique<KStrongest<PointWithInfo>>(
  //         config_->width, config_->guard, config_->kstat,
  //         config_->threshold, config_->threshold2, config_->minr, config_->maxr);
  //   else {
  //     CLOG(ERROR, "radar.navtech_extractor")
  //         << "Unknown detector: " << config_->detector;
  //     throw std::runtime_error("Unknown detector: " + config_->detector);
  //   }
  // }();
  // detector->run(fft_scan, radar_resolution, azimuth_times, azimuth_angles,
  //               raw_point_cloud);

  if (config_->detector == "cen2018") {
    Cen2018 detector = Cen2018<PointWithInfo>(config_->zq, config_->sigma, config_->minr, config_->maxr);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles,
            raw_point_cloud);
  } else if (config_->detector == "kstrongest") {
    KStrongest detector = KStrongest<PointWithInfo>(config_->kstrong, config_->threshold, config_->minr, config_->maxr);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles,
                raw_point_cloud);
  } else if (config_->detector == "cacfar") {
    CACFAR detector = CACFAR<PointWithInfo>(config_->width, config_->guard, config_->threshold, config_->threshold2,
          config_->minr, config_->maxr);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles,
                raw_point_cloud);
  } else if (config_->detector == "oscfar") {
    OSCFAR detector = OSCFAR<PointWithInfo>(config_->width, config_->guard, config_->kstat,
      config_->threshold, config_->threshold2, config_->minr, config_->maxr);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles,
                raw_point_cloud);
  }
  else {
    CLOG(ERROR, "radar.navtech_extractor")
        << "Unknown detector: " << config_->detector;
    throw std::runtime_error("Unknown detector: " + config_->detector);
  }


  // Convert to cartesian format
  pol2Cart2D(raw_point_cloud);

  CLOG(DEBUG, "radar.navtech_extractor")
      << "Extracted " << raw_point_cloud.size() << " points";

  /// Visualize
  if (config_->visualize) {
    // publish the raw scan image
    cv_bridge::CvImage scan_image;
    scan_image.header.frame_id = "radar";
    // scan_image.header.stamp = qdata.scan_msg->header.stamp;
    scan_image.encoding = "mono8";
    scan_image.image = scan;
    scan_pub_->publish(*scan_image.toImageMsg());

    // publish the fft scan image
    cv_bridge::CvImage fft_scan_image;
    fft_scan_image.header.frame_id = "radar";
    // fft_scan_image.header.stamp = qdata.scan_msg->header.stamp;
    fft_scan_image.encoding = "mono8";
    fft_scan.convertTo(fft_scan_image.image, CV_8UC1, 255);
    fft_scan_pub_->publish(*fft_scan_image.toImageMsg());

    // publish the converted point cloud
    auto point_cloud_tmp = raw_point_cloud;
    const auto ref_time = point_cloud_tmp.at(0).time;
    std::for_each(point_cloud_tmp.begin(), point_cloud_tmp.end(),
                  [&](PointWithInfo &point) { point.time -= ref_time; });
    auto pc2_msg = std::make_shared<PointCloudMsg>();
    pcl::toROSMsg(point_cloud_tmp, *pc2_msg);
    pc2_msg->header.frame_id = "radar";
    pc2_msg->header.stamp = rclcpp::Time(*qdata.stamp);
    pointcloud_pub_->publish(*pc2_msg);
  }
}

}  // namespace radar
}  // namespace vtr