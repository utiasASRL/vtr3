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

auto NavtechExtractionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->detector = node->declare_parameter<std::string>(param_prefix + ".detector", config->detector);
  config->minr = node->declare_parameter<double>(param_prefix + ".minr", config->minr);
  config->maxr = node->declare_parameter<double>(param_prefix + ".maxr", config->maxr);
  config->range_offset = node->declare_parameter<double>(param_prefix + ".range_offset", config->range_offset);

  config->kstrong.kstrong = node->declare_parameter<int>(param_prefix + ".kstrong.kstrong", config->kstrong.kstrong);
  config->kstrong.threshold2 = node->declare_parameter<double>(param_prefix + ".kstrong.threshold2", config->kstrong.threshold2);
  config->kstrong.threshold3 = node->declare_parameter<double>(param_prefix + ".kstrong.threshold3", config->kstrong.threshold3);

  config->cen2018.zq = node->declare_parameter<double>(param_prefix + ".cen2018.zq", config->cen2018.zq);
  config->cen2018.sigma = node->declare_parameter<int>(param_prefix + ".cen2018.sigma", config->cen2018.sigma);

  config->cacfar.width = node->declare_parameter<int>(param_prefix + ".cacfar.width", config->cacfar.width);
  config->cacfar.guard = node->declare_parameter<int>(param_prefix + ".cacfar.guard", config->cacfar.guard);
  config->cacfar.threshold = node->declare_parameter<double>(param_prefix + ".cacfar.threshold", config->cacfar.threshold);
  config->cacfar.threshold2 = node->declare_parameter<double>(param_prefix + ".cacfar.threshold2", config->cacfar.threshold2);
  config->cacfar.threshold3 = node->declare_parameter<double>(param_prefix + ".cacfar.threshold3", config->cacfar.threshold3);

  config->oscfar.width = node->declare_parameter<int>(param_prefix + ".oscfar.width", config->oscfar.width);
  config->oscfar.guard = node->declare_parameter<int>(param_prefix + ".oscfar.guard", config->oscfar.guard);
  config->oscfar.kstat = node->declare_parameter<int>(param_prefix + ".oscfar.kstat", config->oscfar.kstat);
  config->oscfar.threshold = node->declare_parameter<double>(param_prefix + ".oscfar.threshold", config->oscfar.threshold);
  config->oscfar.threshold2 = node->declare_parameter<double>(param_prefix + ".oscfar.threshold2", config->oscfar.threshold2);
  config->oscfar.threshold3 = node->declare_parameter<double>(param_prefix + ".oscfar.threshold3", config->oscfar.threshold3);

  config->modified_cacfar.width = node->declare_parameter<int>(param_prefix + ".modified_cacfar.width", config->modified_cacfar.width);
  config->modified_cacfar.guard = node->declare_parameter<int>(param_prefix + ".modified_cacfar.guard", config->modified_cacfar.guard);
  config->modified_cacfar.threshold = node->declare_parameter<double>(param_prefix + ".modified_cacfar.threshold", config->modified_cacfar.threshold);
  config->modified_cacfar.threshold2 = node->declare_parameter<double>(param_prefix + ".modified_cacfar.threshold2", config->modified_cacfar.threshold2);
  config->modified_cacfar.threshold3 = node->declare_parameter<double>(param_prefix + ".modified_cacfar.threshold3", config->modified_cacfar.threshold3);

  config->radar_resolution = node->declare_parameter<double>(param_prefix + ".radar_resolution", config->radar_resolution);
  config->cart_resolution = node->declare_parameter<double>(param_prefix + ".cart_resolution", config->cart_resolution);
  config->beta = node->declare_parameter<double>(param_prefix + ".beta", config->beta);

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
    bev_scan_pub_ = qdata.node->create_publisher<ImageMsg>("bev_scan", 5);
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
  auto &beta = *qdata.beta.emplace();
  auto &raw_point_cloud = *qdata.raw_point_cloud.emplace();

  /// temp variables
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
  beta = config_->beta;

  // Load scan, times, azimuths from scan
  load_radar(scan, azimuth_times, azimuth_angles, fft_scan);

  // Convert to cartesian BEV image
  int cart_pixel_width = (2 * config_->maxr) / cart_resolution;
  radar_polar_to_cartesian(fft_scan, azimuth_angles, cartesian,
                           radar_resolution, cart_resolution, cart_pixel_width,
                           true, CV_32F);
  CLOG(DEBUG, "radar.navtech_extractor")
      << "fft_scan has " << fft_scan.rows << " rows and " << fft_scan.cols
      << " cols with resolution " << radar_resolution;

  // Extract keypoints and times
#if false
  const auto detector = [&]() -> std::unique_ptr<Detector<PointWithInfo>> {
    if (config_->detector == "cen2018")
      return std::make_unique<Cen2018<PointWithInfo>>(
          config_->cen2018.zq, config_->cen2018.sigma, config_->minr,
          config_->maxr);
    else if (config_->detector == "kstrongest")
      return std::make_unique<KStrongest<PointWithInfo>>(
          config_->kstrong.kstrong, config_->kstrong.threshold2
          , config_->kstrong.threshold3, config_->minr, config_->maxr);
    else if (config_->detector == "cacfar")
      return std::make_unique<CACFAR<PointWithInfo>>(
          config_->cacfar.width, config_->cacfar.guard,
          config_->cacfar.threshold, config_->cacfar.threshold2, config_->cacfar.threshold3,
          config_->cacfar.minr, config_->cacfar.maxr);
    else if (config_->detector == "oscfar")
      return std::make_unique<OSCFAR<PointWithInfo>>(
          config_->oscfar.width, config_->oscfar.guard, config_->oscfar.kstat,
          config_->oscfar.threshold, config_->oscfar.threshold2, config_->oscfar.threshold3,
          config_->oscfar.minr, config_->oscfar.maxr);
    else if (config_->detector == "modified_cacfar")
      return std::make_unique<ModifiedCACFAR<PointWithInfo>>(
          config_->modified_cacfar.width, config_->modified_cacfar.guard,
          config_->modified_cacfar.threshold, config_->modified_cacfar.threshold2,
          config_->modified_cacfar.threshold3,
          config_->modified_cacfar.minr, config_->modified_cacfar.maxr);
    else {
      CLOG(ERROR, "radar.navtech_extractor")
          << "Unknown detector: " << config_->detector;
      throw std::runtime_error("Unknown detector: " + config_->detector);
    }
  }();
  detector->run(fft_scan, radar_resolution, azimuth_times, azimuth_angles,
                raw_point_cloud);
#else
  if (config_->detector == "cen2018") {
    Cen2018 detector = Cen2018<PointWithInfo>(
        config_->cen2018.zq, config_->cen2018.sigma, config_->minr,
        config_->maxr, config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles,
                 raw_point_cloud);
  } else if (config_->detector == "kstrongest") {
    KStrongest detector = KStrongest<PointWithInfo>(
        config_->kstrong.kstrong, config_->kstrong.threshold2,
        config_->kstrong.threshold3, config_->minr, config_->maxr,
        config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles,
                 raw_point_cloud);
  } else if (config_->detector == "cacfar") {
    CACFAR detector = CACFAR<PointWithInfo>(
        config_->cacfar.width, config_->cacfar.guard, config_->cacfar.threshold,
        config_->cacfar.threshold2, config_->cacfar.threshold3, config_->minr,
        config_->maxr, config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles,
                 raw_point_cloud);
  } else if (config_->detector == "oscfar") {
    OSCFAR detector = OSCFAR<PointWithInfo>(
        config_->oscfar.width, config_->oscfar.guard, config_->oscfar.kstat,
        config_->oscfar.threshold, config_->oscfar.threshold2,
        config_->oscfar.threshold3, config_->minr, config_->maxr,
        config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles,
                 raw_point_cloud);
  } else if (config_->detector == "modified_cacfar") {
    ModifiedCACFAR detector = ModifiedCACFAR<PointWithInfo>(
        config_->modified_cacfar.width, config_->modified_cacfar.guard,
        config_->modified_cacfar.threshold, config_->modified_cacfar.threshold2,
        config_->modified_cacfar.threshold3, config_->minr, config_->maxr,
        config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles,
                 raw_point_cloud);
  } else {
    CLOG(ERROR, "radar.navtech_extractor")
        << "Unknown detector: " << config_->detector;
    throw std::runtime_error("Unknown detector: " + config_->detector);
  }
#endif

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

    // publish the cartesian bev image
    cv_bridge::CvImage bev_scan_image;
    bev_scan_image.header.frame_id = "radar";
    // bev_scan_image.header.stamp = qdata.scan_msg->header.stamp;
    bev_scan_image.encoding = "mono8";
    cartesian.convertTo(bev_scan_image.image, CV_8UC1, 255);
    bev_scan_pub_->publish(*bev_scan_image.toImageMsg());

    // publish the converted point cloud
    auto point_cloud_tmp = raw_point_cloud;
    std::for_each(point_cloud_tmp.begin(), point_cloud_tmp.end(),
                  [&](PointWithInfo &point) {
                    point.flex21 =
                        static_cast<float>(point.timestamp - *qdata.stamp) /
                        1e9;
                  });
    PointCloudMsg pc2_msg;
    pcl::toROSMsg(point_cloud_tmp, pc2_msg);
    pc2_msg.header.frame_id = "radar";
    pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    pointcloud_pub_->publish(pc2_msg);
  }
}

}  // namespace radar
}  // namespace vtr