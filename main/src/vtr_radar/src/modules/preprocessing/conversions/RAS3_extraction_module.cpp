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
 * \author Sam Qiao, Autonomous Space Robotics Lab (ASRL)
 * \brief NavtechExtractionModule class methods definition
 */
#include "vtr_radar/modules/preprocessing/conversions/RAS3_extraction_module.hpp"

#include "cv_bridge/cv_bridge.h"

#include "vtr_radar/detector/detector.hpp"
#include "vtr_radar/utils/utils.hpp"

#include <pcl/common/common.h>

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

// static constexpr int64_t upgrade_time = 1632182400000000000;

auto RAS3ExtractionModule::Config::fromROS(
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

  // added new cell averging smallest of detector
  config->caso_cfar.width = node->declare_parameter<int>(param_prefix + ".caso_cfar.width", config->caso_cfar.width);
  config->caso_cfar.guard = node->declare_parameter<int>(param_prefix + ".caso_cfar.guard", config->caso_cfar.guard);
  config->caso_cfar.threshold = node->declare_parameter<double>(param_prefix + ".caso_cfar.threshold", config->caso_cfar.threshold);
  


  config->radar_resolution = node->declare_parameter<double>(param_prefix + ".radar_resolution", config->radar_resolution);
  config->cart_resolution = node->declare_parameter<double>(param_prefix + ".cart_resolution", config->cart_resolution);

  // // Doppler stuff
  // config->beta = node->declare_parameter<double>(param_prefix + ".beta", config->beta);
  // config->chirp_type = node->declare_parameter<std::string>(param_prefix + ".chirp_type", config->chirp_type);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void RAS3ExtractionModule::run_(QueryCache &qdata0, OutputCache &,
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

  /// Input radar message from the query cache
  // This is to convert a ROS image to an opencv image
  // CLOG(DEBUG, "radar.navtech_extractor") << "Sam: " << qdata.scan_msg->b_scan_img.encoding;
  auto fft_scan = cv_bridge::toCvCopy(std::make_shared<ImageMsg>(qdata.scan_msg->b_scan_img))->image;

  fft_scan.convertTo(fft_scan, CV_32F); 

  // normalize to 0-1
  fft_scan = fft_scan/255.0;

  /// Output
  auto &beta = *qdata.beta.emplace();
  auto &raw_point_cloud = *qdata.raw_point_cloud.emplace();

  cv::Mat cartesian;
  Cache<Timestamp> qstamp = qdata.stamp;
  // CLOG(DEBUG, "radar.navtech_extractor") << "Sam: The timestamp is " << *qstamp << " nano-secs";

  int64_t time_per_resulution = 1.0/4.0*1e9;
  int64_t current_time_stamp = *qstamp;


  std::vector<int64_t> azimuth_times;
  for (const auto& time : qdata.scan_msg->timestamps) {
    azimuth_times.emplace_back(static_cast<int64_t>(time));
    // CLOG(DEBUG, "radar.navtech_extractor") << "timestamp is " << azimuth_times.back() << "nano-secs";
  }

  // CLOG(DEBUG, "radar.navtech_extractor") <<"Sam: the timestamp size is: "<< qstamp.size();
  // CLOG(DEBUG, "radar.navtech_extractor") << "Sam: the timestamp is: " << qstamp;

  std::vector<double> azimuth_angles;
  for (const auto& encoder_value : qdata.scan_msg->encoder_values) {
    azimuth_angles.emplace_back(static_cast<double>(encoder_value)/16000*2*M_PI);
    // CLOG(DEBUG, "radar.navtech_extractor") << "azimuth_angle is " << azimuth_angles.back() << " radians";
  }

  /// \note for now we retrieve radar resolution from config file
  float radar_resolution = config_->radar_resolution;
  float cart_resolution = config_->cart_resolution;
  beta = config_->beta;

  // Convert to cartesian BEV image
  int cart_pixel_width = (2 * config_->maxr) / cart_resolution;

  radar_polar_to_cartesian(fft_scan, azimuth_angles, cartesian,
                           radar_resolution, cart_resolution, cart_pixel_width,
                           true, CV_32F);

  // debug message
  // CLOG(DEBUG, "radar.navtech_extractor")
  //     << "fft_scan has " << fft_scan.rows << " rows and " << fft_scan.cols
  //     << " cols with resolution " << radar_resolution;

  // CLOG(DEBUG, "radar.navtech_extractor") << "cartesian has " << cartesian.rows
  //                                        << " rows and " << cartesian.cols
  //                                        << " cols with resolution "
  //                                        << cart_resolution;
  
  // CLOG(DEBUG, "radar.navtech_extractor") << "azimuth_angles has " << azimuth_angles.size() << " elements";
  // CLOG(DEBUG, "radar.navtech_extractor") << "azimuth_times has " << azimuth_times.size() << " elements";

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
  }else if(config_->detector == "caso_cfar"){
    CASO_CFAR detector = CASO_CFAR<PointWithInfo>(
        config_->caso_cfar.width, config_->caso_cfar.guard,
        config_->caso_cfar.threshold, config_->minr, config_->maxr,
        config_->range_offset);
        
    // CLOG(DEBUG, "radar.navtech_extractor") << "Sam: CASO" <<  config_->caso_cfar.width;
    // CLOG(DEBUG, "radar.navtech_extractor") << "Sam: CASO" <<  config_->caso_cfar.guard;
    // CLOG(DEBUG, "radar.navtech_extractor") << "Sam: CASO" <<  config_->caso_cfar.threshold;
    // CLOG(DEBUG, "radar.navtech_extractor") << "Sam: CASO" <<  config_->minr;
    // CLOG(DEBUG, "radar.navtech_extractor") << "Sam: CASO" <<  config_->maxr;
    // CLOG(DEBUG, "radar.navtech_extractor") << "Sam: CASO" <<  config_->range_offset;

    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles,
                 raw_point_cloud);  
    }else {
    CLOG(ERROR, "radar.navtech_extractor")
        << "Unknown detector: " << config_->detector;
    throw std::runtime_error("Unknown detector: " + config_->detector);
  }
#endif

  // Convert to cartesian format
  pol2Cart2D(raw_point_cloud);

  CLOG(DEBUG, "radar.navtech_extractor")<< "Radar Extracted " << raw_point_cloud.size() << " points";


  // DEBUG
  // # Sam I like to know the exact details of the point cloud
  // for(int j=0; j<raw_point_cloud.size(); j++){
  //   CLOG(DEBUG, "radar.navtech_extractor") << "Sam: The point is: " << raw_point_cloud.points[j].x << " " << raw_point_cloud.points[j].y ;
  // }

  // if you would like to know what are the min anfd max points in the point cloud
  // PointWithInfo min_pt, max_pt;
  // pcl::getMinMax3D(raw_point_cloud, min_pt, max_pt);

  // CLOG(DEBUG, "radar.navtech_extractor") << "Sam: The min point is: " << min_pt.x << " " << min_pt.y << " " << min_pt.z;
  // CLOG(DEBUG, "radar.navtech_extractor") << "Sam: The max point is: " << max_pt.x << " " << max_pt.y << " " << max_pt.z;

  // DEBUG
  // for (auto &point : pointcloud) {
  //   CLOG(DEBUG, "radar.navtech_extractor") << "Sam: The point x is: " << point.x ;
  //   CLOG(DEBUG, "radar.navtech_extractor") << "Sam: The point y is: " << point.y ;

  // }

  /// Visualize
  if (config_->visualize) {
    // publish the raw scan image
    cv_bridge::CvImage scan_image;
    scan_image.header.frame_id = "radar";
    // scan_image.header.stamp = qdata.scan_msg->header.stamp;
    scan_image.encoding = "mono8";
    scan_image.image = fft_scan;
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