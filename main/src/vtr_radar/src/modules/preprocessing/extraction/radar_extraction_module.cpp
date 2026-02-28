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
 * \file radar_extraction_module.cpp
 * \author Sam Qiao, Autonomous Space Robotics Lab (ASRL)
 * \brief base_radar_extraction_module class methods definition
**/
 
#include "vtr_radar/modules/preprocessing/extraction/radar_extraction_module.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/plot.hpp>
#include <opencv2/core/cvstd_wrapper.hpp>

#include "vtr_radar/detector/detector.hpp"
#include "vtr_radar/utils/utils.hpp"
#include <pcl/common/common.h>

#include <filesystem>

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

template <class PointT>
cv::Mat extract_indices_from_pointcloud(const cv::Mat &raw_scan, const pcl::PointCloud<PointT> &pointcloud, const std::vector<double> &azimuth_angles, float res, float range_offset) {
    const int rows = raw_scan.rows;
    const int cols = raw_scan.cols;
    cv::Mat point_overlay = cv::Mat::zeros(rows, cols, CV_32F);

    for (auto &point : pointcloud) {
      for(int i=0; i<rows; i++){
        if(static_cast<float>(point.phi) == static_cast<float>(azimuth_angles[i])){
          int j = static_cast<int>((point.rho - range_offset) / res);
          if(j >= 0 && j < cols){
            point_overlay.at<float>(i, j) = 255.0;
          }
          break;
        }
      }
    }
    
  return point_overlay;
}

void save_radar_pointcloud_overlay(const cv::Mat &raw_scan, const std::vector<double> &azimuth_angles, const double maxr, const double cart_resolution, const double radar_resolution, const cv::Mat &point_overlay, const std::string config_name){

  int cart_pixel_width = (2 * maxr) / cart_resolution;
  
  cv::Mat L_cart;
  cv::Mat L_cart_colour;

  radar_polar_to_cartesian(point_overlay, azimuth_angles, L_cart, radar_resolution, cart_resolution, cart_pixel_width, true, CV_32F);
  cv::cvtColor(L_cart, L_cart_colour, cv::COLOR_GRAY2BGR);

  cv::Mat raw_scan_convert, raw_colour_convert;
  radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert, radar_resolution, cart_resolution, cart_pixel_width, true, CV_32F);
  cv::cvtColor(raw_scan_convert, raw_colour_convert, cv::COLOR_GRAY2BGR);

  raw_colour_convert = raw_colour_convert + L_cart_colour;

  std::string base_directory = "results/radar/detectors/";
  std::string directory = base_directory + config_name + "/pointcloud_overlays";
  std::filesystem::create_directories(directory);

  std::string path = directory + "/" + config_name + ".jpg";
  cv::imwrite(path, raw_colour_convert);

}

}  // namespace

using namespace tactic;
auto RadarExtractionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->detector = node->declare_parameter<std::string>(param_prefix + ".detector", config->detector);
  config->minr = node->declare_parameter<double>(param_prefix + ".minr", config->minr);
  config->maxr = node->declare_parameter<double>(param_prefix + ".maxr", config->maxr);
  config->range_offset = node->declare_parameter<double>(param_prefix + ".range_offset", config->range_offset);
  config->save_pointcloud_overlay = node->declare_parameter<bool>(param_prefix + ".save_pointcloud_overlay", config->save_pointcloud_overlay);

  // best decttector kpeaks
  // kpeaks
  config->kpeaks.kstrong = node->declare_parameter<int>(param_prefix + ".kpeaks.kstrong", config->kpeaks.kstrong);
  config->kpeaks.threshold2 = node->declare_parameter<double>(param_prefix + ".kpeaks.threshold2", config->kpeaks.threshold2);
  config->kpeaks.threshold3 = node->declare_parameter<double>(param_prefix + ".kpeaks.threshold3", config->kpeaks.threshold3);

  config->kstrongest.kstrong = node->declare_parameter<int>(param_prefix + ".kstrongest.kstrong", config->kstrongest.kstrong);
  config->kstrongest.static_threshold = node->declare_parameter<double>(param_prefix + ".kstrongest.static_threshold", config->kstrongest.static_threshold);

  config->cen2018.zq = node->declare_parameter<double>(param_prefix + ".cen2018.zq", config->cen2018.zq);
  config->cen2018.sigma = node->declare_parameter<int>(param_prefix + ".cen2018.sigma", config->cen2018.sigma);

  config->oscfar.width = node->declare_parameter<int>(param_prefix + ".oscfar.width", config->oscfar.width);
  config->oscfar.guard = node->declare_parameter<int>(param_prefix + ".oscfar.guard", config->oscfar.guard);
  config->oscfar.kstat = node->declare_parameter<int>(param_prefix + ".oscfar.kstat", config->oscfar.kstat);
  config->oscfar.threshold = node->declare_parameter<double>(param_prefix + ".oscfar.threshold", config->oscfar.threshold);

  config->tm_cfar.width = node->declare_parameter<int>(param_prefix + ".tm_cfar.width", config->tm_cfar.width);
  config->tm_cfar.guard = node->declare_parameter<int>(param_prefix + ".tm_cfar.guard", config->tm_cfar.guard);
  config->tm_cfar.threshold = node->declare_parameter<double>(param_prefix + ".tm_cfar.threshold", config->tm_cfar.threshold);
  config->tm_cfar.N1 = node->declare_parameter<int>(param_prefix + ".tm_cfar.N1", config->tm_cfar.N1);
  config->tm_cfar.N2 = node->declare_parameter<int>(param_prefix + ".tm_cfar.N2", config->tm_cfar.N2);

  config->cacfar.width = node->declare_parameter<int>(param_prefix + ".cacfar.width", config->cacfar.width);
  config->cacfar.guard = node->declare_parameter<int>(param_prefix + ".cacfar.guard", config->cacfar.guard);
  config->cacfar.threshold = node->declare_parameter<double>(param_prefix + ".cacfar.threshold", config->cacfar.threshold);

  config->modified_cacfar.width = node->declare_parameter<int>(param_prefix + ".modified_cacfar.width", config->modified_cacfar.width);
  config->modified_cacfar.guard = node->declare_parameter<int>(param_prefix + ".modified_cacfar.guard", config->modified_cacfar.guard);
  config->modified_cacfar.threshold = node->declare_parameter<double>(param_prefix + ".modified_cacfar.threshold", config->modified_cacfar.threshold);
  config->modified_cacfar.threshold2 = node->declare_parameter<double>(param_prefix + ".modified_cacfar.threshold2", config->modified_cacfar.threshold2);
  config->modified_cacfar.threshold3 = node->declare_parameter<double>(param_prefix + ".modified_cacfar.threshold3", config->modified_cacfar.threshold3);

  config->cago_cfar.width = node->declare_parameter<int>(param_prefix + ".cago_cfar.width", config->cago_cfar.width);
  config->cago_cfar.guard = node->declare_parameter<int>(param_prefix + ".cago_cfar.guard", config->cago_cfar.guard);
  config->cago_cfar.threshold = node->declare_parameter<double>(param_prefix + ".cago_cfar.threshold", config->cago_cfar.threshold);

  // added new cell averging smallest of detector
  config->caso_cfar.width = node->declare_parameter<int>(param_prefix + ".caso_cfar.width", config->caso_cfar.width);
  config->caso_cfar.guard = node->declare_parameter<int>(param_prefix + ".caso_cfar.guard", config->caso_cfar.guard);
  config->caso_cfar.threshold = node->declare_parameter<double>(param_prefix + ".caso_cfar.threshold", config->caso_cfar.threshold);

  config->is_cfar.width = node->declare_parameter<int>(param_prefix + ".is_cfar.width", config->is_cfar.width);
  config->is_cfar.guard = node->declare_parameter<int>(param_prefix + ".is_cfar.guard", config->is_cfar.guard);
  config->is_cfar.alpha_I = node->declare_parameter<double>(param_prefix + ".is_cfar.alpha_I", config->is_cfar.alpha_I);
  config->is_cfar.N_TI = node->declare_parameter<int>(param_prefix + ".is_cfar.N_TI", config->is_cfar.N_TI);
  config->is_cfar.beta_I = node->declare_parameter<double>(param_prefix + ".is_cfar.beta_I", config->is_cfar.beta_I);

  config->vi_cfar.width = node->declare_parameter<int>(param_prefix + ".vi_cfar.width", config->vi_cfar.width);
  config->vi_cfar.guard = node->declare_parameter<int>(param_prefix + ".vi_cfar.guard", config->vi_cfar.guard);
  config->vi_cfar.K_VI = node->declare_parameter<double>(param_prefix + ".vi_cfar.K_VI", config->vi_cfar.K_VI);
  config->vi_cfar.K_MR = node->declare_parameter<double>(param_prefix + ".vi_cfar.K_MR", config->vi_cfar.K_MR);
  config->vi_cfar.C_N = node->declare_parameter<double>(param_prefix + ".vi_cfar.C_N", config->vi_cfar.C_N);

  config->cfear_kstrong.width = node->declare_parameter<int>(param_prefix + ".cfear_kstrong.width", config->cfear_kstrong.width);
  config->cfear_kstrong.guard = node->declare_parameter<int>(param_prefix + ".cfear_kstrong.guard", config->cfear_kstrong.guard);
  config->cfear_kstrong.kstrong = node->declare_parameter<int>(param_prefix + ".cfear_kstrong.kstrong", config->cfear_kstrong.kstrong);
  config->cfear_kstrong.z_min = node->declare_parameter<double>(param_prefix + ".cfear_kstrong.z_min", config->cfear_kstrong.z_min);
  config->cfear_kstrong.r = node->declare_parameter<double>(param_prefix + ".cfear_kstrong.r", config->cfear_kstrong.r);
  config->cfear_kstrong.f = node->declare_parameter<double>(param_prefix + ".cfear_kstrong.f", config->cfear_kstrong.f);

  config->bfar.width = node->declare_parameter<int>(param_prefix + ".bfar.width", config->bfar.width);
  config->bfar.guard = node->declare_parameter<int>(param_prefix + ".bfar.guard", config->bfar.guard);
  config->bfar.threshold = node->declare_parameter<double>(param_prefix + ".bfar.threshold", config->bfar.threshold);
  config->bfar.static_threshold = node->declare_parameter<double>(param_prefix + ".bfar.static_threshold", config->bfar.static_threshold);

  config->msca_cfar.width = node->declare_parameter<int>(param_prefix + ".msca_cfar.width", config->msca_cfar.width);
  config->msca_cfar.guard = node->declare_parameter<int>(param_prefix + ".msca_cfar.guard", config->msca_cfar.guard);
  config->msca_cfar.threshold = node->declare_parameter<double>(param_prefix + ".msca_cfar.threshold", config->msca_cfar.threshold);
  config->msca_cfar.M = node->declare_parameter<int>(param_prefix + ".msca_cfar.M", config->msca_cfar.M);

  config->cen2019.width = node->declare_parameter<int>(param_prefix + ".cen2019.width", config->cen2019.width);
  config->cen2019.guard = node->declare_parameter<int>(param_prefix + ".cen2019.guard", config->cen2019.guard);
  config->cen2019.l_max = node->declare_parameter<int>(param_prefix + ".cen2019.l_max", config->cen2019.l_max);

  config->radar_resolution = node->declare_parameter<double>(param_prefix + ".radar_resolution", config->radar_resolution);
  config->cart_resolution = node->declare_parameter<double>(param_prefix + ".cart_resolution", config->cart_resolution);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  
  // Doppler stuff
  config->beta = node->declare_parameter<double>(param_prefix + ".beta", config->beta);
  config->upfront_range_corr = node->declare_parameter<bool>(param_prefix + ".upfront_range_corr", config->upfront_range_corr);

  // clang-format on
  return config;
}

void RadarExtractionModule::run_(QueryCache &qdata0, OutputCache &,
                                   const Graph::Ptr &,
                                   const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  if(!qdata.radar_data) return;

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

  /// Establish output beta and also the raw point cloud
  auto &beta = *qdata.beta.emplace();

  // create a reference to the raw point cloud
  auto &raw_point_cloud = *(qdata.raw_point_cloud.emplace());

  // get the data from the radar cache
  cv::Mat fft_scan = qdata.radar_data->fft_scan;
  cv::Mat cartesian = qdata.radar_data->cartesian;
  std::vector<int64_t> azimuth_times = qdata.radar_data->azimuth_times;
  std::vector<double> azimuth_angles = qdata.radar_data->azimuth_angles;
  std::vector<bool> up_chirps = qdata.radar_data->up_chirps;
  double radar_resolution = config_->radar_resolution;
  double cart_resolution = config_->cart_resolution;

  /// boreas navtech radar upgrade time - approximately 2021-10 onwards
  static constexpr int64_t upgrade_time = 1632182400000000000;
  if (*qdata.stamp > upgrade_time){
    if(radar_resolution == 0.0596){
      CLOG(WARNING, "radar.pc_extractor") << "Double check radar resolution: " << radar_resolution << ". Use 0.04381 for radar data after upgrade time";
    }
  } else{
    if(radar_resolution == 0.04381){
      CLOG(WARNING, "radar.pc_extractor") << "Double check radar resolution: " << radar_resolution << ". Use 0.0596 for radar data before upgrade time";
    }  
  }

  beta = config_->beta;

  CLOG(DEBUG, "radar.pc_extractor") << "Starting radar point cloud extraction using detector: " << config_->detector;

  // Now based on the choice of the detector, we will run the detector accordingly
  if (config_->detector == "cen2018") {
    Cen2018 detector = Cen2018<PointWithInfo>(
        config_->cen2018.zq, config_->cen2018.sigma, config_->minr,
        config_->maxr, config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else if (config_->detector == "kpeaks") {
    KPeaks detector = KPeaks<PointWithInfo>(
        config_->kpeaks.kstrong, config_->kpeaks.threshold2,
        config_->kpeaks.threshold3, config_->minr, config_->maxr,
        config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles,up_chirps,
                 raw_point_cloud);
  }else if (config_->detector == "kstrongest") {
    KStrongest detector = KStrongest<PointWithInfo>(
        config_->kstrongest.kstrong, config_->kstrongest.static_threshold,
        config_->minr, config_->maxr, config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else if (config_->detector == "oscfar") {
    OSCFAR detector = OSCFAR<PointWithInfo>(
        config_->oscfar.width, config_->oscfar.guard, config_->oscfar.kstat,
        config_->oscfar.threshold, config_->minr, config_->maxr,
        config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else if (config_->detector == "tm_cfar") {
    TM_CFAR detector = TM_CFAR<PointWithInfo>(
        config_->tm_cfar.width, config_->tm_cfar.guard,
        config_->tm_cfar.threshold, config_->tm_cfar.N1,
        config_->tm_cfar.N2, config_->minr, config_->maxr, config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else if (config_->detector == "cacfar") {
    CACFAR detector = CACFAR<PointWithInfo>(
        config_->cacfar.width, config_->cacfar.guard,
        config_->cacfar.threshold, config_->minr, config_->maxr, 
        config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else if (config_->detector == "modified_cacfar") {
    ModifiedCACFAR detector = ModifiedCACFAR<PointWithInfo>(
        config_->modified_cacfar.width, config_->modified_cacfar.guard,
        config_->modified_cacfar.threshold, config_->modified_cacfar.threshold2, 
        config_->modified_cacfar.threshold3, config_->minr, config_->maxr, 
        config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else if (config_->detector == "cfear_kstrong") {
    CFEAR_KStrong detector = CFEAR_KStrong<PointWithInfo>(
        config_->cfear_kstrong.width, config_->cfear_kstrong.guard,
        config_->cfear_kstrong.kstrong, config_->cfear_kstrong.z_min,
        config_->cfear_kstrong.r, config_->cfear_kstrong.f, config_->minr, config_->maxr,
        config_->range_offset); 
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else if (config_->detector == "bfar") {
    BFAR detector = BFAR<PointWithInfo>(
        config_->bfar.width, config_->bfar.guard,
        config_->bfar.threshold,
        config_->bfar.static_threshold, config_->minr, config_->maxr, 
        config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else if (config_->detector == "cago_cfar") {
    CAGO_CFAR detector = CAGO_CFAR<PointWithInfo>(
        config_->cago_cfar.width, config_->cago_cfar.guard,
        config_->cago_cfar.threshold, config_->minr, config_->maxr, 
        config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else if (config_->detector == "caso_cfar") {
    CASO_CFAR detector = CASO_CFAR<PointWithInfo>(
        config_->caso_cfar.width, config_->caso_cfar.guard,
        config_->caso_cfar.threshold, config_->minr, config_->maxr, 
        config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else if (config_->detector == "is_cfar") {
    IS_CFAR detector = IS_CFAR<PointWithInfo>(
        config_->is_cfar.width, config_->is_cfar.guard,
        config_->is_cfar.alpha_I, config_->is_cfar.N_TI,
        config_->is_cfar.beta_I, config_->minr, config_->maxr, 
        config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else if (config_->detector == "vi_cfar") {
    VI_CFAR detector = VI_CFAR<PointWithInfo>(
        config_->vi_cfar.width, config_->vi_cfar.guard,
        config_->vi_cfar.K_VI, config_->vi_cfar.K_MR,
        config_->vi_cfar.C_N, config_->minr, config_->maxr, 
        config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else if (config_->detector == "msca_cfar") {
    MSCA_CFAR detector = MSCA_CFAR<PointWithInfo>(
        config_->msca_cfar.width, config_->msca_cfar.guard,
        config_->msca_cfar.threshold, config_->msca_cfar.M,
        config_->minr, config_->maxr, config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else if (config_->detector == "cen2019") {
    Cen2019 detector = Cen2019<PointWithInfo>(
        config_->cen2019.width, config_->cen2019.guard,
        config_->cen2019.l_max, config_->minr, config_->maxr, 
        config_->range_offset);
    detector.run(fft_scan, radar_resolution, azimuth_times, azimuth_angles, up_chirps,
                 raw_point_cloud);
  } else {
    CLOG(ERROR, "radar.pc_extractor")
        << "Unknown detector: " << config_->detector;
    throw std::runtime_error("Unknown detector: " + config_->detector);
  }
// #endif

  // do upfront range correction if desired and radial velocity metadata present
  if (config_->upfront_range_corr) {
    bool all_pts_have_rv = true;
    for (auto &point : raw_point_cloud) {
      if (point.radial_velocity != -1000.0) {
        if (point.up_chirp)
          point.rho += point.radial_velocity * config_->beta;
        else
          point.rho -= point.radial_velocity * config_->beta;
      } else {
        all_pts_have_rv = false;
      }
    }

    if (!all_pts_have_rv) {
      CLOG(ERROR, "radar.navtech_extractor")
          << "Not all points have radial velocity for upfront range "
             "correction!!!!";
    }

    // If we did upfront correction, save the beta value
    beta = 0.0;
  }

  // sort points into a canonical order, this helps to reduce randomness and improves
  // reproducability while multithreading
  std::sort(raw_point_cloud.begin(), raw_point_cloud.end(), [](PointWithInfo a, PointWithInfo b) {
    if (a.timestamp == b.timestamp)
      return a.rho < b.rho;
    else
      return a.timestamp < b.timestamp;
  });

  if(config_->save_pointcloud_overlay){
    cv::Mat point_overlay = extract_indices_from_pointcloud(fft_scan, raw_point_cloud, azimuth_angles, radar_resolution, config_->range_offset);
    save_radar_pointcloud_overlay(fft_scan, azimuth_angles, config_->maxr, cart_resolution, radar_resolution, point_overlay, config_->detector);
  }

  // Convert to cartesian format
  pol2Cart2D(raw_point_cloud);
  CLOG(DEBUG, "radar.pc_extractor")<< "Radar Extracted " << raw_point_cloud.size() << " points";

  /// Visualize to rviz
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