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
 * \file doppler_extraction_module.cpp
 * \author Daniil Lisus, Autonomous Space Robotics Lab (ASRL)
 * \brief doppler_extraction_module class methods definition
**/
 
#include "vtr_radar/modules/preprocessing/extraction/doppler_extraction_module.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/plot.hpp>
#include <opencv2/core/cvstd_wrapper.hpp>

#include "vtr_radar/utils/utils.hpp"
#include <srd/extractor/doppler_extractor.hpp>
#include <pcl/common/common.h>
#include <cmath> 
#include <filesystem>
namespace fs = std::filesystem;


namespace vtr {
namespace radar {
namespace {

srd::extractor::DopplerExtractor::Options load_extractor_options(const DopplerExtractionModule::Config &config) {
  srd::extractor::DopplerExtractor::Options options;
  options.radar_res = config.radar_resolution;
  options.f_t = config.f_t;
  options.meas_freq = config.meas_freq;
  options.del_f = config.del_f;
  options.min_range = config.minr;
  options.max_range = config.maxr;
  options.beta_corr_fact = config.beta_corr_fact;
  options.pad_num = config.pad_num;
  options.max_velocity = config.max_velocity;
  options.sigma_gauss = config.sigma_gauss;
  options.z_q = config.z_q;
  options.ransac_max_iter = config.ransac_max_iter;
  options.ransac_threshold = config.ransac_threshold;
  options.ransac_prior_threshold = config.ransac_prior_threshold;
  options.opt_max_iter = config.opt_max_iter;
  options.opt_threshold = config.opt_threshold;
  options.cauchy_rho = config.cauchy_rho;
  options.x_bias_slope = config.x_bias_slope;
  options.x_bias_intercept = config.x_bias_intercept;
  options.y_bias_slope = config.y_bias_slope;
  options.y_bias_intercept = config.y_bias_intercept;

  return options;
}

}  // namespace

using namespace tactic;
auto DopplerExtractionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // General radar params
  config->radar_resolution = node->declare_parameter<double>(param_prefix + ".radar" + ".radar_resolution", config->radar_resolution);
  config->f_t = node->declare_parameter<double>(param_prefix + ".radar" + ".f_t", config->f_t);
  config->meas_freq = node->declare_parameter<double>(param_prefix + ".radar" + ".meas_freq", config->meas_freq);
  config->del_f = node->declare_parameter<double>(param_prefix + ".radar" + ".del_f", config->del_f);
  // Signal extraction params
  config->minr = node->declare_parameter<double>(param_prefix + ".signal" + ".minr", config->minr);
  config->maxr = node->declare_parameter<double>(param_prefix + ".signal" + ".maxr", config->maxr);
  config->beta_corr_fact = node->declare_parameter<double>(param_prefix + ".signal" + ".beta_corr_fact", config->beta_corr_fact);
  config->pad_num = node->declare_parameter<int>(param_prefix + ".signal" + ".pad_num", config->pad_num);
  config->max_velocity = node->declare_parameter<double>(param_prefix + ".signal" + ".max_velocity", config->max_velocity);
  // Filter params
  config->sigma_gauss = node->declare_parameter<double>(param_prefix + ".filter" + ".sigma_gauss", config->sigma_gauss);
  config->z_q = node->declare_parameter<double>(param_prefix + ".filter" + ".z_q", config->z_q);
  // RANSAC params
  config->ransac_max_iter = node->declare_parameter<int>(param_prefix + ".ransac" + ".max_iter", config->ransac_max_iter);
  config->ransac_threshold = node->declare_parameter<double>(param_prefix + ".ransac" + ".threshold", config->ransac_threshold);
  config->ransac_prior_threshold = node->declare_parameter<double>(param_prefix + ".ransac" + ".prior_threshold", config->ransac_prior_threshold);
  // Estimation params
  config->opt_max_iter = node->declare_parameter<int>(param_prefix + ".optimization" + ".max_iter", config->opt_max_iter);
  config->opt_threshold = node->declare_parameter<double>(param_prefix + ".optimization" + ".threshold", config->opt_threshold);
  config->cauchy_rho = node->declare_parameter<double>(param_prefix + ".optimization" + ".cauchy_rho", config->cauchy_rho);
  config->x_bias_slope = node->declare_parameter<double>(param_prefix + ".optimization" + ".x_bias_slope", config->x_bias_slope);
  config->x_bias_intercept = node->declare_parameter<double>(param_prefix + ".optimization" + ".x_bias_intercept", config->x_bias_intercept);
  config->y_bias_slope = node->declare_parameter<double>(param_prefix + ".optimization" + ".y_bias_slope", config->y_bias_slope);
  config->y_bias_intercept = node->declare_parameter<double>(param_prefix + ".optimization" + ".y_bias_intercept", config->y_bias_intercept);

  // clang-format on
  return config;
}

void DopplerExtractionModule::run_(QueryCache &qdata0, OutputCache &,
                                   const Graph::Ptr &,
                                   const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  if(!qdata.radar_data) return;

  // get the data from the radar cache
  cv::Mat fft_scan = qdata.radar_data->fft_scan;
  cv::Mat cartesian = qdata.radar_data->cartesian;
  std::vector<int64_t> azimuth_times = qdata.radar_data->azimuth_times;
  std::vector<double> azimuth_angles = qdata.radar_data->azimuth_angles;
  std::vector<bool> up_chirps = qdata.radar_data->up_chirps;
  double radar_resolution = config_->radar_resolution;

  /// boreas navtech radar upgrade time - approximately 2021-10 onwards
  static constexpr int64_t upgrade_time = 1632182400000000000;
  if (*qdata.stamp > upgrade_time){
    if(radar_resolution == 0.0596){
      CLOG(WARNING, "radar.doppler_extractor") << "Double check radar resolution: " << radar_resolution << ". Use 0.04381 for radar data after upgrade time";
    }
  } else{
    if(radar_resolution == 0.04381){
      CLOG(WARNING, "radar.doppler_extractor") << "Double check radar resolution: " << radar_resolution << ". Use 0.0596 for radar data before upgrade time";
    }  
  }

  // create a reference to the output DopplerScan
  auto &doppler_scan = *(qdata.doppler_scan.emplace());
  const auto opts = load_extractor_options(*config_);
  srd::extractor::DopplerExtractor extractor(opts);

  // Extract the doppler data
  extractor.extract_doppler(
    fft_scan,
    azimuth_angles,
    azimuth_times,
    up_chirps,
    doppler_scan);

  // RANSAC the doppler data
  auto prior_model = Eigen::Vector2d(0, 0);
  auto &w_m_r_in_r_odo_prior = qdata.w_m_r_in_r_odo_prior;
  if (w_m_r_in_r_odo_prior) {
    prior_model(0) = (*w_m_r_in_r_odo_prior)(0);
    prior_model(1) = (*w_m_r_in_r_odo_prior)(1);
  } else {
    prior_model.setZero();
  }
  extractor.ransac_scan(doppler_scan, prior_model);

  // Register the doppler scan to get final velocity estimate
  Eigen::Vector2d varpi = extractor.register_scan(doppler_scan, prior_model);

  // Check if we have negative velocity forward and flip everything
  if (varpi(0) < -0.5) {
    CLOG(WARNING, "radar.doppler_extractor") << "Negative forward velocity detected, flipping doppler measurements.";
    for (auto &d : doppler_scan) {
      d.radial_velocity = -d.radial_velocity;
    }
    varpi(0) = -varpi(0);
    varpi(1) = -varpi(1);
  }

  CLOG(DEBUG, "radar.doppler_extractor") << "Doppler data size: " << doppler_scan.size() << ", varpi: " << varpi.transpose();  
  qdata.vel_meas = varpi;
}

}  // namespace radar
}  // namespace vtr