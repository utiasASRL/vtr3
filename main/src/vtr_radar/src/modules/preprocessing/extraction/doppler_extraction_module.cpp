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
#include <pcl/common/common.h>
#include <cmath> 
#include <filesystem>
namespace fs = std::filesystem;


namespace vtr {
namespace radar {
namespace {

void cen_filter(cv::Mat &signal, int sigma_gauss, int z_q) {
  const int signal_len = signal.cols;
  double max_val;
  double min_val;
  cv::minMaxLoc(signal, &min_val, &max_val, NULL, NULL);

  // First, subtract mean of signal
  cv::Mat q = signal.clone();
  float mean = 0;
  int num_mean = 0;
  for (int i = 0; i < signal_len; ++i) {
      mean += signal.at<float>(0, i);
      num_mean++;
  }
  mean /= num_mean;
  for (int i = 0; i < signal_len; ++i) {  
      q.at<float>(0, i) = signal.at<float>(0, i) - mean;
  }

  // Apply a gaussian filter
  // TODO: binomial filter may be more efficient
  int fsize = sigma_gauss * 2 * 3;
  if (fsize % 2 == 0) fsize += 1;
  const int mu = fsize / 2;
  const float sig_sqr = sigma_gauss * sigma_gauss;
  cv::Mat filter = cv::Mat::zeros(1, fsize, CV_32F);
  float s = 0;
  for (int i = 0; i < fsize; ++i) {
      filter.at<float>(0, i) = exp(-0.5 * (i - mu) * (i - mu) / sig_sqr);
      s += filter.at<float>(0, i);
  }
  filter /= s;
  cv::Mat p;
  cv::filter2D(q, p, -1, filter, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);

  // Estimate variance of noise at each azimuth
  double sigma_q = 0;
  int nonzero = 0;
  for (int i = 0; i < signal_len; ++i) {
      float n = q.at<float>(0, i);
      if (n < 0) {
          sigma_q += 2 * (n * n);
          nonzero++;
      }
  }
  if (nonzero)
      sigma_q = 0.1 * sqrt(sigma_q / nonzero);
  else
      sigma_q = 0.034;
  // Compute threshold for zeroing out signal
  const double thres = z_q * sigma_q;

  // Filter based on distributions
  cv::Mat pow_0;
  cv::Mat pow_p;
  cv::Mat pow_qp;
  cv::Mat n0;
  cv::Mat npp;
  cv::Mat nqp;
  cv::pow(0.0 * p / sigma_q, 2, pow_0);
  cv::pow(p / sigma_q, 2, pow_p);
  cv::pow((q - p) / sigma_q, 2, pow_qp);

  cv::exp(-0.5 * pow_0, n0);
  cv::exp(-0.5 * pow_p, npp);
  cv::exp(-0.5 * pow_qp, nqp);

  //cv::divide(nqp, n0, nqp);
  //cv::divide(npp, n0, npp);

  const cv::Mat y = q.mul(1 - nqp) + p.mul(nqp - npp);

  // // Zero out signal below threshold
  cv::Mat mask = y > thres;
  mask.convertTo(mask, CV_32F);
  mask /= 255;
  signal = y.mul(mask);
}

void extract_doppler(
  const cv::Mat &fft_data,
  const std::vector<double> &azimuths,
  const std::vector<int64_t> &timestamps,
  const std::vector<bool> &chirps,
  DopplerScan &doppler_scan,
  const DopplerExtractionModule::Config &config) {

  const int N = fft_data.rows;
  const int range_bins = fft_data.cols;

  // Load parameters
  const double radar_resolution = config.radar_resolution; // meters/pixel
  const int minr = config.minr; // meters
  int maxr = config.maxr; // meters
  const double beta_up = config.beta_corr_fact * config.f_t / (config.del_f * config.meas_freq);
  const double beta_down = -beta_up;

  const int pad_num = config.pad_num;
  // Filter parameters
  const int sigma_gauss = config.sigma_gauss;
  const int z_q = config.z_q;

  // Handle case where we want to use entire range
  if (maxr <= 0) {
    maxr = range_bins * radar_resolution;
  }
  const int minr_pix = minr / radar_resolution;
  const int maxr_pix = maxr / radar_resolution;

  DopplerScan doppler_scan_temp = doppler_scan;
  auto fft_data_copy = fft_data.clone();
  cv::Point max_idx;
  double del_r = 0;
  for (int i = 0; i < N - 1; ++i) {
    double u_val = 0;
    double u_az = 0;
    long double u_time = 0;

    // Load in current and subsequent signal
    
    cv::Mat az_i = fft_data_copy.row(i).colRange(minr_pix, maxr_pix);
    cv::Mat az_i1 = fft_data_copy.row(i+1).colRange(minr_pix, maxr_pix);
    // timer[1].second->stop();

    // Filter the signals
    if (i == 0) {
        cen_filter(az_i, sigma_gauss, z_q);
    }
    cen_filter(az_i1, sigma_gauss, z_q);

    // Check if az_i or az_i1 is all zeros
    if (cv::countNonZero(az_i) == 0 || cv::countNonZero(az_i1) == 0) {
      CLOG(WARNING, "radar.doppler_extractor") << "az_i or az_i1 is all zeros";
      continue;
    }

    // Cross-correlate the signals
    cv::Mat corr;
    // Pad az_i1 by 0's before and after
    cv::Mat az_i1_padded = cv::Mat::zeros(1, az_i.cols + 2*pad_num, CV_32F);
    az_i1.copyTo(az_i1_padded.colRange(pad_num, az_i.cols + pad_num));
    // Perform cross-correlation
    cv::matchTemplate(az_i, az_i1_padded, corr, cv::TM_CCORR_NORMED);
    cv::minMaxLoc(corr, NULL, NULL, NULL, &max_idx);

    // Compute shift and velocity
    del_r = (max_idx.x - pad_num) / 2.0;

    // If chirps[i] is true, then this chirp corresponds to an up chirp
    if (chirps[i]) {
      u_val = del_r * radar_resolution / beta_up;
    } else {
      u_val = del_r * radar_resolution / beta_down;
    }

    // Skip obvious outliers
    if (u_val > 30) {
        continue;
    }

    // Compute the intermediate azimuth
    u_az = (azimuths[i] + azimuths[i+1]) / 2.0;

    // Compute the intermediate time
    // Convert timestamps to seconds
    long double t1 = static_cast<long double>(timestamps[i]) / 1e6;
    long double t2 = static_cast<long double>(timestamps[i+1]) / 1e6;
    u_time = static_cast<int64_t>((t1 + t2) / 2.0 * 1e6);

    Azimuth u_i; // Initialize azimuth struct to be added to doppler_scan
    u_i.azimuth = u_az;
    u_i.timestamp = u_time;
    u_i.radial_velocity = u_val;
    u_i.azimuth_idx = i;
    // std::cout << "u_i: " << u_i.azimuth << " " << u_time - timestamps[0] << " " << u_i.radial_velocity << std::endl;
    doppler_scan_temp.push_back(u_i);
  }

  doppler_scan = doppler_scan_temp;
}

void ransac_scan(
  DopplerScan &doppler_scan,
  Eigen::Vector2d &prior_model,
  const DopplerExtractionModule::Config &config) {
  // Load parameters
  int vel_dim = 2;
  int ransac_max_iter = config.ransac_max_iter;
  double ransac_threshold = config.ransac_threshold;
  double ransac_prior_threshold = config.ransac_prior_threshold;
  int num_meas = doppler_scan.size();
  int best_num_inlier = 0;

  // Check feasibility
  if (doppler_scan.size() < 2) {
    CLOG(WARNING, "radar.doppler_extractor") << "Not enough points to perform RANSAC";
    return;
  }

  Eigen::VectorXd residuals;
  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_meas);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_meas, vel_dim);
  for (int i = 0; i < num_meas; ++i) {
    b(i) = doppler_scan[i].radial_velocity;
    A(i, 0) = cos(doppler_scan[i].azimuth);
    A(i, 1) = sin(doppler_scan[i].azimuth);
  }

  std::vector<int> inlier_mask = std::vector<int>(num_meas, 0);
  std::vector<int> best_inlier_mask = std::vector<int>(num_meas, 0);
  Eigen::VectorXd model;
  Eigen::VectorXd best_model;

  for (int iter = 0; iter < ransac_max_iter; ++iter) {
      // Randomly sample 2 points
      std::vector<int> idx;
      for (int i = 0; i < 2; ++i) {
          idx.push_back(rand() % A.rows());
      }

      Eigen::MatrixXd A_sample = Eigen::MatrixXd::Zero(2, vel_dim);
      Eigen::VectorXd b_sample = Eigen::VectorXd::Zero(2);
      for (int i = 0; i < 2; ++i) {
          A_sample.row(i) = A.row(idx[i]);
          b_sample(i) = b(idx[i]);
      }

      // Fit line to sample
      model = A_sample.inverse() * b_sample;

      // Reject models that are too different to prior
      double prior_model_err = (model - prior_model).norm();
      if (prior_model_err > ransac_prior_threshold) {
          continue;
      }

      // Compute residuals
      residuals = A * model - b;

      // Compute inliers
      for (int i = 0; i < residuals.size(); ++i) {
          if (std::abs(residuals(i)) < ransac_threshold) {
              inlier_mask[i] = 1;
          } else {
              inlier_mask[i] = 0;
          }
      }

      int num_inlier = Eigen::Map<Eigen::ArrayXi>(inlier_mask.data(), inlier_mask.size()).count();

      if (num_inlier > best_num_inlier) {
          best_num_inlier = num_inlier;
          best_inlier_mask = inlier_mask;
          best_model = model;
      }
  }

  // If we have less than 10 inliers, then just use the prior model
  if (best_num_inlier < 10) {
      residuals = A * prior_model - b;
      for (int i = 0; i < residuals.size(); ++i) {
          if (std::abs(residuals(i)) < ransac_threshold) {
              inlier_mask[i] = 1;
          } else {
              inlier_mask[i] = 0;
          }
      }
      best_inlier_mask = inlier_mask;
  }

  // Remove outliers
  DopplerScan ransac_scan;
  for (int i = 0; i < num_meas; ++i) {
      if (best_inlier_mask[i]) {
          ransac_scan.push_back(doppler_scan[i]);
      }
  }
  
  // Overwrite doppler_scan with ransac_scan
  doppler_scan = ransac_scan;
}

Eigen::Vector2d registerScan(
  const DopplerScan &doppler_scan,
  Eigen::Vector2d &prior_model,
  const DopplerExtractionModule::Config &config) {
  // Load in parameters for easy reference
  int vel_dim = 2;
  int num_meas = doppler_scan.size();

  // initialize doppler measurement residual and A/b matrices for least squares
  Eigen::VectorXd residuals = Eigen::VectorXd::Zero(num_meas);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_meas, vel_dim);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_meas);
  for (int i = 0; i < num_meas; ++i) {
    b(i) = doppler_scan[i].radial_velocity;
    A(i, 0) = cos(doppler_scan[i].azimuth);
    A(i, 1) = sin(doppler_scan[i].azimuth);
  }

  // compute cauchy weights based on latest residual
  Eigen::VectorXd w_inv_diag = Eigen::VectorXd::Ones(num_meas);
  Eigen::VectorXd cauchy_w = Eigen::VectorXd::Ones(num_meas);
  Eigen::VectorXd trim_w = Eigen::VectorXd::Ones(num_meas);
  Eigen::VectorXd varpi_prev = Eigen::VectorXd::Zero(vel_dim);
  Eigen::VectorXd varpi_curr = Eigen::VectorXd::Zero(vel_dim);
  Eigen::MatrixXd lhs = Eigen::MatrixXd::Zero(vel_dim, vel_dim);
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(vel_dim);

  // Load in initial guess to be used for the first iteration
  varpi_prev = prior_model;
  varpi_curr = varpi_prev;

  // Decide on cauchy rho based on the velocity
  double cauchy_rho = config.cauchy_rho;
  double trim_dist = config.trim_dist;

  // run least squares on Ax = b
  for (int iter = 0; iter < config.opt_max_iter; ++iter) {
    // compute new cauchy weights
    residuals = A * varpi_curr - b;
    cauchy_w = 1.0 / (1.0 + ( residuals.array() / cauchy_rho ).square());

    // Implement trim weight where errors above 0.5 have weight of 0
    //trim_w = (residuals.array().abs() < trim_dist).cast<double>();

    w_inv_diag = cauchy_w.array();

    lhs = A.transpose() * w_inv_diag.asDiagonal() * A;
    rhs = A.transpose() * w_inv_diag.asDiagonal() * b;

    // solve
    varpi_prev = varpi_curr;
    varpi_curr = lhs.inverse() * rhs;
    
    // Check for convergence
    if ((varpi_curr - varpi_prev).norm() < config.opt_threshold) {
      break;
    }
  }

  // Only correct bias in x if we're confident we're moving
  // This bias is calibrated only for x_vel > 0.2
  if (std::abs(varpi_curr(0)) > 0.2) {
    varpi_curr(0) = varpi_curr(0) + varpi_curr(0) * config.x_bias_slope + config.x_bias_intercept;
  }
  varpi_curr(1) = varpi_curr(1) + varpi_curr(1) * config.y_bias_slope + config.y_bias_intercept;

  return varpi_curr;
}

Eigen::Matrix3d toRoll(const double &r) {
  Eigen::Matrix3d roll;
  roll << 1, 0, 0, 0, cos(r), sin(r), 0, -sin(r), cos(r);
  return roll;
}

Eigen::Matrix3d toPitch(const double &p) {
  Eigen::Matrix3d pitch;
  pitch << cos(p), 0, -sin(p), 0, 1, 0, sin(p), 0, cos(p);
  return pitch;
}

Eigen::Matrix3d toYaw(const double &y) {
  Eigen::Matrix3d yaw;
  yaw << cos(y), sin(y), 0, -sin(y), cos(y), 0, 0, 0, 1;
  return yaw;
}

Eigen::Matrix3d rpy2rot(const double &r, const double &p, const double &y) {
  return toRoll(r) * toPitch(p) * toYaw(y);
}

// void load_gt_pose(std::ifstream &pose_stream, Eigen::Matrix<double, 6, 1> &v_gt) {
//   std::string gt_pose;
//   std::getline(pose_stream, gt_pose);
//   std::stringstream ss(gt_pose);
//   std::vector<double> gt;
//   for (std::string str; std::getline(ss, str, ',');)
//           gt.push_back(std::stod(str));
  
//   Eigen::Matrix4d T_ab = Eigen::Matrix4d::Identity();
//   T_ab.block<3, 3>(0, 0) = rpy2rot(gt[7], gt[8], gt[9]);
//   T_ab.block<3, 1>(0, 3) << gt[1], gt[2], gt[3];

//   // Save ground truth linear velocity
//   Eigen::Vector4d v_a_tilde(gt[4], gt[5], gt[6], 1);
//   Eigen::Vector4d v_b_tilde = T_ab.transpose() * v_a_tilde;
//   v_gt.head<3>() = v_b_tilde.block<3, 1>(0, 0);

//   // Save ground truth angular velocity (its saved as angvel_z,angvel_y,angvel_x)
//   v_gt.tail<3>() << gt[12], gt[11], gt[10];
// }

double quadraticInterpolation(int64_t t, int64_t t0, double v0, int64_t t1, double v1, int64_t t2, double v2) {
  double td = static_cast<double>(t);
  double td0 = static_cast<double>(t0);
  double td1 = static_cast<double>(t1);
  double td2 = static_cast<double>(t2);

  double L0 = ((td - td1) * (td - td2)) / ((td0 - td1) * (td0 - td2));
  double L1 = ((td - td0) * (td - td2)) / ((td1 - td0) * (td1 - td2));
  double L2 = ((td - td0) * (td - td1)) / ((td2 - td0) * (td2 - td1));

  return v0 * L0 + v1 * L1 + v2 * L2;
}

Eigen::Matrix<double, 2, 1> getVel(std::vector<double> &gt) {
  Eigen::Matrix4d T_ab = Eigen::Matrix4d::Identity();
  T_ab.block<3, 3>(0, 0) = rpy2rot(std::round(gt[7] / M_PI) * M_PI, std::round(gt[8] / M_PI) * M_PI, gt[9]); // RPY assumed in radians
  T_ab.block<3, 1>(0, 3) << gt[1], gt[2], gt[3];

  // Linear velocity in B
  Eigen::Vector4d v_a_tilde(gt[4], gt[5], gt[6], 1);
  Eigen::Vector4d v_b_tilde = T_ab.transpose() * v_a_tilde;
  Eigen::Matrix<double, 2, 1> v_gt = v_b_tilde.head<2>();
  return v_gt;

  // // Angular velocity in B
  // v_gt.tail<3>() << gt[12], gt[11], gt[10];
}

bool load_gt_velocities(std::ifstream &pose_stream, std::vector<Eigen::Matrix<double, 2, 1>> &v_gts, std::vector<int64_t> azimuth_timestamps, int64_t target_timestamp) {
  std::string line;
  std::vector<double> prev_gt;
  std::vector<double> gt;
  std::vector<double> next_gt;
  while (std::getline(pose_stream, line)) {
      std::stringstream ss(line);
      gt = std::vector<double>();
      for (std::string str; std::getline(ss, str, ','); )
          gt.push_back(std::stod(str));

      if (gt[0] == target_timestamp/1000) {
        if (std::getline(pose_stream, line)) {
          std::getline(pose_stream, line);
          std::stringstream ss(line);
          for (std::string str; std::getline(ss, str, ','); )
            next_gt.push_back(std::stod(str));
        }

        if (prev_gt.empty() || next_gt.empty()) {
          v_gts = std::vector<Eigen::Matrix<double, 2, 1>>(azimuth_timestamps.size(), Eigen::Matrix<double, 2, 1>::Zero());
        } else {
          const auto v_prev = getVel(prev_gt);
          const auto time_prev = prev_gt[0];
          const auto v_curr = getVel(gt);
          const auto time_curr = gt[0];
          const auto v_next = getVel(next_gt);
          const auto time_next = next_gt[0];
          // Interpolate velocity at azimuth timestamps
          for (size_t ii = 0; ii < azimuth_timestamps.size(); ++ii) {
            const auto t_az = azimuth_timestamps[ii] / 1000;
            const double v_fwd = quadraticInterpolation(t_az-time_prev, 0, v_prev[0], time_curr-time_prev, v_curr[0], time_next-time_prev, v_next[0]);
            const double v_bwd = quadraticInterpolation(t_az-time_next, 0, v_next[1], time_curr-time_next, v_curr[1], time_prev-time_next, v_prev[1]);
            v_gts.push_back(Eigen::Matrix<double, 2, 1>(v_fwd, v_bwd));
          }
        }

        return true;
      } else {
        prev_gt = gt;
      }
  }

  return false; // timestamp not found
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
  config->trim_dist = node->declare_parameter<double>(param_prefix + ".optimization" + ".trim_dist", config->trim_dist);
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

  // create a reference to the output DopplerScan
  auto &doppler_scan = *(qdata.doppler_scan.emplace());

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

  // // Extract the doppler data
  // extract_doppler(fft_scan, azimuth_angles, azimuth_times, up_chirps, doppler_scan, *config_);
  // // RANSAC the doppler data
  // auto prior_model = Eigen::Vector2d(0, 0);
  // auto &w_m_r_in_r_odo_prior = qdata.w_m_r_in_r_odo_prior;
  // if (w_m_r_in_r_odo_prior) {
  //   prior_model(0) = (*w_m_r_in_r_odo_prior)(0);
  //   prior_model(1) = (*w_m_r_in_r_odo_prior)(1);
  // } else {
  //   prior_model.setZero();
  // }
  // ransac_scan(doppler_scan, prior_model, *config_);
  // // Publish the doppler data

  // Eigen::Vector2d varpi = registerScan(doppler_scan, prior_model, *config_);

  // CLOG(DEBUG, "radar.doppler_extractor") << "Doppler data size: " << doppler_scan.size() << ", varpi: " << varpi.transpose();
  // const auto &vel_meas = *qdata.vel_meas;
  
  // qdata.vel_meas = varpi;

  // Load in gt for testing
  const auto &sequence_name = *qdata.seq_name;
  const fs::path pose_file = fs::path("/home/dl/Documents/phd/dev/doppler_radar/data/vtr_data") / sequence_name / "applanix" / "radar_poses.csv";

  std::ifstream pose_stream(pose_file, std::ios::in);
  std::string header;
  std::getline(pose_stream, header);
  std::vector<Eigen::Matrix<double, 2, 1>> w_r_v_radar_gts;
  // Eigen::Matrix<double, 6, 1> w_r_v_robot;
  bool found_gt = load_gt_velocities(pose_stream, w_r_v_radar_gts, azimuth_times, *qdata.stamp);

  if (!found_gt) {
    CLOG(ERROR, "radar.doppler_extractor") << "Ground truth not found for timestamp: " << *qdata.stamp;
  }

  // const auto &T_s_r = *qdata.T_s_r;
  // // Transform v_gt to the radar frame
  // w_r_v_radar(2) = 0;
  // w_r_v_radar(3) = 0;
  // w_r_v_radar(4) = 0;
  // w_r_v_robot = lgmath::se3::tranAd(T_s_r.matrix().inverse()) * w_r_v_radar;

  qdata.vel_meas = w_r_v_radar_gts[azimuth_times.size() / 2];

  CLOG(DEBUG, "radar.doppler_extractor") << w_r_v_radar_gts[azimuth_times.size() / 2];

  // Fill up doppler_scan with gt data
  for (size_t ii = 0; ii < azimuth_angles.size(); ++ii) {
    const Eigen::Vector2d v_gt = w_r_v_radar_gts[ii];
    Azimuth u_ii;
    u_ii.azimuth = azimuth_angles[ii];
    u_ii.timestamp = azimuth_times[ii];
    u_ii.radial_velocity = v_gt[0] * std::cos(u_ii.azimuth) + v_gt[1] * std::sin(u_ii.azimuth);
    u_ii.azimuth_idx = ii;
    doppler_scan.push_back(u_ii);
  }

}

}  // namespace radar
}  // namespace vtr