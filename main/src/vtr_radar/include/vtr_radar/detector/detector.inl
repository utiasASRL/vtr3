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
 * \file detector.inl
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 * \brief Keypoint extraction methods for Navtech radar
 */
#pragma once

#include "vtr_radar/detector/detector.hpp"

namespace vtr {
namespace radar {

namespace {
bool sort_desc_by_first(const std::pair<float, int> &a,
                        const std::pair<float, int> &b) {
  return (a.first > b.first);
}
bool sort_asc_by_second(const std::pair<int, float> &a,
                        const std::pair<int, float> &b) {
  return (a.second < b.second);
}

} // namespace
//  K-peaks implementation with scale correction due to range quantization errors
template <class PointT>
void KPeaks<PointT>::run(const cv::Mat &raw_scan, const float &res,
                             const std::vector<int64_t> &azimuth_times,
                             const std::vector<double> &azimuth_angles,
                             const std::vector<bool> &up_chirps,
                             pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  auto mincol = minr_ / res;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  for (int i = 0; i < rows; ++i) {
    std::vector<std::pair<float, int>> intens;
    const float thres = threshold3_;
    // 1. Collect threshold-exceeding points (in column order)
    for (int j = mincol; j < maxcol; ++j) {
      if (raw_scan.at<float>(i, j) >= thres) {
        intens.emplace_back(raw_scan.at<float>(i, j), j);
      }
    }
    if (!intens.empty()) {
      // 2. Group adjacent bins into peaks
      std::vector<std::pair<float, std::vector<int>>> peaks;
      float current_max = intens[0].first;
      std::vector<int> current_bins = {intens[0].second};
      for (size_t idx = 1; idx < intens.size(); ++idx) {
        const int current_j = intens[idx].second;
        if (current_j == current_bins.back() + 1) {
          // Continuation of current peak
          current_max = std::max(current_max, intens[idx].first);
          current_bins.push_back(current_j);
        } else {
          // Finalize current peak
          peaks.emplace_back(current_max, current_bins);
          current_max = intens[idx].first;
          current_bins = {current_j};
        }
      }
      peaks.emplace_back(current_max, current_bins);  // Add last peak
      // 3. Sort peaks by maximum intensity
      std::sort(peaks.begin(), peaks.end(),
               [](const auto& a, const auto& b) { return a.first > b.first; });
      // 4. Select top-k peaks with averaged positions
      const double azimuth = azimuth_angles[i];
      const int64_t time = azimuth_times[i];
      pcl::PointCloud<PointT> polar_time;
      for (int p = 0; p < std::min(kstrong_, (int)peaks.size()); ++p) {
        const auto& peak = peaks[p];
        const float avg_j = std::accumulate(peak.second.begin(),
                                           peak.second.end(), 0.0f)
                           / peak.second.size();
        PointT point;
        point.rho = (avg_j * res) + static_cast<float>(range_offset_);
        point.phi = azimuth;
        point.theta = 0;
        point.timestamp = time;
        polar_time.push_back(point);
      }
      pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
    }
  }
}


// K-strongest implementation as per Elliot's paper
template <class PointT>
void KStrongest<PointT>::run(const cv::Mat &raw_scan, const float &res,
                             const std::vector<int64_t> &azimuth_times,
                             const std::vector<double> &azimuth_angles,
                             const std::vector<bool> &up_chirps,
                             pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  auto mincol = minr_ / res;

  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res;

  if (maxcol > cols || maxcol < 0) maxcol = cols;

  // #pragma omp parallel for
  for (int i = 0; i < rows; ++i) {
    std::vector<std::pair<float, int>> intens;

    const float thres = static_threshold_;
    for (int j = mincol; j < maxcol; ++j) {
      if (raw_scan.at<float>(i, j) >= thres){
        intens.push_back(std::make_pair(raw_scan.at<float>(i, j), j));
      }
    }

    int thresholded_point_count = intens.size();
    if(thresholded_point_count > 0){
      // sort intensities in descending order
      std::sort(intens.begin(), intens.end(), sort_desc_by_first);

      const double azimuth = azimuth_angles[i];
      const int64_t time = azimuth_times[i];
      const bool up_chirp = up_chirps[i];
      pcl::PointCloud<PointT> polar_time;
      for (int j = 0; j < kstrong_; ++j) {
        if (j >= thresholded_point_count){break;}
        PointT p;
        p.rho = static_cast<float>(intens[j].second) * res + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        p.up_chirp = up_chirp;
        polar_time.push_back(p);
      }
      // #pragma omp critical
      {
        pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
      }
    }
  }
}

template <class PointT>
void Cen2018<PointT>::run(const cv::Mat &raw_scan, const float &res,
                          const std::vector<int64_t> &azimuth_times,
                          const std::vector<double> &azimuth_angles,
                          const std::vector<bool> &up_chirps,
                          pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  auto mincol = minr_ / res;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const auto N = maxcol - mincol;

  std::vector<float> sigma_q(rows, 0);

  // Convert raw signal to dB
  cv::Mat raw_scan_dB = raw_scan.clone() *255/2;

  // TODO: try implementing an efficient median filter
  // Estimate the bias and subtract it from the signal
  cv::Mat q = raw_scan_dB.clone();
  for (int i = 0; i < rows; ++i) {
    double mean = 0;
    for (int j = mincol; j < maxcol; ++j) {
      mean += raw_scan_dB.at<float>(i, j);
    }
    mean /= N;
    for (int j = mincol; j < maxcol; ++j) {
      q.at<float>(i, j) = raw_scan_dB.at<float>(i, j) - mean;
    }
  }

  // Create 1D Gaussian Filter
  // TODO: binomial filter may be more efficient
  int fsize = sigma_ * 2 * 3;
  if (fsize % 2 == 0) fsize += 1;
  // const int mu = fsize / 2;
  // const float sig_sqr = sigma_ * sigma_;
  // cv::Mat filter = cv::Mat::zeros(1, fsize, CV_32F);
  // float s = 0;
  // for (int i = 0; i < fsize; ++i) {
  //   filter.at<float>(0, i) = exp(-0.5 * (i - mu) * (i - mu) / sig_sqr);
  //   s += filter.at<float>(0, i);
  // }
  // filter /= s;

  cv::Mat p;
  // cv::filter2D(q, p, -1, filter, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);

  cv::GaussianBlur(q,p,cv::Size(fsize,1),sigma_,sigma_);
  
  // Estimate variance of noise at each azimuth
  for (int i = 0; i < rows; ++i) {
    int nonzero = 0;
    for (int j = mincol; j < maxcol; ++j) {
      double n = q.at<float>(i, j);
      if (n < 0) {
        sigma_q[i] += 2 * (n * n);
        nonzero++;
      }
    }
    if (nonzero)
      sigma_q[i] = sqrt(sigma_q[i] / nonzero);
    else
      sigma_q[i] = 0.034;
  }

  // Extract peak centers from each azimuth
  // #pragma omp parallel for
  for (int i = 0; i < rows; ++i) {
    pcl::PointCloud<PointT> polar_time;
    float peak_points = 0;
    int num_peak_points = 0;
    const float thres = zq_ * sigma_q[i];
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    const bool up_chirp = up_chirps[i];
    for (int j = mincol; j < maxcol; ++j) {
      const float nqp = exp(
          -0.5 * pow((q.at<float>(i, j) - p.at<float>(i, j)) / sigma_q[i], 2));
      const float npp = exp(-0.5 * pow(p.at<float>(i, j) / sigma_q[i], 2));
      const float b = nqp - npp;
      const float y = q.at<float>(i, j) * (1 - nqp) + p.at<float>(i, j) * b;
      if (y > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        p.up_chirp = up_chirp;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    }

    // #pragma omp critical
    {
      pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
    }
  }
}

template <class PointT>
void OSCFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                         const std::vector<int64_t> &azimuth_times,
                         const std::vector<double> &azimuth_angles,
                         const std::vector<bool> &up_chirps,
                         pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2;
  if (maxcol > cols || maxcol < 0) maxcol = cols;

  float conversion_factor = 255.0/20.0*std::log(10.0);
  cv::Mat raw_scan_watts_sqrd = raw_scan * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  // #pragma omp parallel for
  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    const bool up_chirp = up_chirps[i];
    pcl::PointCloud<PointT> polar_time;

    std::vector<std::pair<int, float>> window;
    window.reserve(width_ - 1);
    int j = mincol - 1;
    for (int k = -w2; k < 0; ++k) {
      window.emplace_back(j + k, raw_scan_watts_sqrd.at<float>(i, j + k));
    }
    for (int k = 1; k <= w2; ++k) {
      window.emplace_back(j + k, raw_scan_watts_sqrd.at<float>(i, j + k));
    }
    std::sort(window.begin(), window.end(), sort_asc_by_second);

    float peak_points = 0;
    int num_peak_points = 0;

    for (j = mincol; j < maxcol; ++j) {
      // remove cell under test and left-most cell
      window.erase(std::remove_if(window.begin(), window.end(),
                                  [&](const std::pair<float, int> &p) -> bool {
                                    return p.first == j ||
                                           p.first == j - w2 - 1;
                                  }),
                   window.end());
      // insert prev CUT and right-most cell
      auto prevcut = std::make_pair(j - 1, raw_scan_watts_sqrd.at<float>(i, j - 1));
      auto it = std::lower_bound(window.begin(), window.end(), prevcut,
                                 sort_asc_by_second);
      window.insert(it, prevcut);
      auto newentry = std::make_pair(j + w2, raw_scan_watts_sqrd.at<float>(i, j + w2));
      it = std::lower_bound(window.begin(), window.end(), newentry,
                            sort_asc_by_second);
      window.insert(it, newentry);

      // DEBUG: check that the window is indeed sorted:
      // assert(window.size() == width_ - 1);
      // for (size_t k = 1; k < window.size(); ++k) {
      //   if (window[k - 1].second > window[k].second) {
      //     throw std::runtime_error("window not sorted.");
      //   }
      // }
      // (statistic) estimate of clutter power
      double stat = window[kstat_].second;
      const float thres = threshold_ * stat;

      if (raw_scan_watts_sqrd.at<float>(i, j) > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {        
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        p.up_chirp = up_chirp;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    }
    // #pragma omp critical
    {
      pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
    }
  }
}

template <class PointT>
void TM_CFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                         const std::vector<int64_t> &azimuth_times,
                         const std::vector<double> &azimuth_angles,
                         const std::vector<bool> &up_chirps,
                         pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2;
  if (maxcol > cols || maxcol < 0) maxcol = cols;

  float conversion_factor = 255.0/20.0*std::log(10.0);
  cv::Mat raw_scan_watts_sqrd = raw_scan * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  // #pragma omp parallel for
  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    const bool up_chirp = up_chirps[i];
    pcl::PointCloud<PointT> polar_time;

    std::vector<std::pair<int, float>> window;
    window.reserve(width_ - 1);
    int j = mincol - 1;
    for (int k = -w2; k < 0; ++k) {
      window.emplace_back(j + k, raw_scan_watts_sqrd.at<float>(i, j + k));
    }
    for (int k = 1; k <= w2; ++k) {
      window.emplace_back(j + k, raw_scan_watts_sqrd.at<float>(i, j + k));
    }
    std::sort(window.begin(), window.end(), sort_asc_by_second);

    float peak_points = 0;
    int num_peak_points = 0;

    // Set both N1 and N2 to the same value for simplicity
    int N1 = N1_;
    int N2 = N1_;

    for (j = mincol; j < maxcol; ++j) {
      // remove cell under test and left-most cell
      window.erase(std::remove_if(window.begin(), window.end(),
                                  [&](const std::pair<float, int> &p) -> bool {
                                    return p.first == j ||
                                           p.first == j - w2 - 1;
                                  }),
                   window.end());
      // insert prev CUT and right-most cell
      auto prevcut = std::make_pair(j - 1, raw_scan_watts_sqrd.at<float>(i, j - 1));
      auto it = std::lower_bound(window.begin(), window.end(), prevcut,
                                 sort_asc_by_second);
      window.insert(it, prevcut);
      auto newentry = std::make_pair(j + w2, raw_scan_watts_sqrd.at<float>(i, j + w2));
      it = std::lower_bound(window.begin(), window.end(), newentry,
                            sort_asc_by_second);
      window.insert(it, newentry);

      // Copy window
      std::vector<std::pair<int, float>> trimmed_window = window;

      // Remove first N1 elements and last N2 elements
      if (N1 > 0 && N1 <= static_cast<int>(trimmed_window.size())) {
        trimmed_window.erase(trimmed_window.begin(), trimmed_window.begin() + N1);
      }
      if (N2 > 0 && N2 <= static_cast<int>(trimmed_window.size())) {
        trimmed_window.erase(trimmed_window.end() - N2, trimmed_window.end());
      }

      double sum = 0;
      for (const auto& cell : trimmed_window) {sum += cell.second;}
      double stat = sum / static_cast<int>(trimmed_window.size());
      
      const float thres = threshold_ * stat;


      if (raw_scan_watts_sqrd.at<float>(i, j) > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        p.up_chirp = up_chirp;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    }

    // #pragma omp critical
    {
      pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
    }
  }
}

template <class PointT>
void CACFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 const std::vector<bool> &up_chirps,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;

  float conversion_factor = 255.0/20.0*std::log(10.0);
  cv::Mat raw_scan_watts_sqrd = raw_scan * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    const bool up_chirp = up_chirps[i];
    pcl::PointCloud<PointT> polar_time;
    float peak_points = 0;
    int num_peak_points = 0;

    double left = 0;
    double right = 0;
    double mean = 0;

    for (int k = -w2 - guard_; k < -guard_; ++k){
      left += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));
    }

    for (int k = guard_ + 1; k <= w2 + guard_; ++k){
      right += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));
    }

    for (int j = mincol; j < maxcol; ++j) {
      // Intensity of test cell Y with sqaure law
      float intensity_Y=raw_scan_watts_sqrd.at<float>(i, j);

      left = left - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + -w2 - guard_ - 1)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j -guard_ - 1));
      right = right - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + guard_)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + w2 + guard_));
      mean = (left+right) / (2 * w2);

      const float thres = threshold_ * mean;

      if (intensity_Y > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        p.up_chirp = up_chirp;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    } 

    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
}

template <class PointT>
void ModifiedCACFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 const std::vector<bool> &up_chirps,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const int N = maxcol - mincol;

  float conversion_factor = 255.0/20.0*std::log(10.0);
  cv::Mat raw_scan_watts_sqrd = raw_scan * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  double static_threshold_watts = std::pow(10,(255.0*threshold3_/20.0));
  double static_threshold_squared = static_threshold_watts*static_threshold_watts;
  if(threshold3_ == 0.0){static_threshold_squared=0.0;}

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    const bool up_chirp = up_chirps[i];
    pcl::PointCloud<PointT> polar_time;
    double mean = 0;
    for (int j = mincol; j < maxcol; ++j) {
      mean += raw_scan_watts_sqrd.at<float>(i, j);
    }
    mean /= N;

    float peak_points = 0;
    int num_peak_points = 0;

    for (int j = mincol; j < maxcol; ++j) {
      double left = 0;
      double right = 0;
      for (int k = -w2 - guard_; k < -guard_; ++k)
        left += raw_scan_watts_sqrd.at<float>(i, j + k);
      for (int k = guard_ + 1; k <= w2 + guard_; ++k)
        right += raw_scan_watts_sqrd.at<float>(i, j + k);
      // (statistic) estimate of clutter power
      // const double stat = (left + right) / (2 * w2);
      const double stat = std::max(left, right) / w2;  // GO-CFAR  // use max min or average
      const float thres = threshold_ * stat + threshold2_ * mean + static_threshold_squared;
      if (raw_scan_watts_sqrd.at<float>(i, j) > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + range_offset_;
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        p.up_chirp = up_chirp;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    }
    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
}

template <class PointT>
void CAGO_CFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 const std::vector<bool> &up_chirps,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;

  // Convert Navtechs 8-bit dB half steps to watts
  cv::Mat raw_scan_watts_sqrd = raw_scan.clone();
  float conversion_factor = 255.0/20.0*std::log(10.0);
  raw_scan_watts_sqrd = raw_scan_watts_sqrd * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    const bool up_chirp = up_chirps[i];
    pcl::PointCloud<PointT> polar_time;
    float peak_points = 0;
    int num_peak_points = 0;

    double left = 0;
    double right = 0;
    double mean = 0;

    for (int k = -w2 - guard_; k < -guard_; ++k){
      left += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));
    }
    for (int k = guard_ + 1; k <= w2 + guard_; ++k){
      right += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));
    }

    for (int j = mincol; j < maxcol; ++j) {
      // Intensity of test cell Y with sqaure law
      float intensity_Y= raw_scan_watts_sqrd.at<float>(i, j);

      left = left - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + -w2 - guard_ - 1)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j -guard_ - 1));
      right = right - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + guard_)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + w2 + guard_));

      mean = std::max(left, right) / w2;

      const float thres = threshold_ * mean;

      if (intensity_Y > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        p.up_chirp = up_chirp;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    } 

    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
}

template <class PointT>
void CASO_CFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 const std::vector<bool> &up_chirps,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;

  // Convert Navtechs 8-bit dB half steps to watts
  cv::Mat raw_scan_watts_sqrd = raw_scan.clone();
  double conversion_factor = 255.0/20.0*std::log(10.0);
  raw_scan_watts_sqrd = raw_scan_watts_sqrd * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    const bool up_chirp = up_chirps[i];
    pcl::PointCloud<PointT> polar_time;
    double mean = 0;
    float peak_points = 0;
    int num_peak_points = 0;

    double left = 0;
    double right = 0;
    for (int k = -w2 - guard_; k < -guard_; ++k){
      left += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));
    }
    for (int k = guard_ + 1; k <= w2 + guard_; ++k){
      right += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));
    }

    for (int j = mincol; j < maxcol; ++j) {
      // Intensity of test cell Y with sqaure law
      float intensity_Y= raw_scan_watts_sqrd.at<float>(i, j);

      left = left - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + -w2 - guard_ - 1)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j -guard_ - 1));
      right = right - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + guard_)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + w2 + guard_));

      mean = std::min(left, right) / w2;

      const float thres = threshold_ * mean;

      if (intensity_Y > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        p.up_chirp = up_chirp;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    } 
    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
}


struct hash_pair {
    template <class T1, class T2>
    size_t operator()(const std::pair<T1, T2>& p) const
    {
        // Hash the first element
        size_t hash1 = std::hash<T1>{}(p.first);
        // Hash the second element
        size_t hash2 = std::hash<T2>{}(p.second);
        // Combine the two hash values
        return hash1 ^ (hash2 + 0x9e3779b9 + (hash1 << 6) + (hash1 >> 2));
    }
};

template <class PointT>
void CFEAR_KStrong<PointT>::run(const cv::Mat &raw_scan, const float &res,
                             const std::vector<int64_t> &azimuth_times,
                             const std::vector<double> &azimuth_angles,
                             const std::vector<bool> &up_chirps,
                             pcl::PointCloud<PointT> &pointcloud) {

  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  auto mincol = minr_ / res;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res;
  if (maxcol > cols || maxcol < 0) maxcol = cols;

  cv::Mat point_overlay = cv::Mat::zeros(rows,cols,CV_32F);

  // float grid_size = r_/f_;
  float grid_size = 0.5;

  std::vector<std::pair<float,float>> P_f;
  std::vector<std::pair<float,float>> P_d;
  std::unordered_map<std::pair<float, float>, std::vector<std::pair<float,float>>, hash_pair> P_d_points;

  for (int i = 0; i < rows; ++i) {
    std::vector<std::pair<float, int>> intens;
  
    const float thres = z_min_;

    for (int j = mincol; j < maxcol; ++j) {
      if (raw_scan.at<float>(i, j) >= thres){
        intens.push_back(std::make_pair(raw_scan.at<float>(i, j), j));
      }
    }

    int thresholded_point_count = intens.size();
    if(thresholded_point_count > 0){
      // sort intensities in descending order
      std::sort(intens.begin(), intens.end(), sort_desc_by_first);

      pcl::PointCloud<PointT> polar_time;
      for (int j = 0; j < kstrong_; ++j) {
        if (j >= thresholded_point_count){break;}
        const double az = azimuth_angles[i];
        float cart_x = (intens[j].second * res + range_offset_) * cos(az);
        float cart_y = (intens[j].second * res + range_offset_) * sin(az);

        P_f.push_back(std::make_pair(cart_x,cart_y));
      }
    }
  }

  for(auto&[p_x,p_y] : P_f){

    int grid_x = p_x / grid_size;
    int grid_y = p_y / grid_size;

    float center_x = grid_x * grid_size + grid_size / 2;
    float center_y = grid_y * grid_size + grid_size / 2;

    P_d.push_back(std::make_pair(center_x,center_y));
  }

  for(auto& [pd_x,pd_y] : P_d){
    std::vector<std::pair<float,float>> point_set;
    std::vector<std::pair<float,float>> polar_point_set;

    for(auto& [pf_x,pf_y]: P_f){
      float dx = pd_x-pf_x;
      float dy = pd_y-pf_y;
      float diff = std::sqrt(dx*dx + dy*dy);
  
      if(diff < r_){
        point_set.push_back(std::make_pair(pf_x,pf_y));
      }

    }
    P_d_points[std::make_pair(pd_x,pd_y)] = point_set;
  }

  for(auto& [grid_point,filtered_points] : P_d_points){
    pcl::PointCloud<PointT> polar_time;
  
    float sample_mean_x = 0.0, sample_mean_y = 0.0;

    // Compute the sample mean
    int N_f = filtered_points.size();
    if(N_f < 6){continue;}

    for(auto& [p_x,p_y] : filtered_points){
      sample_mean_x += p_x;
      sample_mean_y += p_y;
    }
    sample_mean_x /= N_f;
    sample_mean_y /= N_f;

    // Compute the sample covariance
    float sum_xx = 0.0, sum_yy = 0.0, sum_xy = 0.0;
    for (auto& [p_x,p_y]  : filtered_points) {
        float dx = p_x - sample_mean_x;
        float dy = p_y - sample_mean_y;
        sum_xx += dx * dx;
        sum_yy += dy * dy;
        sum_xy += dx * dy;
    }
    float cov_xx = sum_xx / (N_f - 1);
    float cov_yy = sum_yy / (N_f - 1);
    float cov_xy = sum_xy / (N_f - 1);

    Eigen::Matrix2d covarianceMatrix;
    covarianceMatrix << cov_xx, cov_xy,
                        cov_xy, cov_yy;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covarianceMatrix);
    Eigen::Vector2d eigen_values = solver.eigenvalues().real();
    float max_eigen = eigen_values.maxCoeff();
    float min_eigen = eigen_values.minCoeff();

    if(max_eigen/min_eigen > 10e5){continue;} 
    else{
      float dist = std::sqrt(sample_mean_x * sample_mean_x + sample_mean_y * sample_mean_y);
      double theta = std::atan2(sample_mean_y, sample_mean_x);

      double normalizedAngle = std::fmod(theta, 2 * M_PI);
      if (normalizedAngle < 0) {
          normalizedAngle += 2 * M_PI;
      }

      int index = static_cast<int>(std::floor(normalizedAngle * rows / (2 * M_PI))) % rows;

      double a1 = azimuth_angles[index];
      double a2 = azimuth_angles[index+1];

      if(a1 > normalizedAngle){
        if(index >=1){
          index--;
          a1 = azimuth_angles[index];
          a2 = azimuth_angles[index+1];
        }
      }
      if(a2 < normalizedAngle){
        if(index < rows-1){
          index++;
          a1 = azimuth_angles[index];
          a2 = azimuth_angles[index+1];
        }
      }
      int64_t t1=0;
      int64_t t2=0;
      int64_t time=0;
      bool up_chirp = true;
      if(index<=0){
        time = azimuth_times[0];
        up_chirp = up_chirps[0];
      } else if(index >= rows-1){
        time = azimuth_times[rows-1];
        up_chirp = up_chirps[rows-1];
      } else{
        t1 = azimuth_times[index];
        t2 = azimuth_times[index+1];
        time = t1 + (t2-t1)*(normalizedAngle-a1)/(a2-a1);
        up_chirp = up_chirps[index];
      }

      PointT p;
      p.rho = dist;
      p.phi = normalizedAngle;
      p.theta = 0;
      p.timestamp = time;
      p.up_chirp = up_chirp;
      polar_time.push_back(p);

    }
    // #pragma omp critical
    {
      pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
    }
  }
}

template <class PointT>
void BFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 const std::vector<bool> &up_chirps,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;

  double static_threshold_watts = std::pow(10,(255.0*static_threshold_/20.0));
  double static_threshold_squared = static_threshold_watts*static_threshold_watts;
  if(static_threshold_ == 0.0){static_threshold_squared=0.0;}


  float conversion_factor = 255.0/20.0*std::log(10.0);
  cv::Mat raw_scan_watts_sqrd = raw_scan * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  cv::Mat point_overlay = cv::Mat::zeros(rows,cols,CV_32F);

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    const bool up_chirp = up_chirps[i];
    pcl::PointCloud<PointT> polar_time;
    float peak_points = 0;
    int num_peak_points = 0;

    double left = 0.0;
    double right = 0.0;

    for (int k = -w2 - guard_; k < -guard_; ++k){
        left += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));
      }

    for (int k = guard_ + 1; k <= w2 + guard_; ++k){
      right += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));
    }

    for (int j = mincol; j < maxcol; ++j) {
      // Intensity of test cell Y
      float intensity_Y = raw_scan_watts_sqrd.at<float>(i, j);

      left = left - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + -w2 - guard_ - 1)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j -guard_ - 1));
      right = right - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + guard_)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + w2 + guard_));

      // suggestion from CFEAR repo to improve runtime
      if(intensity_Y < static_threshold_squared){continue;}

      double mean = (left+right) / (2 * w2);

      float thres = threshold_ * mean + static_threshold_squared;

      if (intensity_Y > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho =res * peak_points /static_cast<float>(num_peak_points) + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        p.up_chirp = up_chirp;
        polar_time.push_back(p);
        peak_points = 0.0;
        num_peak_points = 0;
      }
    }
    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
}

template <class PointT>
void MSCA_CFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 const std::vector<bool> &up_chirps,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;

  cv::Mat raw_scan_watts_sqrd = raw_scan.clone();
  float conversion_factor = 255.0/20.0*std::log(10.0);
  raw_scan_watts_sqrd = raw_scan_watts_sqrd * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    const bool up_chirp = up_chirps[i];
    pcl::PointCloud<PointT> polar_time;
    float peak_points = 0;
    int num_peak_points = 0;

    for (int j = mincol; j < maxcol; ++j) {
      // Intensity of test cell Y
      float intensity_Y =raw_scan_watts_sqrd.at<float>(i, j);
      double left = 0;
      double right = 0;
      double mean = 0;

      for (int k = -w2 - guard_; k < -guard_ - M_; ++k){
        left += static_cast<double>(std::min(raw_scan_watts_sqrd.at<float>(i, j + k), raw_scan_watts_sqrd.at<float>(i, j + k + M_)));
      }

      for (int k = guard_ + 1; k <= w2 + guard_ - M_; ++k){
        right += static_cast<double>(std::min(raw_scan_watts_sqrd.at<float>(i, j + k), raw_scan_watts_sqrd.at<float>(i, j + k + M_)));
      }

      mean = (left+right) / (2 * (w2 - M_));

      const float thres = threshold_ * mean;

      if (intensity_Y > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        p.up_chirp = up_chirp;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    }
    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
}

template <class PointT>
void IS_CFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 const std::vector<bool> &up_chirps,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;

  cv::Mat raw_scan_watts_sqrd = raw_scan.clone();
  float conversion_factor = 255.0/20.0*std::log(10.0);
  raw_scan_watts_sqrd = raw_scan_watts_sqrd * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    const bool up_chirp = up_chirps[i];
    pcl::PointCloud<PointT> polar_time;

    float peak_points = 0;
    int num_peak_points = 0;

    for (int j = mincol; j < maxcol; ++j) {
      // Intensity of test cell Y
      float intensity_Y = raw_scan_watts_sqrd.at<float>(i, j);

      double left = 0;
      double right = 0;
      double left_0 = 0;
      double right_0 = 0;
      int n_a = 0;
      int n_b = 0;
      
      for (int k = -w2 - guard_; k < -guard_; ++k){
        float intensity = raw_scan_watts_sqrd.at<float>(i, j + k);
        left += intensity;
        if(intensity < intensity_Y*alpha_I_){
          left_0 += intensity;
          n_a++;
        }
      }

      for (int k = guard_ + 1; k <= w2 + guard_; ++k){
        float intensity = raw_scan_watts_sqrd.at<float>(i, j + k);
        right += intensity;
        if(intensity < intensity_Y*alpha_I_){
          right_0 += intensity;
          n_b++;
        }
      }

      float Th = 0;
      if(n_a <= N_TI_ && n_b <= N_TI_){
        Th = beta_I_ * (left+right / (2 * w2));
      } else if(n_a > N_TI_ && n_b > N_TI_){
        Th = beta_I_ * (left_0+right_0 / (n_a+n_b));
      } else if(n_a <= N_TI_ && n_b > N_TI_){
        Th = beta_I_ * (left / w2);
      } else{
        Th = beta_I_ * (right / w2);
      }


      if (intensity_Y > Th) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        p.up_chirp = up_chirp;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    } 

    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
}

template <class PointT>
void VI_CFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 const std::vector<bool> &up_chirps,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;

  float conversion_factor = 255.0/20.0*std::log(10.0);
  cv::Mat raw_scan_watts_sqrd = raw_scan * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    const bool up_chirp = up_chirps[i];
    pcl::PointCloud<PointT> polar_time;
    float peak_points = 0;
    int num_peak_points = 0;

    for (int j = mincol; j < maxcol; ++j) {
      bool variable_leading = false;
      bool variable_lagging = false;
      bool different_means = false;

      // Intensity of test cell Y with sqaure law
      float intensity_Y= raw_scan_watts_sqrd.at<float>(i, j);

      double left = 0.0;
      double right = 0.0;
      double left_vi_sqrd = 0.0;
      double right_vi_sqrd = 0.0;

      double VI_simplified_leading = 0.0;
      double VI_simplified_lagging = 0.0;
      double MR = 0.0;

      for (int k = -w2 - guard_; k < -guard_; ++k){
        double intensity = static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + k));
        left += intensity;
        left_vi_sqrd += std::pow(intensity,2);;
      }

      for (int k = guard_ + 1; k <= w2 + guard_; ++k){
        double intensity = static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + k));
        right += intensity;
        right_vi_sqrd += std::pow(intensity,2);
      }

      VI_simplified_leading = 2*w2*(right_vi_sqrd)/(right*right);
      VI_simplified_lagging = 2*w2*(left_vi_sqrd)/(left*left);
      
      MR = left/right;

      if(VI_simplified_leading > K_VI_){variable_leading = true;}
      if(VI_simplified_lagging > K_VI_){variable_lagging = true;}

      if(1.0/K_MR_ > MR || MR > K_MR_){different_means=true;}

      float threshold = 0.0;

      if(!variable_leading && !variable_lagging && !different_means){
        threshold = C_N_ * (left+right) / (2*w2);
      }
      else if(!variable_leading && !variable_lagging && different_means){
        threshold = C_N_ * std::max(left,right) / w2;
      }
      else if(variable_leading && !variable_lagging){
        threshold = C_N_ * right / w2;
      }
      else if(!variable_leading && variable_lagging){
        threshold = C_N_ * left / w2;
      }
      else if(variable_leading && variable_lagging){
        threshold = C_N_ * std::min(left,right) / w2;
      }

      if (intensity_Y > threshold) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        p.up_chirp = up_chirp;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    } 
    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
}

template <class PointT>
void Cen2019<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 const std::vector<bool> &up_chirps,
                                 pcl::PointCloud<PointT> &pointcloud) {

                                  
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;


  const cv::Mat raw_scan_dB = raw_scan.clone()*255.0/2.0;
  cv::Mat raw_scan_dB_cart;

  cv::Mat grad_x;
  cv::Mat filt_x = (cv::Mat_<float>(3,3) << 1, 1, 1, 0, 0, 0, -1, -1, -1);
  cv::filter2D(raw_scan_dB, grad_x, -1, filt_x, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);

  cv::Mat grad_y;
  cv::Mat filt_y = (cv::Mat_<float>(3,3) << 1, 0, -1, 1, 0, -1, 1, 0, -1);
  cv::filter2D(raw_scan_dB, grad_y, -1, filt_y, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);

  cv::Mat grad_mag;
  cv::pow(grad_x,2,grad_x);
  cv::pow(grad_y,2,grad_y);
  cv::sqrt(grad_x + grad_y, grad_mag);
  cv::normalize(grad_mag, grad_mag, 0, 1, cv::NORM_MINMAX);

  cv::Mat s_prime = raw_scan_dB - cv::mean(raw_scan_dB)[0];

  cv::Mat H = (1-grad_mag).mul(s_prime);
  
  // sortIdx can only sort by col or row so we must convert H to a single column matrix
  cv::Mat I = H.clone();
  I = I.reshape(1, rows * cols);
  cv::sortIdx(I, I, cv::SORT_EVERY_COLUMN  + cv::SORT_DESCENDING);

  int l = 0;
  int curr_idx = 0;
  int check_points_count = 0;
  int max_bounded_points = rows * (maxcol-mincol+1);

  // Treat 1 as False and 0 as True
  // This is slightly counter intuitive but allows the use of cv::hasNonZero 
  // This used to check if there are any "False" or unvisited cells left in R
  cv::Mat R = cv::Mat::ones(rows,cols,CV_32F);
  std::vector<std::tuple<int, int, int>> Q_regions;

  std::unordered_map<int, std::vector<std::pair<int,int>>> Q_Map;

  while(l < l_max_ && check_points_count <  max_bounded_points){
    // Convert linear index to (a,r)
    int raw_idx = I.at<int>(curr_idx);
    int a = raw_idx / cols;
    int r = raw_idx % cols;
    curr_idx++;

    if(r>maxcol || r<mincol){continue;}

    // where R(a,r) == False is the same as R(a,r) == 1
    if(R.at<float>(a,r) == 1.0){
      int r_low = r;
      int r_high = r;

      bool r_low_found = false;
      bool r_high_found = false;

      // Search s_prime along a for the closest range indices (rlow, rhigh) below and above r with values less than 0
      for(int k = r-1; k >= mincol; --k){
        if(s_prime.at<float>(a,k) < 0.0){
          r_low = k+1;
          r_low_found = true;
          break;
        }
      }
      if(!r_low_found){r_low = mincol;}

      for(int k = r+1; k < maxcol; ++k){
        if(s_prime.at<float>(a,k) < 0.0){
          r_high = k;
          r_high_found = true;
          break;
        }
      }
      if(!r_high_found){r_high = maxcol-1;}

      cv::Mat R_slice = cv::Mat(R, cv::Range(a,a+1), cv::Range(r_low,r_high+1));

      // if none in R[a, rlow : rhigh] then
      if(cv::countNonZero(R_slice) == r_high-r_low+1){
        Q_Map[a].push_back(std::make_pair(r_low, r_high));
        l++;
      }
      for(int k = r_low-1; k <= r_high; ++k){
        R.at<float>(a,k) = 0.0;
        check_points_count++;  
      }
    }
  }

  // For every azimuth
  for(auto& [a,ranges] : Q_Map){
    const double azimuth = azimuth_angles[a];
    const int64_t time = azimuth_times[a];
    const bool up_chirp = up_chirps[a];
    pcl::PointCloud<PointT> polar_time;

    // For every region in this azimuth
    for(auto& pair : ranges){
      int r_low = pair.first;
      int r_high = pair.second;

      cv::Mat Q_minus, Q_plus;
      if(a == 0){
        Q_minus = cv::Mat(R, cv::Range(rows-1,rows), cv::Range(r_low,r_high));
        Q_plus = cv::Mat(R, cv::Range(a+1,a+2), cv::Range(r_low,r_high));
      } else if(a == rows-1){
        Q_minus = cv::Mat(R, cv::Range(a-1,a), cv::Range(r_low,r_high));
        Q_plus = cv::Mat(R, cv::Range(0,1), cv::Range(r_low,r_high));
      } else {
        Q_minus = cv::Mat(R, cv::Range(a-1,a), cv::Range(r_low,r_high));
        Q_plus = cv::Mat(R, cv::Range(a+1,a+2), cv::Range(r_low,r_high));
      }

      int range = r_high - r_low;
      if(cv::countNonZero(Q_minus) < range || cv::countNonZero(Q_plus) < range){
        cv::Mat H_vals = cv::Mat(H, cv::Range(a,a+1), cv::Range(r_low,r_high));
        double maxVal;
        int maxIdx[2];
        cv::minMaxIdx(H_vals, 0, &maxVal, 0, maxIdx);
        int max_r_idx = maxIdx[1] + r_low;

        PointT p;
        p.rho = res * max_r_idx + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        p.up_chirp = up_chirp;
        polar_time.push_back(p);
      }
    }
    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
}

}  // namespace radar
}  // namespace vtr