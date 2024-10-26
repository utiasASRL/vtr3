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

namespace vtr {
namespace radar {

namespace {
bool sort_desc_by_first(const std::pair<int, float> &a,
                        const std::pair<int, float> &b) {
  return (a.first > b.first);
}
bool sort_asc_by_second(const std::pair<int, float> &a,
                        const std::pair<int, float> &b) {
  return (a.second < b.second);
}

}  // namespace

template <class PointT>
void KStrongest<PointT>::run(const cv::Mat &raw_scan, const float &res,
                             const std::vector<int64_t> &azimuth_times,
                             const std::vector<double> &azimuth_angles,
                             pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  auto mincol = minr_ / res;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const auto N = maxcol - mincol;

  // #pragma omp parallel for
  for (int i = 0; i < rows; ++i) {
    std::vector<std::pair<float, int>> intens;
    intens.reserve(N / 2);
    double mean = 0;
    for (int j = mincol; j < maxcol; ++j) {
      mean += raw_scan.at<float>(i, j);
    }
    mean /= N;
    const double thres = mean * threshold2_ + threshold3_;
    for (int j = mincol; j < maxcol; ++j) {
      if (raw_scan.at<float>(i, j) >= thres)
        intens.emplace_back(raw_scan.at<float>(i, j), j);
    }
    // sort intensities in descending order
    std::sort(intens.begin(), intens.end(), sort_desc_by_first);
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    for (int j = 0; j < kstrong_; ++j) {
      if (intens[j].first < thres) break;
      PointT p;
      p.rho = float(intens[j].second) * res + range_offset_;
      p.phi = azimuth;
      p.theta = 0;
      p.timestamp = time;
      polar_time.push_back(p);
    }
    // #pragma omp critical
    {
      pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
    }
  }
}

template <class PointT>
void Cen2018<PointT>::run(const cv::Mat &raw_scan, const float &res,
                          const std::vector<int64_t> &azimuth_times,
                          const std::vector<double> &azimuth_angles,
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
  // TODO: try implementing an efficient median filter
  // Estimate the bias and subtract it from the signal
  cv::Mat q = raw_scan.clone();
  for (int i = 0; i < rows; ++i) {
    float mean = 0;
    for (int j = mincol; j < maxcol; ++j) {
      mean += raw_scan.at<float>(i, j);
    }
    mean /= N;
    for (int j = mincol; j < maxcol; ++j) {
      q.at<float>(i, j) = raw_scan.at<float>(i, j) - mean;
    }
  }

  // Create 1D Gaussian Filter
  // TODO: binomial filter may be more efficient
  int fsize = sigma_ * 2 * 3;
  if (fsize % 2 == 0) fsize += 1;
  const int mu = fsize / 2;
  const float sig_sqr = sigma_ * sigma_;
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
  for (int i = 0; i < rows; ++i) {
    int nonzero = 0;
    for (int j = mincol; j < maxcol; ++j) {
      float n = q.at<float>(i, j);
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
        p.rho = res * peak_points / num_peak_points + range_offset_;
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    }
    if (num_peak_points > 0) {
      const double azimuth = azimuth_angles[rows - 1];
      const int64_t time = azimuth_times[rows - 1];
      PointT p;
      p.rho = res * peak_points / num_peak_points + range_offset_;
      p.phi = azimuth;
      p.theta = 0;
      p.timestamp = time;
      polar_time.push_back(p);
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
                         pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  const int window = width_ + guard_ * 2;
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const int N = maxcol - mincol;

  // #pragma omp parallel for
  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    double mean = 0;
    for (int j = mincol; j < maxcol; ++j) {
      mean += raw_scan.at<float>(i, j);
    }
    mean /= N;

    for (int j = mincol; j < maxcol; ++j) {
      double stat = 0;  // (statistic) estimate of clutter power
      double left = 0;
      double right = 0;
      // for (int k = -w2 - guard_; k <= w2 + guard_; ++k) {
      //   if (k < -guard_ || k > guard_)
      //     stat += raw_scan.at<float>(i, j + k);
      // }
      for (int k = -w2 - guard_; k < -guard_; ++k) {
        left += raw_scan.at<float>(i, j + k);
      }
      for (int k = guard_ + 1; k <= w2 + guard_; ++k) {
        right += raw_scan.at<float>(i, j + k);
      }
      stat = std::max(left, right);
      const float thres =
          threshold_ * stat / (window / 2) + threshold2_ * mean + threshold3_;
      if (raw_scan.at<float>(i, j) > thres) {
        PointT p;
        p.rho = j * res + range_offset_;
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
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
  const int N = maxcol - mincol;

  // #pragma omp parallel for
  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    double mean = 0;
    for (int j = mincol; j < maxcol; ++j) {
      mean += raw_scan.at<float>(i, j);
    }
    mean /= N;

    std::vector<std::pair<int, float>> window;
    window.reserve(width_ - 1);
    int j = mincol - 1;
    for (int k = -w2; k < 0; ++k) {
      window.emplace_back(j + k, raw_scan.at<float>(i, j + k));
    }
    for (int k = 1; k <= w2; ++k) {
      window.emplace_back(j + k, raw_scan.at<float>(i, j + k));
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
      auto prevcut = std::make_pair(j - 1, raw_scan.at<float>(i, j - 1));
      auto it = std::lower_bound(window.begin(), window.end(), prevcut,
                                 sort_asc_by_second);
      window.insert(it, prevcut);
      auto newentry = std::make_pair(j + w2, raw_scan.at<float>(i, j + w2));
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
      const float thres = threshold_ * stat + threshold2_ * mean + threshold3_;
      if (raw_scan.at<float>(i, j) > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + range_offset_;
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
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
void ModifiedCACFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
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

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    double mean = 0;
    for (int j = mincol; j < maxcol; ++j) {
      mean += raw_scan.at<float>(i, j);
    }
    mean /= N;

    float peak_points = 0;
    int num_peak_points = 0;

    for (int j = mincol; j < maxcol; ++j) {
      double left = 0;
      double right = 0;
      for (int k = -w2 - guard_; k < -guard_; ++k)
        left += raw_scan.at<float>(i, j + k);
      for (int k = guard_ + 1; k <= w2 + guard_; ++k)
        right += raw_scan.at<float>(i, j + k);
      // (statistic) estimate of clutter power
      // const double stat = (left + right) / (2 * w2);
      const double stat = std::max(left, right) / w2;  // GO-CFAR  // use max min or average
      const float thres = threshold_ * stat + threshold2_ * mean + threshold3_;
      if (raw_scan.at<float>(i, j) > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + range_offset_;
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
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

  // // debug
  // std::cout<< "width:" << width_<<std::endl;
  // std::cout<< "Guard:" << guard_<<std::endl;
  // std::cout<< "Theshold" << threshold_<<std::endl;
  // std::cout<< "minr" << minr_<<std::endl;
  // std::cout<< "maxr" << maxr_<<std::endl;
  
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
    pcl::PointCloud<PointT> polar_time;
    double mean = 0;
    float peak_points = 0;
    int num_peak_points = 0;

    double left = 0;
    double right = 0;
    for (int k = -w2 - guard_; k < -guard_; ++k){left += raw_scan_watts_sqrd.at<float>(i, mincol-1 + k);}
    for (int k = guard_ + 1; k <= w2 + guard_; ++k){right += raw_scan_watts_sqrd.at<float>(i, mincol-1 + k);}

    for (int j = mincol; j < maxcol; ++j) {
      // Intensity of test cell Y with sqaure law
      double intensity_Y=raw_scan_watts_sqrd.at<float>(i, j);

      left = left - raw_scan_watts_sqrd.at<float>(i, j + -w2 - guard_ - 1) + raw_scan_watts_sqrd.at<float>(i, j -guard_ - 1);
      right = right - raw_scan_watts_sqrd.at<float>(i, j + guard_) + raw_scan_watts_sqrd.at<float>(i, j + w2 + guard_);

      mean = std::min(left, right) / w2;

      const double thres = threshold_ * mean;

      if (intensity_Y > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + range_offset_;
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    } 

    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }

}

}  // namespace radar
}  // namespace vtr