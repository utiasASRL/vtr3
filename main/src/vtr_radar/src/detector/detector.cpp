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
 * \file detector.cpp
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 * \brief Keypoint extraction methods for Navtech radar
 */
#include "vtr_radar/detector/detector.hpp"

namespace vtr {
namespace radar {

namespace {
bool sortbysec(const std::pair<int, double> &a,
               const std::pair<int, double> &b) {
  return (a.second > b.second);
}
}  // namespace

template <class PointT>
void KStrongest<PointT>::run(const cv::Mat &raw_scan, const float &res,
                             const std::vector<double> &azimuth_times,
                             const std::vector<double> &azimuth_angles,
                             pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();

  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  auto mincol = minr_ * res;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ * res;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const auto N = maxcol - mincol;

#pragma omp parallel for
  for (int i = 0; i < rows; ++i) {
    std::vector<std::pair<int, double>> intens(N, 0);
    double mean = 0;
    for (int j = mincol; j < maxcol; ++j) {
      intens[j - mincol] = std::make_pair(j, raw_scan.at<float>(i, j));
      mean += raw_scan.at<float>(i, j);
    }
    mean /= N;

    // sort intensities in descending order
    std::sort(intens.begin(), intens.end(), sortbysec);
    const double azimuth = azimuth_angles[i];
    const double time = azimuth_times[i];
    const double thres = mean * threshold_;
    pcl::PointCloud<PointT> polar_time;
    for (int j = 0; j < kstrong_; ++j) {
      if (intens[j].second < thres) break;
      const uint rad = intens[j].first * res;
      PointT p;
      p.rho = rad;
      p.phi = azimuth;
      p.theta = 0;
      p.time = time;
      polar_time.push_back(p);
    }
#pragma omp critical
    {
      pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
    }
  }
}

template <class PointT>
void Cen2018<PointT>::run(const cv::Mat &raw_scan, const float &res,
                          const std::vector<double> &azimuth_times,
                          const std::vector<double> &azimuth_angles,
                          pcl::PointCloud<PointT> &pointcloud) {}  // TODO

template <class PointT>
void CACFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                         const std::vector<double> &azimuth_times,
                         const std::vector<double> &azimuth_angles,
                         pcl::PointCloud<PointT> &pointcloud) {}  // TODO

template <class PointT>
void OSCFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                         const std::vector<double> &azimuth_times,
                         const std::vector<double> &azimuth_angles,
                         pcl::PointCloud<PointT> &pointcloud) {}  // TODO

}  // namespace radar
}  // namespace vtr