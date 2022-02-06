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
 * \file detector.hpp
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 * \brief Keypoint extraction methods for Navtech radar
 */

#pragma once

#include "opencv2/opencv.hpp"

#include "vtr_radar/types.hpp"

namespace vtr {
namespace radar {

template <class PointT>
class Detector {
 public:
  virtual ~Detector() = default;

  virtual void run(const cv::Mat &raw_scan, const float &res,
                   const std::vector<double> &azimuth_times,
                   const std::vector<double> &azimuth_angles,
                   pcl::PointCloud<PointT> &pointcloud) = 0;
};

template <class PointT>
class KStrongest : public Detector<PointT> {
 public:
  KStrongest() = default;
  KStrongest(int kstrong, double threshold, double minr, double maxr)
      : kstrong_(kstrong), threshold_(threshold), minr_(minr), maxr_(maxr) {}

  void run(const cv::Mat &raw_scan, const float &res,
           const std::vector<double> &azimuth_times,
           const std::vector<double> &azimuth_angles,
           pcl::PointCloud<PointT> &pointcloud) override;

 private:
  int kstrong_ = 10;
  double threshold_ = 1.5;
  double minr_ = 2.0;
  double maxr_ = 100.0;
};

template <class PointT>
class Cen2018 : public Detector<PointT> {
 public:
  Cen2018() = default;
  Cen2018(double zq, int sigma, double minr, double maxr)
      : zq_(zq), sigma_(sigma), minr_(minr), maxr_(maxr) {}

  void run(const cv::Mat &raw_scan, const float &res,
           const std::vector<double> &azimuth_times,
           const std::vector<double> &azimuth_angles,
           pcl::PointCloud<PointT> &pointcloud) override;

 private:
  double zq_ = 3.0;
  int sigma_ = 17;
  double minr_ = 2.0;
  double maxr_ = 100.0;
};

template <class PointT>
class CACFAR : public Detector<PointT> {
 public:
  CACFAR() = default;
  CACFAR(int width, int guard, double threshold, double minr, double maxr)
      : width_(width),
        guard_(guard),
        threshold_(threshold),
        minr_(minr),
        maxr_(maxr) {}

  void run(const cv::Mat &raw_scan, const float &res,
           const std::vector<double> &azimuth_times,
           const std::vector<double> &azimuth_angles,
           pcl::PointCloud<PointT> &pointcloud) override;

 private:
  int width_ = 40;
  int guard_ = 2;
  double threshold_ = 3.0;
  double minr_ = 2.0;
  double maxr_ = 100.0;
};

template <class PointT>
class OSCFAR : public Detector<PointT> {
 public:
  OSCFAR() = default;
  OSCFAR(int width, int guard, int kstat, double threshold, double minr,
         double maxr)
      : width_(width),
        guard_(guard),
        kstat_(kstat),
        threshold_(threshold),
        minr_(minr),
        maxr_(maxr) {}

  void run(const cv::Mat &raw_scan, const float &res,
           const std::vector<double> &azimuth_times,
           const std::vector<double> &azimuth_angles,
           pcl::PointCloud<PointT> &pointcloud) override;

 private:
  int width_ = 40;
  int guard_ = 2;
  int kstat_ = 20;
  double threshold_ = 1.25;
  double minr_ = 2.0;
  double maxr_ = 100.0;
};

}  // namespace radar
}  // namespace vtr