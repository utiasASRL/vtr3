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

#include "vtr_radar/data_types/point.hpp"
#include "vtr_radar/utils/utils.hpp"

namespace vtr {
namespace radar {

template <class PointT>
class Detector {
 public:
  virtual ~Detector() = default;

  virtual void run(const cv::Mat &raw_scan, const float &res,
                   const std::vector<int64_t> &azimuth_times,
                   const std::vector<double> &azimuth_angles,
                   pcl::PointCloud<PointT> &pointcloud) = 0;
};

template <class PointT>
class KStrongest : public Detector<PointT> {
 public:
  KStrongest() = default;
  KStrongest(int kstrong, double threshold2, double threshold3, double minr,
             double maxr, double range_offset)
      : kstrong_(kstrong),
        threshold2_(threshold2),
        threshold3_(threshold3),
        minr_(minr),
        maxr_(maxr),
        range_offset_(range_offset) {}

  void run(const cv::Mat &raw_scan, const float &res,
           const std::vector<int64_t> &azimuth_times,
           const std::vector<double> &azimuth_angles,
           pcl::PointCloud<PointT> &pointcloud) override;

 private:
  int kstrong_ = 10;
  double threshold2_ = 1.5;
  double threshold3_ = 0.22;
  double minr_ = 2.0;
  double maxr_ = 100.0;
  double range_offset_ = -0.31;
};

template <class PointT>
class Cen2018 : public Detector<PointT> {
 public:
  Cen2018() = default;
  Cen2018(double zq, int sigma, double minr, double maxr, double range_offset)
      : zq_(zq),
        sigma_(sigma),
        minr_(minr),
        maxr_(maxr),
        range_offset_(range_offset) {}

  void run(const cv::Mat &raw_scan, const float &res,
           const std::vector<int64_t> &azimuth_times,
           const std::vector<double> &azimuth_angles,
           pcl::PointCloud<PointT> &pointcloud) override;

 private:
  double zq_ = 3.0;
  int sigma_ = 17;  // kernel size = sigma_ * 2 * 3 (+1 to make it odd)
  double minr_ = 2.0;
  double maxr_ = 100.0;
  double range_offset_ = -0.31;
};

template <class PointT>
class CACFAR : public Detector<PointT> {
 public:
  CACFAR() = default;
  CACFAR(int width, int guard, double threshold, double threshold2,
         double threshold3, double minr, double maxr, double range_offset
      : width_(width),
        guard_(guard),
        threshold_(threshold),
        threshold2_(threshold2),
        threshold3_(threshold3),
        minr_(minr),
        maxr_(maxr),
        range_offset_(range_offset) {}

  void run(const cv::Mat &raw_scan, const float &res,
           const std::vector<int64_t> &azimuth_times,
           const std::vector<double> &azimuth_angles,
           pcl::PointCloud<PointT> &pointcloud) override;

 private:
  int width_ = 41;  // window = width + 2 * guard
  int guard_ = 2;
  double threshold_ = 3.0;
  double threshold2_ = 1.1;
  double threshold3_ = 0.22;
  double minr_ = 2.0;
  double maxr_ = 100.0;
  double range_offset_ = -0.31;
};

template <class PointT>
class OSCFAR : public Detector<PointT> {
 public:
  OSCFAR() = default;
  OSCFAR(int width, int guard, int kstat, double threshold, double threshold2,
         double threshold3, double minr, double maxr, double range_offset)
      : width_(width),
        guard_(guard),
        kstat_(kstat),
        threshold_(threshold),
        threshold2_(threshold2),
        threshold3_(threshold3),
        minr_(minr),
        maxr_(maxr),
        range_offset_(range_offset) {}

  void run(const cv::Mat &raw_scan, const float &res,
           const std::vector<int64_t> &azimuth_times,
           const std::vector<double> &azimuth_angles,
           pcl::PointCloud<PointT> &pointcloud) override;

 private:
  int width_ = 40;
  int guard_ = 2;
  int kstat_ = 20;
  double threshold_ = 1.25;
  double threshold2_ = 1.2;
  double threshold3_ = 0.22;
  double minr_ = 2.0;
  double maxr_ = 100.0;
  double range_offset_ = -0.31;
};

template <class PointT>
class ModifiedCACFAR : public Detector<PointT> {
 public:
  ModifiedCACFAR() = default;
  ModifiedCACFAR(int width, int guard, double threshold, double threshold2,
                 double threshold3, double minr, double maxr,
                 double range_offset)
      : width_(width),
        guard_(guard),
        threshold_(threshold),
        threshold2_(threshold2),
        threshold3_(threshold3),
        minr_(minr),
        maxr_(maxr),
        range_offset_(range_offset) {}

  void run(const cv::Mat &raw_scan, const float &res,
           const std::vector<int64_t> &azimuth_times,
           const std::vector<double> &azimuth_angles,
           pcl::PointCloud<PointT> &pointcloud) override;

 private:
  int width_ = 41;  // window = width + 2 * guard
  int guard_ = 2;
  double threshold_ = 3.0;
  double threshold2_ = 1.1;
  double threshold3_ = 0.22;
  double minr_ = 2.0;
  double maxr_ = 100.0;
  double range_offset_ = -0.31;
};

}  // namespace radar
}  // namespace vtr

#include "vtr_radar/detector/detector.inl"