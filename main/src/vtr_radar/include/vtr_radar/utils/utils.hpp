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
 * \file utils.hpp
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <opencv2/opencv.hpp>

#include "lgmath.hpp"

#include "vtr_common/utils/hash.hpp"
#include "vtr_radar/data_types/point.hpp"
#include "vtr_radar/utils/nanoflann.hpp"

namespace vtr {
namespace radar {

template <class PointT>
struct NanoFLANNAdapter {
  NanoFLANNAdapter(const pcl::PointCloud<PointT> &points) : points_(points) {}

  const pcl::PointCloud<PointT> &points_;

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return points_.size(); }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate
  // value, the "if/else's" are actually solved at compile time.
  inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0)
      return points_[idx].x;
    else if (dim == 1)
      return points_[idx].y;
    else
      return points_[idx].z;
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in
  //   "bb" so it can be avoided to redo it again. Look at bb.size() to find out
  //   the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX & /* bb */) const {
    return false;
  }
};

template <>
struct NanoFLANNAdapter<pcl::PointXY> {
  NanoFLANNAdapter(const pcl::PointCloud<pcl::PointXY> &points)
      : points_(points) {}

  const pcl::PointCloud<pcl::PointXY> &points_;

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return points_.size(); }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate
  // value, the "if/else's" are actually solved at compile time.
  inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0)
      return points_[idx].x;
    else if (dim == 1)
      return points_[idx].y;
    else
      throw std::invalid_argument("Invalid dim");
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in
  //   "bb" so it can be avoided to redo it again. Look at bb.size() to find out
  //   the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX & /* bb */) const {
    return false;
  }
};

// KDTree type definition
using KDTreeParams = nanoflann::KDTreeSingleIndexAdaptorParams;
using KDTreeSearchParams = nanoflann::SearchParams;
using KDTreeResultSet = nanoflann::KNNResultSet<float>;
template <class PointT>
using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, NanoFLANNAdapter<PointT>>,
    NanoFLANNAdapter<PointT>>;
template <class PointT>
using DynamicKDTree = nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<float, NanoFLANNAdapter<PointT>>,
    NanoFLANNAdapter<PointT>>;

template <class PointT>
void pol2Cart2D(pcl::PointCloud<PointT> &pointcloud) {
  for (auto &point : pointcloud) {
    point.x = point.rho * std::cos(point.phi);
    point.y = point.rho * std::sin(point.phi);
    point.z = 0.0;
  }
}

template <class PointT>
void cart2pol(pcl::PointCloud<PointT> &point_cloud) {
  for (auto &p : point_cloud) {
    p.rho = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    p.theta = atan2(sqrt(p.x * p.x + p.y * p.y), p.z);
    p.phi = atan2(p.y, p.x) + M_PI / 2;
  }
}

void load_radar(const std::string &path, std::vector<double> &timestamps,
                std::vector<double> &azimuths, cv::Mat &fft_data);

void load_radar(const cv::Mat &raw_data, std::vector<double> &timestamps,
                std::vector<double> &azimuths, cv::Mat &fft_data);

/** \brief Returns the cartesian image of a radar scan */
// clang-format off
void radar_polar_to_cartesian(const cv::Mat &fft_data,
                              const std::vector<double> &azimuths,
                              cv::Mat &cartesian,
                              const float radar_resolution,
                              const float cart_resolution,
                              const int cart_pixel_width,
                              const bool interpolate_crossover,
                              const int output_type = CV_8UC1);
// clang-format on

// vbar is 3x1 v_s_i_in_s where (s) is sensor, (i) is a static frame
// like an inertial frame or (pm) frame in this repository
// beta \approx f_transmission / (slope of modulation pattern dfdt)
// correct radial distance using Doppler factor
// relative velocity (> 0: point/sensor moving towards each other)
template <class PointT>
void removeDoppler(pcl::PointCloud<PointT>& point_cloud,
  const Eigen::VectorXd &vbar, const double beta) {
  for (auto& p : point_cloud) {
    Eigen::Vector3d abar = {p.x, p.y, p.z};
    abar.normalize();
    const double delta_rho = beta * abar.transpose() * vbar;
    p.x += delta_rho * abar(0);
    p.y += delta_rho * abar(1);
    p.z += delta_rho * abar(2);
  }
}

}  // namespace radar
}  // namespace vtr