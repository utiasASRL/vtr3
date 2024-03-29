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
 * \file himmelsbach.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "pcl/point_cloud.h"

#include "vtr_logging/logging.hpp"

namespace vtr {
namespace lidar {

/**
 * \brief Himmelsbach's algorithm for ground extraction.
 * \note this algorithms assumes that close to the sensor origin must be ground,
 * since the slope of the first fitted line determines slope of subsequent
 * fitted lines due to its smmooth transition requirement. We partially address
 * this by ignoring the smoothness constraints if the slope is too large.
 */
template <class PointT>
class Himmelsbach {
 public:
  using PointCloud = pcl::PointCloud<PointT>;
  /** \brief Returns ground point indices computed from Himmelsbach algorithm */
  std::vector<size_t> operator()(const PointCloud& points) const;

 private:
  struct Line {
   public:
    Line(const PointCloud& points, const std::vector<size_t>& line_pts,
         const float& m0, const float& b0)
        : m(m0), b(b0) {
      const auto ps = points[line_pts.front()];
      xs = std::sqrt(ps.x * ps.x + ps.y * ps.y);
      const auto pe = points[line_pts.back()];
      xe = std::sqrt(pe.x * pe.x + pe.y * pe.y);
    }
    float m;
    float b;
    float xs;
    float xe;
  };

 private:
  /** \brief */
  std::vector<std::vector<size_t>> sortPointsSegments(
      const PointCloud& points) const;
  /** \brief */
  std::vector<int> sortPointsBins(const PointCloud& points,
                                  const std::vector<size_t>& segment) const;
  /** \brief */
  using FitLineRval = std::tuple<float, float, float>; /* [m, b, rmse] */
  FitLineRval fitLine(const PointCloud& points,
                      const std::vector<size_t>& line_set) const;
  /** \brief */
  float distPointLine(const PointT& point, const Line& line) const;

 public:
  float z_offset = 2.13f;

  float alpha = 2.0 * M_PI / 180.0;
  float tolerance = 0.25;
  float Tm = 0.4;
  float Tm_small = 0.2;
  float Tb = 0.8;
  float Trmse = 0.1;
  float Tdprev = 1.0;

  float rmin = 3.0;
  size_t num_bins_small = 30;
  float bin_size_small = 3.0;
  size_t num_bins_large = 30;
  float bin_size_large = 3.0;
};

template <class PointT>
std::vector<size_t> Himmelsbach<PointT>::operator()(
    const PointCloud& points) const {
  std::vector<size_t> ground_idx;
  // sort points into segments
  const auto segments = sortPointsSegments(points);
  // Sort points into bins
  for (const auto& segment : segments) {
    if (segment.empty()) continue;
    //
    const auto bins = sortPointsBins(points, segment);
    //
    std::vector<Line> lines;       // extracted lines
    std::vector<size_t> line_set;  // current set of point index forming a line
    size_t i = 0;                  // current bin index
    while (i < bins.size() - 1) {
      const auto& idx = bins[i];
      if (idx < 0) {
        ++i;
        continue;
      } else if (line_set.size() >= 2) {
        std::vector<size_t> tmp(line_set);
        tmp.emplace_back(idx);
        const auto [m, b, rmse] = fitLine(points, tmp);
        if (std::abs(m) <= Tm &&
            (std::abs(m) > Tm_small || std::abs(b + z_offset) <= Tb) &&
            rmse <= Trmse) {
          line_set.emplace_back(idx);
          ++i;
        } else {
          const auto [m, b, rmse] = fitLine(points, line_set);
          lines.emplace_back(points, line_set, m, b);
          line_set.clear();
        }
      } else {
        // this mprev condition prevents initialization issues, especially when
        // first first two representative points are not both on the ground
        const auto mprev = lines.empty() ? 0 : std::abs(lines.back().m);
        const auto dprev =
            lines.empty() ? -1 : distPointLine(points[idx], lines.back());
        if (mprev > Tm || dprev <= Tdprev || lines.empty() || !line_set.empty())
          line_set.emplace_back(idx);
        ++i;
      }
    }
    ///
    if (line_set.size() >= 2) {
      const auto [m, b, rmse] = fitLine(points, line_set);
      lines.emplace_back(points, line_set, m, b);
      line_set.clear();
    }
    // assign points as inliers if they are within a threshold of the ground
    // model
    for (const auto& idx : segment) {
      const auto& point = points[idx];
      const auto& r = std::sqrt(point.x * point.x + point.y * point.y);
      // get line that's closest to the candidate point based on distance to
      // endpoints
      int closest = -1;
      float dmin = std::numeric_limits<float>::max();
      for (size_t i = 0; i < lines.size(); ++i) {
        const auto& line = lines[i];
        const auto ds = std::abs(line.xs - r);
        const auto de = std::abs(line.xe - r);
        const auto d = std::min(ds, de);
        if (d < dmin && std::abs(line.m) < Tm) {
          dmin = d;
          closest = i;
        }
      }
      if (closest >= 0) {
        const auto e = distPointLine(point, lines[closest]);
        if (e < tolerance) ground_idx.emplace_back(idx);
      }
    }
  }
  return ground_idx;
}

template <class PointT>
std::vector<std::vector<size_t>> Himmelsbach<PointT>::sortPointsSegments(
    const PointCloud& points) const {
  std::vector<std::vector<size_t>> segments;
  segments.resize(static_cast<size_t>(std::ceil(2 * M_PI / alpha)));
  for (size_t i = 0; i < points.size(); ++i) {
    const auto& point = points[i];
    auto angle = std::atan2(point.y, point.x);
    if (angle < 0) angle += 2 * M_PI;
    size_t idx = std::floor(angle / alpha);
    segments[idx].push_back(i);
  }
  return segments;
}

template <class PointT>
std::vector<int> Himmelsbach<PointT>::sortPointsBins(
    const PointCloud& points, const std::vector<size_t>& segment) const {
  const auto num_bins = num_bins_small + num_bins_large;
  const auto rsmall = rmin + bin_size_small * num_bins_small;
  const auto rlarge = rsmall + bin_size_large * num_bins_large;
  //
  std::vector<std::vector<size_t>> bins;
  bins.resize(num_bins);
  for (const auto& idx : segment) {
    const auto& point = points[idx];
    const auto& r = std::sqrt(point.x * point.x + point.y * point.y);
    int bin = -1;
    if (rmin <= r && r < rsmall)
      bin = (r - rmin) / bin_size_small;
    else if (rsmall <= r && r < rlarge)
      bin = num_bins_small + (r - rsmall) / bin_size_large;
    //
    if (bin >= 0) bins[(size_t)bin].push_back(idx);
  }
  // The point with the lowest z-coordinate in each bin becomes the
  // representative point
  std::vector<int> bins_out;
  bins_out.resize(num_bins, -1);
  for (size_t i = 0; i < num_bins; ++i) {
    const auto& bin_pts = bins[i];
    float zmin = std::numeric_limits<float>::max();
    int lowest = -1;
    for (const auto& idx : bin_pts) {
      const auto& point = points[idx];
      if (point.z < zmin) {
        zmin = point.z;
        lowest = idx;
      }
    }
    bins_out[i] = lowest;
  }

  return bins_out;
}

template <class PointT>
auto Himmelsbach<PointT>::fitLine(const PointCloud& points,
                                  const std::vector<size_t>& line_set) const
    -> FitLineRval {
  Eigen::MatrixXf A(line_set.size(), 2);
  Eigen::VectorXf b(line_set.size());
  for (size_t i = 0; i < line_set.size(); ++i) {
    const auto& point = points[line_set[i]];
    A(i, 0) = std::sqrt(point.x * point.x + point.y * point.y);
    A(i, 1) = 1;
    b(i) = point.z;
  }
  const auto soln = (A.transpose() * A).ldlt().solve(A.transpose() * b).eval();
  const auto err = A * soln - b;
  const auto mse = ((err.transpose() * err) / (float)line_set.size()).value();
  const auto rmse = std::sqrt(mse);
  return std::make_tuple(soln(0), soln(1), rmse);
}

template <class PointT>
float Himmelsbach<PointT>::distPointLine(const PointT& point,
                                         const Line& line) const {
  const auto r = std::sqrt(point.x * point.x + point.y * point.y);
  const auto line_z = line.m * r + line.b;
  return std::abs(line_z - point.z) / std::sqrt(line.m * line.m + 1);
}

}  // namespace lidar
}  // namespace vtr