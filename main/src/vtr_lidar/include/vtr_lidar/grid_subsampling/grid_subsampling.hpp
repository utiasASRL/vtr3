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
 * \file grid_subsampling.hpp
 * \brief Grid subsampling utilities
 *
 * \author Hugues Thomas, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_lidar/cloud/cloud.hpp>

namespace vtr {
namespace lidar {

class SampledData {
 public:
  // Elements
  // ********

  int count;
  PointXYZ point;
  std::vector<float> features;
  std::vector<std::unordered_map<int, int>> labels;

  // Methods
  // *******

  // Constructor
  SampledData() {
    count = 0;
    point = PointXYZ();
  }

  SampledData(const size_t fdim, const size_t ldim) {
    count = 0;
    point = PointXYZ();
    features = std::vector<float>(fdim);
    labels = std::vector<std::unordered_map<int, int>>(ldim);
  }

  // Method Update
  void update_all(const PointXYZ p, std::vector<float>::iterator f_begin,
                  std::vector<int>::iterator l_begin) {
    count += 1;
    point += p;
    transform(features.begin(), features.end(), f_begin, features.begin(),
              std::plus<float>());
    int i = 0;
    for (std::vector<int>::iterator it = l_begin; it != l_begin + labels.size();
         ++it) {
      labels[i][*it] += 1;
      i++;
    }
    return;
  }
  void update_features(const PointXYZ p, std::vector<float>::iterator f_begin) {
    count += 1;
    point += p;
    transform(features.begin(), features.end(), f_begin, features.begin(),
              std::plus<float>());
    return;
  }
  void update_classes(const PointXYZ p, std::vector<int>::iterator l_begin) {
    count += 1;
    point += p;
    int i = 0;
    for (std::vector<int>::iterator it = l_begin; it != l_begin + labels.size();
         ++it) {
      labels[i][*it] += 1;
      i++;
    }
    return;
  }
  void update_points(const PointXYZ p) {
    count += 1;
    point += p;
    return;
  }
};

class SampledPts {
 public:
  // Elements
  // ********

  int count;
  PointXYZ point;

  // Methods
  // *******

  // Constructor
  SampledPts() {
    count = 0;
    point = PointXYZ();
  }
  SampledPts(const PointXYZ& p0) {
    count = 1;
    point = p0;
  }

  void update_points(const PointXYZ p) {
    count += 1;
    point += p;
    return;
  }
};

class SampledCenter {
 public:
  // Elements
  // ********

  size_t idx;
  PointXYZ center;
  float d2;

  // Methods
  // *******

  // Constructor
  SampledCenter() {
    idx = 0;
    center = PointXYZ();
    d2 = 0;
  }
  SampledCenter(size_t idx0, const PointXYZ& p0, const PointXYZ& center0) {
    idx = idx0;
    center = center0;
    d2 = (p0 - center0).sq_norm();
  }

  void update_points(size_t idx0, const PointXYZ& p0) {
    float new_d2 = (p0 - center).sq_norm();
    if (new_d2 < d2) {
      d2 = new_d2;
      idx = idx0;
    }
    return;
  }
};

void gridSubsamplingCenters(const std::vector<PointXYZ>& original_points,
                            const float sampleDl,
                            std::vector<PointXYZ>& subsampled_points,
                            std::vector<size_t>& subsampled_inds);

void grid_subsampling_spheres(std::vector<PointXYZ>& original_points,
                              std::vector<PointXYZ>& subsampled_points,
                              float sampleDl);

void grid_subsampling(std::vector<PointXYZ>& original_points,
                      std::vector<PointXYZ>& subsampled_points,
                      std::vector<float>& original_features,
                      std::vector<float>& subsampled_features,
                      std::vector<int>& original_classes,
                      std::vector<int>& subsampled_classes, float sampleDl,
                      int verbose);

void batch_grid_subsampling(std::vector<PointXYZ>& original_points,
                            std::vector<PointXYZ>& subsampled_points,
                            std::vector<float>& original_features,
                            std::vector<float>& subsampled_features,
                            std::vector<int>& original_classes,
                            std::vector<int>& subsampled_classes,
                            std::vector<int>& original_batches,
                            std::vector<int>& subsampled_batches,
                            float sampleDl, int max_p);

}  // namespace lidar
}  // namespace vtr