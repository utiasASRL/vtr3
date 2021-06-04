#pragma once

#include "vtr_lidar/cloud/cloud.h"
#include "vtr_lidar/pointmap/pointmap.h"

using namespace std;

class SampledData {
 public:
  // Elements
  // ********

  int count;
  PointXYZ point;
  vector<float> features;
  vector<unordered_map<int, int>> labels;

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
    features = vector<float>(fdim);
    labels = vector<unordered_map<int, int>>(ldim);
  }

  // Method Update
  void update_all(const PointXYZ p, vector<float>::iterator f_begin,
                  vector<int>::iterator l_begin) {
    count += 1;
    point += p;
    transform(features.begin(), features.end(), f_begin, features.begin(),
              plus<float>());
    int i = 0;
    for (vector<int>::iterator it = l_begin; it != l_begin + labels.size();
         ++it) {
      labels[i][*it] += 1;
      i++;
    }
    return;
  }
  void update_features(const PointXYZ p, vector<float>::iterator f_begin) {
    count += 1;
    point += p;
    transform(features.begin(), features.end(), f_begin, features.begin(),
              plus<float>());
    return;
  }
  void update_classes(const PointXYZ p, vector<int>::iterator l_begin) {
    count += 1;
    point += p;
    int i = 0;
    for (vector<int>::iterator it = l_begin; it != l_begin + labels.size();
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

void grid_subsampling_centers(vector<PointXYZ>& original_points,
                              vector<PointXYZ>& subsampled_points,
                              vector<size_t>& subsampled_inds, float sampleDl);

void grid_subsampling_spheres(vector<PointXYZ>& original_points,
                              vector<PointXYZ>& subsampled_points,
                              float sampleDl);

void grid_subsampling(vector<PointXYZ>& original_points,
                      vector<PointXYZ>& subsampled_points,
                      vector<float>& original_features,
                      vector<float>& subsampled_features,
                      vector<int>& original_classes,
                      vector<int>& subsampled_classes, float sampleDl,
                      int verbose);

void batch_grid_subsampling(
    vector<PointXYZ>& original_points, vector<PointXYZ>& subsampled_points,
    vector<float>& original_features, vector<float>& subsampled_features,
    vector<int>& original_classes, vector<int>& subsampled_classes,
    vector<int>& original_batches, vector<int>& subsampled_batches,
    float sampleDl, int max_p);
