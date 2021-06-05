#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>

#include "vtr_lidar/cloud/cloud.h"
#include "vtr_lidar/nanoflann/nanoflann.hpp"

using namespace std;

// KDTree type definition
typedef nanoflann::KDTreeSingleIndexAdaptorParams KDTree_Params;
typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3>
    PointXYZ_KDTree;
typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3>
    PointXYZ_Dynamic_KDTree;

//-------------------------------------------------------------------------------------------
//
// PointMapPython Class
// ********************
//
//	PointMap designed to be used in python. As it is hard to transfert
// unordered map to 	python dict structure, we rebuild the hashmap every
// update (not very efficient).
//
//-------------------------------------------------------------------------------------------

class MapVoxelData {
 public:
  // Elements
  // ********

  bool occupied;
  int count;
  PointXYZ centroid;
  PointXYZ normal;
  float score;

  // Methods
  // *******

  // Constructor
  MapVoxelData() {
    occupied = false;
    count = 0;
    score = -1.0f;
    centroid = PointXYZ();
    normal = PointXYZ();
  }
  MapVoxelData(const PointXYZ p0, const PointXYZ n0, const float s0,
               const int c0) {
    occupied = true;
    count = c0;
    score = s0;
    centroid = p0;
    normal = n0;
  }

  MapVoxelData(const PointXYZ p0) {
    // We initiate only the centroid
    count = 1;
    centroid = p0;

    // Other varaible are kept null
    occupied = false;
    score = -1.0f;
    normal = PointXYZ();
  }

  void update_centroid(const PointXYZ p0) {
    count += 1;
    centroid += p0;
  }

  void update_normal(const float s0, const PointXYZ n0) {
    // We keep the worst normal
    occupied = true;

    // Rule for normal update:
    // IF current_score=2 : normal was computed with planarity in the map, do
    // not modify IF s0 < score - 0.1 : Too bad score dont update (This includes
    // the condition above) IF s0 > score + 0.1 : Better score, use new normal
    // IF abs(s0 - score) < 0.1 : Similar score, avergae normal
    // When averaging be careful of orientation. Dont worry about norm, we
    // renormalize every normal in the end

    if (s0 > score + 0.1) {
      score = s0;
      normal = n0;
    } else if (s0 > score - 0.1) {
      if (s0 > score) score = s0;
      if (normal.dot(n0) > 0)
        normal += n0;
      else
        normal -= n0;
    }
  }
};

class PointMapPython {
 public:
  // Elements
  // ********

  float dl;
  vector<PointXYZ> points;
  vector<PointXYZ> normals;
  vector<float> scores;
  vector<int> counts;

  // Methods
  // *******

  // Constructor
  PointMapPython() { dl = 1.0f; }
  PointMapPython(const float dl0) { dl = dl0; }

  // Methods
  void update(vector<PointXYZ>& points0, vector<PointXYZ>& normals0,
              vector<float>& scores0);

  void init_samples(const PointXYZ originCorner, const PointXYZ maxCorner,
                    unordered_map<size_t, MapVoxelData>& samples);

  void add_samples(const vector<PointXYZ>& points0,
                   const vector<PointXYZ>& normals0,
                   const vector<float>& scores0, const PointXYZ originCorner,
                   const PointXYZ maxCorner,
                   unordered_map<size_t, MapVoxelData>& samples);

  size_t size() { return points.size(); }
};

//-------------------------------------------------------------------------------------------
//
// VoxKey
// ******
//
//	Here we define a struct that will be used as key in our hash map. It
// contains 3 integers.
//  Then we specialize the std::hash function for this class.
//
//-------------------------------------------------------------------------------------------

class VoxKey {
 public:
  int x;
  int y;
  int z;

  VoxKey() {
    x = 0;
    y = 0;
    z = 0;
  }
  VoxKey(int x0, int y0, int z0) {
    x = x0;
    y = y0;
    z = z0;
  }

  bool operator==(const VoxKey& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

inline VoxKey operator+(const VoxKey A, const VoxKey B) {
  return VoxKey(A.x + B.x, A.y + B.y, A.z + B.z);
}

inline VoxKey operator-(const VoxKey A, const VoxKey B) {
  return VoxKey(A.x - B.x, A.y - B.y, A.z - B.z);
}

// Simple utility function to combine hashtables
template <typename T, typename... Rest>
void hash_combine(std::size_t& seed, const T& v, const Rest&... rest) {
  seed ^= std::hash<T>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  (hash_combine(seed, rest), ...);
}

// Specialization of std:hash function
namespace std {
template <>
struct hash<VoxKey> {
  std::size_t operator()(const VoxKey& k) const {
    std::size_t ret = 0;
    hash_combine(ret, k.x, k.y, k.z);
    return ret;
  }
};
}  // namespace std

//-------------------------------------------------------------------------------------------
//
// PixKey
// ******
//
//	Same as VoxKey but in 2D
//
//-------------------------------------------------------------------------------------------

class PixKey {
 public:
  int x;
  int y;

  PixKey() {
    x = 0;
    y = 0;
  }
  PixKey(int x0, int y0) {
    x = x0;
    y = y0;
  }

  bool operator==(const PixKey& other) const {
    return (x == other.x && y == other.y);
  }
};

inline PixKey operator+(const PixKey A, const PixKey B) {
  return PixKey(A.x + B.x, A.y + B.y);
}

inline PixKey operator-(const PixKey A, const PixKey B) {
  return PixKey(A.x - B.x, A.y - B.y);
}

// Specialization of std:hash function
namespace std {
template <>
struct hash<PixKey> {
  std::size_t operator()(const PixKey& k) const {
    std::size_t ret = 0;
    hash_combine(ret, k.x, k.y);
    return ret;
  }
};
}  // namespace std

//-------------------------------------------------------------------------------------------
//
// PointMap Class
// **************
//
//	PointMap designed to be used in C++. Everything should be more efficient
// here.
//
//-------------------------------------------------------------------------------------------

class PointMap {
 public:
  // Elements
  // ********

  // Voxel size
  float dl;

  // Count the number of frames used tu update this map
  int update_idx;

  // Map limits
  VoxKey minVox;
  VoxKey maxVox;

  // Containers for the data
  PointCloud cloud;
  vector<PointXYZ> normals;
  vector<float> scores;
  vector<int> counts;

  // Sparse hashmap that contain voxels (each voxel data is in the contiguous
  // vector containers)
  unordered_map<VoxKey, size_t> samples;

  // KDTree for neighbors query
  PointXYZ_Dynamic_KDTree tree;

  // Methods
  // *******

  // Constructor
  PointMap() : tree(3, cloud, KDTree_Params(10 /* max leaf */)) {
    dl = 1.0f;
    update_idx = 0;
  }
  PointMap(const float dl0) : tree(3, cloud, KDTree_Params(10 /* max leaf */)) {
    dl = dl0;
    update_idx = 0;
  }
  PointMap(const float dl0, vector<PointXYZ>& init_points,
           vector<PointXYZ>& init_normals, vector<float>& init_scores)
      : tree(3, cloud, KDTree_Params(10 /* max leaf */)) {
    // Set voxel size
    dl = dl0;

    // Init limits
    maxVox.x = numeric_limits<int>::min();
    maxVox.y = numeric_limits<int>::min();
    maxVox.z = numeric_limits<int>::min();
    minVox.x = numeric_limits<int>::max();
    minVox.y = numeric_limits<int>::max();
    minVox.z = numeric_limits<int>::max();

    // Optionally init map
    update_idx = 0;
    if (init_points.size() > 0) {
      update_idx = -1;
      update(init_points, init_normals, init_scores);
    }
  }

  // Size of the map (number of point/voxel in the map)
  size_t size() { return cloud.pts.size(); }

  // Init of voxel centroid
  void init_sample(const VoxKey& k, const PointXYZ& p0, const PointXYZ& n0,
                   const float& s0, const int& c0) {
    // We place anew key in the hashmap
    samples.emplace(k, cloud.pts.size());

    // We add new voxel data but initiate only the centroid
    cloud.pts.push_back(p0);
    normals.push_back(n0);
    scores.push_back(s0);

    // Count is useless, instead save index of first frame placing a point in
    // this cell
    counts.push_back(c0);
  }

  // Update of voxel centroid
  void update_sample(const size_t idx, const PointXYZ& p0, const PointXYZ& n0,
                     const float& s0) {
    // Update count for optional removal count of points (USELESS see
    // init_sample)
    // counts[idx] += 1;

    // Update normal if we have a clear view of it  and closer distance (see
    // computation of score)
    if (s0 > scores[idx]) {
      scores[idx] = s0;
      normals[idx] = n0;
    }
  }

  void update_limits(const VoxKey& k) {
    if (k.x < minVox.x) minVox.x = k.x;
    if (k.y < minVox.y) minVox.y = k.y;
    if (k.z < minVox.z) minVox.z = k.z;

    if (k.x > maxVox.x) maxVox.x = k.x;
    if (k.y > maxVox.y) maxVox.y = k.y;
    if (k.z > maxVox.z) maxVox.z = k.z;
  }

  // Update map with a set of new points
  void update(vector<PointXYZ>& points0, vector<PointXYZ>& normals0,
              vector<float>& scores0) {
    // Reserve new space if needed
    if (samples.size() < 1) samples.reserve(10 * points0.size());
    if (cloud.pts.capacity() < cloud.pts.size() + points0.size()) {
      cloud.pts.reserve(cloud.pts.capacity() + points0.size());
      counts.reserve(counts.capacity() + points0.size());
      normals.reserve(normals.capacity() + points0.size());
      scores.reserve(scores.capacity() + points0.size());
    }

    // std::cout << std::endl << "--------------------------------------" <<
    // std::endl; std::cout << "current max_load_factor: " <<
    // samples.max_load_factor() << std::endl; std::cout << "current size: " <<
    // samples.size() << std::endl; std::cout << "current bucket_count: " <<
    // samples.bucket_count() << std::endl; std::cout << "current load_factor: "
    // << samples.load_factor() << std::endl; std::cout <<
    // "--------------------------------------" << std::endl << std::endl;

    // Initialize variables
    float inv_dl = 1 / dl;
    size_t i = 0;
    VoxKey k0;
    size_t num_added = 0;

    for (auto& p : points0) {
      // Position of point in sample map
      PointXYZ p_pos = p * inv_dl;

      // Corresponding key
      k0.x = (int)floor(p_pos.x);
      k0.y = (int)floor(p_pos.y);
      k0.z = (int)floor(p_pos.z);

      // Update the point count
      if (samples.count(k0) < 1) {
        // Create a new sample at this location
        init_sample(k0, p, normals0[i], scores0[i], update_idx);
        num_added++;

        // Update grid limits
        update_limits(k0);
      } else
        update_sample(samples[k0], p, normals0[i], scores0[i]);
      i++;
    }

    // Update tree
    tree.addPoints(cloud.pts.size() - num_added, cloud.pts.size() - 1);

    // Update frame count
    update_idx++;
  }

  // Debug method that saves the map as ply file
  void debug_save_ply(string& path, int idx) {
    cout << endl << "---------------------------------------------" << endl;
    char buffer[200];
    sprintf(buffer, "pointmap_%05d.ply", (int)idx);
    string filepath = path + string(buffer);
    cout << filepath << endl;
    save_cloud(filepath, cloud.pts, normals, scores);
    cout << "---------------------------------------------" << endl << endl;
  }
};

class OccupGrid2D {
 public:
  // Elements
  // ********

  // Voxel size
  float dl;

  // Maximum value of the counts
  int max_count;

  // Map limits
  PixKey minPix;
  PixKey maxPix;

  vector<float> scores;
  vector<int> counts;
  vector<PointXY> points;

  // Sparse hashmap that contain voxels (each voxel data is in the contiguous
  // vector containers)
  unordered_map<PixKey, size_t> samples;

  // Methods
  // *******

  // Constructor
  OccupGrid2D() {
    dl = 1.0f;
    max_count = 10;
  }
  OccupGrid2D(const float dl0, const int m0) {
    dl = dl0;
    max_count = m0;
  }

  // Size of the map (number of point/pixel in the map)
  size_t size() { return points.size(); }

  // Init of pixel centroid
  void init_sample(const PixKey& k, const PointXY& p0, const float& s0) {
    // We place anew key in the hashmap
    samples.emplace(k, points.size());

    // We add new voxel data but initiate only the centroid
    points.push_back(p0);
    counts.push_back(1);
    scores.push_back(s0);
  }

  // Update of voxel centroid
  void update_sample(const size_t idx, const float& s0) {
    // Update only count for optional removal count of points and centroid of
    // the cell
    if (counts[idx] < max_count)
      scores[idx] += (s0 - scores[idx]) / ++counts[idx];
    else
      scores[idx] += (s0 - scores[idx]) / max_count;
  }

  void update_limits(const PixKey& k) {
    if (k.x < minPix.x) minPix.x = k.x;
    if (k.y < minPix.y) minPix.y = k.y;

    if (k.x > maxPix.x) maxPix.x = k.x;
    if (k.y > maxPix.y) maxPix.y = k.y;
  }

  // Update map with a set of new points
  void update_from_3D(vector<PointXYZ>& points3D, PointXYZ& center3D,
                      Plane3D& ground_P, float zMin, float zMax) {
    ////////////////
    // Init steps //
    ////////////////

    // TODO: Every once and a while delete the pixels that have a low score
    // (<0.1). 		 Just create again the samples, vectors etc.
    //

    // Reserve new space if needed
    if (points.size() < 1) {
      samples.reserve(points3D.size());
      points.reserve(points3D.size());
      counts.reserve(points3D.size());
      scores.reserve(points3D.size());
    }

    // Initialize variables
    float inv_dl = 1 / dl;
    PixKey k0;

    // Every pixel can be updated only once.
    vector<bool> not_updated(points.size(), true);

    //////////////////////////
    // Convert to 2D ranges //
    //////////////////////////

    // Init free range table (1D grid containing range for each angle)
    float angle_res = 0.5 * M_PI / 180.0;
    size_t n_angles = (size_t)floor(2.0 * M_PI / angle_res) + 1;
    vector<float> range_table(n_angles, -1.0);

    // Get distances to ground
    vector<float> distances;
    ground_P.point_distances(points3D, distances);

    ////////////////////////
    // Update full pixels //
    ////////////////////////

    // Loop over 3D points
    size_t p_i = 0;
    for (auto& p : points3D) {
      // Check height limits
      if (distances[p_i] < zMin || distances[p_i] > zMax) {
        p_i++;
        continue;
      }

      // Corresponding key
      k0.x = (int)floor(p.x * inv_dl);
      k0.y = (int)floor(p.y * inv_dl);

      // Update the point count
      if (samples.count(k0) < 1) {
        // Create a new sample at this location
        init_sample(k0,
                    PointXY(((float)k0.x + 0.5) * dl, ((float)k0.y + 0.5) * dl),
                    1.0);

        // Update grid limits
        update_limits(k0);
      } else {
        size_t i0 = samples[k0];
        if (i0 < not_updated.size() && not_updated[i0]) {
          update_sample(i0, 1.0);
          not_updated[i0] = false;
        }
      }

      // Add the angle and its corresponding free_range
      PointXY diff2D(p - center3D);
      size_t angle_idx =
          (size_t)floor((atan2(diff2D.y, diff2D.x) + M_PI) / angle_res);
      float d2 = diff2D.sq_norm();
      if (range_table[angle_idx] < 0 || d2 < pow(range_table[angle_idx], 2))
        range_table[angle_idx] = sqrt(d2);

      p_i++;
    }

    ///////////////////////////////
    // Interpolate the 2D ranges //
    ///////////////////////////////

    // First find the last valid value
    int last_i, next_i;
    last_i = range_table.size() - 1;
    while (last_i >= 0) {
      if (range_table[last_i] > 0) break;
      last_i--;
    }

    // Interpolate
    next_i = 0;
    last_i -= range_table.size();
    while (next_i < range_table.size()) {
      if (range_table[next_i] > 0) {
        if (last_i < 0) {
          int diff = next_i - last_i;
          if (diff > 1) {
            for (int i = last_i + 1; i < next_i; i++) {
              int real_i = i;
              if (real_i < 0) real_i += range_table.size();
              float t = (i - last_i) / diff;
              int real_last_i = last_i + range_table.size();
              range_table[real_i] =
                  t * range_table[real_last_i] + (1 - t) * range_table[next_i];
            }
          }
        } else {
          int diff = next_i - last_i;
          if (diff > 1) {
            for (int i = last_i + 1; i < next_i; i++) {
              float t = (i - last_i) / diff;
              range_table[i] =
                  t * range_table[last_i] + (1 - t) * range_table[next_i];
            }
          }
        }
        last_i = next_i;
      }
      next_i++;
    }

    ////////////////////////
    // Update free pixels //
    ////////////////////////

    // Apply margin to free ranges
    float margin = dl;
    for (auto& r : range_table) r -= margin;

    // Update free pixels
    PointXY center2D(center3D);
    float min_r2 = pow(0.1, 2);
    p_i = 0;
    for (auto& p : points) {
      // Ignore points updated just now
      if (p_i >= not_updated.size()) break;
      if (!not_updated[p_i]) {
        p_i++;
        continue;
      }

      // Compute angle and range
      PointXY diff2D(p - center2D);
      size_t angle_idx = (atan2(diff2D.y, diff2D.x) + M_PI) / angle_res;
      float d2 = diff2D.sq_norm();

      // Update score
      if (d2 > min_r2 && d2 < pow(range_table[angle_idx], 2))
        update_sample(p_i, 0.0);
      p_i++;
    }
  }

  // Debug method that saves the map as ply file
  void debug_save_ply(string& path, int idx) {
    vector<PointXYZ> points3D;
    points3D.reserve(points.size());
    for (auto& p : points) points3D.push_back(PointXYZ(p.x, p.y, 0));

    cout << endl << "---------------------------------------------" << endl;
    vector<float> features(scores);
    features.reserve(scores.size() * 2);
    for (auto& c : counts) features.push_back((float)c);

    char buffer[200];
    sprintf(buffer, "debug_map2D_%05d.ply", (int)idx);
    string filepath = path + string(buffer);
    cout << filepath << endl;
    save_cloud(filepath, points3D, features);
    cout << "---------------------------------------------" << endl << endl;
  }
};