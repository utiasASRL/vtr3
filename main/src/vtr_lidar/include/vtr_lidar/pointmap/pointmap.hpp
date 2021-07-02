#pragma once

#include <vtr_lidar/cloud/cloud.h>
#include <vtr_lidar/nanoflann/nanoflann.hpp>

// KDTree type definition
using KDTree_Params = nanoflann::KDTreeSingleIndexAdaptorParams;
using PointXYZ_KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3>;
using PointXYZ_Dynamic_KDTree = nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3>;

namespace {
// Simple utility function to combine hashtables
template <typename T, typename... Rest>
void hash_combine(std::size_t& seed, const T& v, const Rest&... rest) {
  seed ^= std::hash<T>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  (hash_combine(seed, rest), ...);
}

}  // namespace

namespace vtr {
namespace lidar {

struct VoxKey {
  VoxKey(int x0 = 0, int y0 = 0, int z0 = 0) : x(x0), y(y0), z(z0) {}

  bool operator==(const VoxKey& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }

  int x, y, z;
};

inline VoxKey operator+(const VoxKey A, const VoxKey B) {
  return VoxKey(A.x + B.x, A.y + B.y, A.z + B.z);
}

inline VoxKey operator-(const VoxKey A, const VoxKey B) {
  return VoxKey(A.x - B.x, A.y - B.y, A.z - B.z);
}

struct PixKey {
  PixKey(int x0 = 0, int y0 = 0) : x(x0), y(y0) {}

  bool operator==(const PixKey& other) const {
    return (x == other.x && y == other.y);
  }

  int x, y;
};

inline PixKey operator+(const PixKey A, const PixKey B) {
  return PixKey(A.x + B.x, A.y + B.y);
}

inline PixKey operator-(const PixKey A, const PixKey B) {
  return PixKey(A.x - B.x, A.y - B.y);
}

}  // namespace lidar
}  // namespace vtr

// Specialization of std:hash function
namespace std {
using namespace vtr::lidar;

template <>
struct hash<VoxKey> {
  std::size_t operator()(const VoxKey& k) const {
    std::size_t ret = 0;
    hash_combine(ret, k.x, k.y, k.z);
    return ret;
  }
};

template <>
struct hash<PixKey> {
  std::size_t operator()(const PixKey& k) const {
    std::size_t ret = 0;
    hash_combine(ret, k.x, k.y);
    return ret;
  }
};
}  // namespace std

namespace vtr {
namespace lidar {

class PointMap {
 public:
  /** \brief Constructors */
  PointMap(const float dl = 1.0f)
      : dl_(dl), tree(3, cloud, KDTree_Params(10 /* max leaf */)) {}

  /** \brief Size of the map (number of point/voxel in the map) */
  size_t size() { return cloud.pts.size(); }

  std::pair<float, float> getMovability(VoxKey k) const {
    return this->movabilities[this->samples.at(k)];
  }

  // Update map with a set of new points
  void update(const std::vector<PointXYZ>& points,
              const std::vector<PointXYZ>& normals,
              const std::vector<float>& scores);

 private:
  VoxKey getKey(const PointXYZ& p) const {
    // Position of point in sample map
    PointXYZ p_pos = p / dl_;
    VoxKey k((int)floor(p_pos.x), (int)floor(p_pos.y), (int)floor(p_pos.z));
    return k;
  }

  void updateCapacity(size_t num_pts) {
    // Reserve new space if needed
    if (samples.empty()) samples.reserve(10 * num_pts);
    if (cloud.pts.capacity() < cloud.pts.size() + num_pts) {
      cloud.pts.reserve(cloud.pts.capacity() + num_pts);
      normals.reserve(normals.capacity() + num_pts);
      scores.reserve(scores.capacity() + num_pts);
      movabilities.reserve(movabilities.capacity() + num_pts);
    }
  }

  /** \brief Initialize a voxel centroid */
  void initSample(const VoxKey& k, const PointXYZ& p, const PointXYZ& n,
                  const float& s) {
    // We place anew key in the hashmap
    samples.emplace(k, cloud.pts.size());

    // We add new voxel data but initiate only the centroid
    cloud.pts.push_back(p);
    normals.push_back(n);
    scores.push_back(s);
    movabilities.push_back(std::pair<float, float>{0, 0});
  }

  // Update of voxel centroid
  void updateSample(const size_t idx, const PointXYZ&, const PointXYZ& n,
                    const float& s) {
    // Update normal if we have a clear view of it and closer distance (see
    // computation of score)
    if (s > scores[idx]) {
      scores[idx] = s;
      normals[idx] = n;
    }
  }

  void updateLimits(const VoxKey& k) {
    if (k.x < min_vox_.x) min_vox_.x = k.x;
    if (k.y < min_vox_.y) min_vox_.y = k.y;
    if (k.z < min_vox_.z) min_vox_.z = k.z;

    if (k.x > max_vox_.x) max_vox_.x = k.x;
    if (k.y > max_vox_.y) max_vox_.y = k.y;
    if (k.z > max_vox_.z) max_vox_.z = k.z;
  }

 private:
  // Voxel size
  float dl_ = 1.0f;

  // Map limits
  VoxKey max_vox_ =
      VoxKey(std::numeric_limits<int>::min(), std::numeric_limits<int>::min(),
             std::numeric_limits<int>::min());
  VoxKey min_vox_ =
      VoxKey(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(),
             std::numeric_limits<int>::max());

 public:
  // Containers for the data
  PointCloud cloud;
  std::vector<PointXYZ> normals;
  std::vector<float> scores;
  std::vector<std::pair<float, float>> movabilities;

  // Sparse hashmap that contain voxels (each voxel data is in the contiguous
  // vector containers)
  std::unordered_map<VoxKey, size_t> samples;

  // KDTree for neighbors query
  PointXYZ_Dynamic_KDTree tree;

  friend class PointMapMigrator;
};

class PointMapMigrator {
 public:
  /** \brief Constructors \todo also need to get the transformation!*/
  PointMapMigrator(const Eigen::Matrix4d& T_on, const PointMap& old_map,
                   PointMap& new_map)
      : C_on_(T_on.block<3, 3>(0, 0).cast<float>()),
        r_no_ino_(T_on.block<3, 1>(0, 3).cast<float>()),
        old_map_(old_map),
        new_map_(new_map) {}

  /** \brief Update map with a set of new points in new map frame */
  void update(const std::vector<PointXYZ>& points,
              const std::vector<PointXYZ>& normals,
              const std::vector<float>& scores);

 private:
  const Eigen::Matrix3f C_on_;
  const Eigen::Vector3f r_no_ino_;
  const PointMap& old_map_;
  PointMap& new_map_;
};

}  // namespace lidar
}  // namespace vtr