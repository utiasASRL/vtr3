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
 * \file mesh2pcd.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <array>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <Eigen/Core>

#include "vtr_common/utils/hash.hpp"

namespace vtr {
namespace lidar {
namespace mesh2pcd {

namespace utils {

inline float sign(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2,
                  const Eigen::Vector3f& p3, const Eigen::Vector3f& n) {
  return (p3 - p1).cross(p2 - p1).dot(n);
}

inline bool inside(const Eigen::Vector3f& p, const Eigen::Vector3f& p1,
                   const Eigen::Vector3f& p2, const Eigen::Vector3f& p3,
                   const Eigen::Vector3f& n) {
  const auto s1 = sign(p, p1, p2, n);
  const auto s2 = sign(p, p2, p3, n);
  const auto s3 = sign(p, p3, p1, n);
  return (s1 > 0 && s2 > 0 && s3 > 0) || (s1 < 0 && s2 < 0 && s3 < 0);
}

inline Eigen::Vector3f intersection(const Eigen::Vector3f& ln,
                                    const Eigen::Vector3f& pv,
                                    const Eigen::Vector3f& pn) {
  return ln * (pv.dot(pn) / ln.dot(pn));
}

inline Eigen::Vector3f cart2pol(const Eigen::Vector3f& cart) {
  // clang-format off
  const float rho = cart.norm();
  const float theta = std::atan2(std::sqrt(cart[0] * cart[0] + cart[1] * cart[1]), cart[2]);
  const float phi = std::atan2(cart[1], cart[0]);
  // clang-format on
  return Eigen::Vector3f(rho, theta, phi);
}

inline Eigen::Vector3f pol2cart(const Eigen::Vector3f& pol) {
  const float x = std::cos(pol[2]) * std::sin(pol[1]) * pol[0];
  const float y = std::sin(pol[2]) * std::sin(pol[1]) * pol[0];
  const float z = std::cos(pol[1]) * pol[0];
  return Eigen::Vector3f(x, y, z);
}

}  // namespace utils

/**
 * \brief Converts a mesh to a point cloud using ray-tracing.
 * \note Does not support over-head and under-head objects.
 */
class Mesh2PcdConverter {
 public:
  struct Config {
    // sensor specs
    double theta_min = 0.0;
    double theta_max = M_PI;
    double theta_res = 0.1;

    double phi_min = -M_PI;
    double phi_max = M_PI;
    double phi_res = 0.1;
  };

  Mesh2PcdConverter(const std::string& filename, const Config& config);

  template <class PointT>
  void addToPcd(
      pcl::PointCloud<PointT>& pcd,
      const Eigen::Matrix4f& T_pcd_obj = Eigen::Matrix4f::Identity()) const;

 private:
  struct Key {
    Key(int x0 = 0, int y0 = 0) : x(x0), y(y0) {}
    bool operator==(const Key& other) const {
      return (x == other.x && y == other.y);
    }
    int x, y;
  };
  friend class std::hash<Key>;
  Key getKey(const Eigen::Vector2f& p) const {
    return Key((int)std::round(p[0] / config_.theta_res),
               (int)std::round(p[1] / config_.phi_res));
  }

  struct CandidateRays {
    std::vector<int> thetas;
    std::vector<int> phis;
  };
  CandidateRays getCandidateRays(
      const size_t& ind, const std::vector<Eigen::Vector2f>& points) const;

 private:
  Config config_;

  std::vector<Eigen::Vector4f> vertices_;
  std::vector<Eigen::Vector4f> normals_;
  std::vector<std::array<int, 4>> faces_;
};

}  // namespace mesh2pcd
}  // namespace lidar
}  // namespace vtr

// Specialization of std:hash function
namespace std {

template <>
struct hash<vtr::lidar::mesh2pcd::Mesh2PcdConverter::Key> {
  std::size_t operator()(
      const vtr::lidar::mesh2pcd::Mesh2PcdConverter::Key& k) const {
    std::size_t ret = 0;
    vtr::common::hash_combine(ret, k.x, k.y);
    return ret;
  }
};

}  // namespace std

#include "vtr_lidar/mesh2pcd/mesh2pcd.inl"