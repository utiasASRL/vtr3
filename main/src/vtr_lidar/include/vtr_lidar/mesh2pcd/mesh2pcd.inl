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
 * \file mesh2pcd.inl
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/mesh2pcd/mesh2pcd.hpp"

namespace vtr {
namespace lidar {
namespace mesh2pcd {

template <class PointT>
void Mesh2PcdConverter::addToPcd(pcl::PointCloud<PointT>& pcd,
                                 const Eigen::Matrix4f& T_pcd_obj) const {
  // object vertices to pcd frame
  std::vector<Eigen::Vector3f> vertices_org;
  vertices_org.reserve(vertices_.size());
  for (const auto& v : vertices_)
    vertices_org.emplace_back((T_pcd_obj * v).head<3>());

  // object normals to pcd frame
  std::vector<Eigen::Vector3f> normals_org;
  normals_org.reserve(normals_.size());
  for (const auto& n : normals_)
    normals_org.emplace_back((T_pcd_obj * n).head<3>());

  // vertices to polar coordinates
  std::vector<Eigen::Vector2f> vertices_polar;
  vertices_polar.reserve(vertices_org.size());
  for (auto& v : vertices_org)
    vertices_polar.emplace_back(utils::cart2pol(v).tail<2>());

  // construct frustum grid from existing points (assuming from a lidar scan)
  std::unordered_map<Key, std::pair<float, size_t>> key2depthidx;
  for (size_t i = 0; i < pcd.size(); ++i) {
    const auto& pt = pcd.at(i);
    const auto pt_polar = utils::cart2pol(pt.getVector3fMap());
    const auto& rho = pt_polar(0);
    const auto& theta = pt_polar(1);
    const auto& phi = pt_polar(2);
    const auto key = getKey(theta, phi);
    auto res = key2depthidx.try_emplace(key, rho, pcd.size());
    if (!res.second) {
      auto& depthidx = res.first->second;
      auto& depth = depthidx.first;
      auto& idx = depthidx.second;
      if (rho < depth) {
        depth = rho;
        idx = i;
      }
    }
  }

  // for each face
  for (size_t ind = 0; ind < faces_.size(); ++ind) {
    const auto& f = faces_.at(ind);
    // vertices and normal
    const auto& p1 = vertices_org.at(f[0]);
    const auto& p2 = vertices_org.at(f[1]);
    const auto& p3 = vertices_org.at(f[2]);
    const auto& n = normals_org.at(f[3]);

    // bounding box
    const auto rays = getCandidateRays(ind, vertices_polar);
    for (const auto& i : rays.thetas) {
      for (const auto& j : rays.phis) {
        const auto theta = i * config_.theta_res;
        const auto phi = j * config_.phi_res;

        // get point on the triangular plane
        const auto l = utils::pol2cart(Eigen::Vector3f(1.0, theta, phi));
        const auto q = utils::intersection(l, p1, n);

        // check if the point is in the triangle
        if (!utils::inside(q, p1, p2, p3, n)) continue;

        // get the distance to the query point
        const auto rho = q.norm();
        if (rho < config_.range_min || rho > config_.range_max) continue;

        // insert into the grid
        Key k{i, j};  // theta, phi
        auto res = key2depthidx.try_emplace(k, rho, pcd.size());
        if (res.second /* insertion is successful */) {
          PointT pt;
          pt.getVector3fMap() = q;
          // pt.getNormalVector3fMap() = n;  /// \todo face the sensor
          pcd.push_back(pt);
        } else {
          auto& depthidx = res.first->second;
          auto& depth = depthidx.first;
          auto& idx = depthidx.second;
          if (rho < depth) {
            auto& pt = pcd.at(idx);
            pt.getVector3fMap() = q;
            // pt.getNormalVector3fMap() = n;
            depth = rho;
          }
        }
      }  // end for each phi
    }    // end for each theta
  }      // end for each face
}

}  // namespace mesh2pcd
}  // namespace lidar
}  // namespace vtr