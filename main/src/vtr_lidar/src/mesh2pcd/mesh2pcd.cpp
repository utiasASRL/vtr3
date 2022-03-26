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
 * \file mesh2pcd.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/mesh2pcd/mesh2pcd.hpp"

#include <fstream>

#include "vtr_logging/logging.hpp"

namespace vtr {
namespace lidar {
namespace mesh2pcd {

Mesh2PcdConverter::Mesh2PcdConverter(const std::string& filename,
                                     const Config& config)
    : config_(config) {
  std::ifstream in(filename);
  if (!in.is_open())
    throw std::runtime_error("Failed to open file: " + filename);

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    // comment line
    if (line[0] == '#') continue;
    //
    std::istringstream iss(line);
    std::string token;
    iss >> token;
    if (token == "v") {
      // format is: "v <x> <y> <z>"
      float x, y, z;
      iss >> x >> y >> z;
      vertices_.emplace_back(x, y, z, 1.0);  // homogeneous coordinates
    } else if (token == "vn") {
      // format is: "vn <x> <y> <z>"
      float x, y, z;
      iss >> x >> y >> z;
      normals_.emplace_back(x, y, z, 0.0);  // homogeneous coordinates
    } else if (token == "f") {
      // format is: "f <v1>//<n1> <v2>//<n2> <v3>//<n3>"
      int v1, v2, v3, n1, n2, n3;
      const int ret = std::sscanf(line.c_str(), "f %d//%d %d//%d %d//%d", &v1,
                                  &n1, &v2, &n2, &v3, &n3);
      if (ret != 6)
        throw std::runtime_error("Failed to parse triangular face: " + line);
      // assuming n1, n2, n3 are all the same
      faces_.emplace_back(std::array<int, 4>{v1 - 1, v2 - 1, v3 - 1, n1 - 1});
    }
  }
  CLOG(INFO, "lidar.mesh2pcd")
      << "Loaded number of vertices: " << vertices_.size()
      << ", number of normals: " << normals_.size()
      << ", number of faces: " << faces_.size();
}

auto Mesh2PcdConverter::getCandidateRays(
    const size_t& ind, const std::vector<Eigen::Vector2f>& points) const
    -> CandidateRays {
  const auto& face = faces_.at(ind);
  // clang-format off
  float theta_min = std::numeric_limits<float>::max();
  float theta_max = std::numeric_limits<float>::min();
  float phi_min = std::numeric_limits<float>::max();
  float phi_max = std::numeric_limits<float>::min();
  float dphi = std::numeric_limits<float>::min();
  // boundary from point coordinates
  for (size_t i = 0; i < 3; ++i) {
    const auto& p1 = points.at(face[i]);
    const auto& p2 = points.at(face[(i + 1) % 3]);
    // theta
    theta_min = std::min(theta_min, p1[0]);
    theta_max = std::max(theta_max, p1[0]);
    // phi - need to consider the wrap around issue
    const float phi_min_tmp = std::min(p1[1], p2[1]);
    const float phi_max_tmp = std::max(p1[1], p2[1]);
    float dphi_tmp = phi_max_tmp - phi_min_tmp;
    dphi_tmp = dphi_tmp < M_PI ? dphi_tmp : 2 * M_PI - dphi_tmp;
    if (dphi_tmp > dphi) {
      dphi = dphi_tmp;
      phi_min = phi_min_tmp;
      phi_max = phi_max_tmp;
    }
  }
  // inflated boundary for theta (worst case: line is parallel to xy-plane)
  {
    float dphi = phi_max - phi_min;
    dphi = dphi < M_PI ? dphi : 2 * M_PI - dphi;
    {
      const float tmp_xy = std::cos(dphi / 2.0) * std::sin(theta_min);
      const float tmp_z = std::cos(theta_min);
      const float tmp_theta_min = std::atan2(tmp_xy, tmp_z);
      theta_min = std::min(theta_min, tmp_theta_min);
    }
    {
      const float tmp_xy = std::cos(dphi / 2.0) * std::sin(theta_max);
      const float tmp_z = std::cos(theta_max);
      const float tmp_theta_max = std::atan2(tmp_xy, tmp_z);
      theta_max = std::max(theta_max, tmp_theta_max);
    }
  }

  CandidateRays rays;
  // thetas
  const int theta_lb = std::max((int)std::floor(theta_min / config_.theta_res), (int)std::ceil(config_.theta_min / config_.theta_res));
  const int theta_ub = std::min((int)std::ceil(theta_max / config_.theta_res), (int)std::floor(config_.theta_max / config_.theta_res));
  for (int i = theta_lb; i <= theta_ub; ++i) rays.thetas.emplace_back(i);
  // phis
  if ((phi_max - phi_min) < M_PI) {
    const int phi_lb = std::max((int)std::floor(phi_min / config_.phi_res), (int)std::ceil(config_.phi_min / config_.phi_res));
    const int phi_ub = std::min((int)std::ceil(phi_max / config_.phi_res), (int)std::floor(config_.phi_max / config_.phi_res));
    for (int i = phi_lb; i <= phi_ub; ++i) rays.phis.emplace_back(i);
  } else {
    /// here we need to consider the +/-M_PI wrap around issue, so 2 segments
    //
    const int phi_lb1 = std::ceil(config_.phi_min / config_.phi_res);
    const int phi_ub1 = std::max((int)std::ceil(phi_min / config_.phi_res), (int)std::ceil(config_.phi_min / config_.phi_res));
    for (int i = phi_lb1; i <= phi_ub1; ++i) rays.phis.emplace_back(i);
    //
    const int phi_lb2 = std::min((int)std::floor(phi_max / config_.phi_res), (int)std::floor(config_.phi_max / config_.phi_res));
    const int phi_ub2 = std::floor(config_.phi_max / config_.phi_res);
    for (int i = phi_lb2; i <= phi_ub2; ++i) rays.phis.emplace_back(i);
  }
  // clang-format on
  return rays;
}

}  // namespace mesh2pcd
}  // namespace lidar
}  // namespace vtr