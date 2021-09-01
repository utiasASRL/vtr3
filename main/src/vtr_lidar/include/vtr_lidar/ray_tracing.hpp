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
 * \file ray_tracing.hpp
 * \brief Ray tracing utilities; FrustumGrid class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_lidar/polar_processing/polar_processing.hpp>

namespace vtr {
namespace lidar {

class FrustumGrid {
 public:
  FrustumGrid(const float& phi_res, const float& theta_res,
              const std::vector<PointXYZ>& points)
      : phi_res_(phi_res),
        theta_res_(theta_res),
        inner_ratio_(1 - std::max(phi_res, theta_res) / 2),
        outer_ratio_(1 + std::max(phi_res, theta_res) / 2),
        points_(points) {
    /// Create points in polar coordinates
    std::vector<PointXYZ> polar_points(points);
    cart2Pol_(polar_points);
    /// Insert into the frustum grid
    for (const auto& p : polar_points) {
      const auto k = getKey(p);
      if (frustum_grid_.count(k) == 0)
        frustum_grid_[k] = p.x;
      else
        /// // always choose the further point
        // frustum_grid_.at(k) = std::max(p.x, frustum_grid_.at(k));
        /// always choose the closer point
        frustum_grid_.at(k) = std::min(p.x, frustum_grid_.at(k));
    }
  }

  bool find(const PixKey& k) { return frustum_grid_.count(k) > 0; }

  bool isCloser(const PixKey& k, const float& rho) {
    return rho < (frustum_grid_.at(k) * inner_ratio_);
  }

  bool isFarther(const PixKey& k, const float& rho) {
    return rho > (frustum_grid_.at(k) * outer_ratio_);
  }

  PixKey getKey(const PointXYZ& p) const {
    // Position of point in sample map
    PixKey k((int)std::floor(p.y / theta_res_),
             (int)std::floor(p.z / phi_res_));
    return k;
  }

 private:
  /** \brief Resolution */
  const float phi_res_;
  const float theta_res_;
  const float inner_ratio_;
  const float outer_ratio_;

  const std::vector<PointXYZ>& points_;

  std::unordered_map<PixKey, float> frustum_grid_;
};

}  // namespace lidar
}  // namespace vtr