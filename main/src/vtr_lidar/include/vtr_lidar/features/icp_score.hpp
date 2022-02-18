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
 * \file icp_score.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 * \brief heuristics for calculating icp weights
 */
namespace vtr {
namespace lidar {

template <class PointT>
std::vector<float> smartICPScore(pcl::PointCloud<PointT> &point_cloud) {
  // There are more points close to the lidar, so we dont want to pick them to
  // much. Furthermore, points away carry more rotational information.

  // Parameters
  float S0 = 0.5;
  float r0 = 5.0;

  // Variables
  float inv_ro = 1 / r0;
  float S1 = 1.0 + S0;

  std::vector<float> scores;
  scores.reserve(point_cloud.size());

  // loop over all
  for (auto &point : point_cloud) {
    point.icp_score =
        point.normal_score * (S1 - exp(-pow(point.rho * inv_ro, 2)));
    scores.emplace_back(point.icp_score);
  }

  return scores;
}

}  // namespace lidar
}  // namespace vtr