// Copyright 2024, Autonomous Space Robotics Lab (ASRL)
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
 * \file corridor_filter.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/filters/corridor_filter.hpp"



namespace vtr {
namespace lidar {

std::vector<int> points_near_vertex(const pcl::PointCloud<PointWithInfo>& point_cloud, Eigen::Vector4d path_point, double corridor_width) {
    std::vector<int> indices;
    indices.reserve(point_cloud.size());
    for (size_t i = 0; i < point_cloud.size(); ++i) {
        if (abs(path_point[0] - point_cloud[i].x) < corridor_width &&
            abs(path_point[2] - point_cloud[i].z) < 0.75)
        indices.emplace_back(i);
    }
    return indices;
}

pcl::PointCloud<PointWithInfo> filter_by_corridor(const pcl::PointCloud<PointWithInfo>& point_cloud, long curr_sid, double path_length, const tactic::LocalizationChain& chain, double corridor_width, 
        const tactic::EdgeTransform &T_cam_w) {
    auto lock = chain.guard();
    // compute vertex lookahead
    const auto distance = chain.dist(curr_sid);
    std::set<int> indices;

    // forwards
    for (auto query_sid = curr_sid;
         query_sid < chain.size()
              && (chain.dist(query_sid) - distance) < path_length;
         ++query_sid) {
      const auto T_cam_query = T_cam_w * chain.pose(query_sid);
      const auto p_cam_query = T_cam_query.matrix().block<4, 1>(0, 3);
      CLOG(DEBUG, "lidar.perspective") << "Pose origin" << p_cam_query;

      auto idx_i = points_near_vertex(point_cloud, p_cam_query, corridor_width);
      indices.insert(std::make_move_iterator(idx_i.begin()),
        std::make_move_iterator(idx_i.end()));
    }
    return pcl::PointCloud<PointWithInfo>(point_cloud, std::vector<int>{std::make_move_iterator(indices.begin()),
        std::make_move_iterator(indices.end())});
}


} //lidar
} //vtr