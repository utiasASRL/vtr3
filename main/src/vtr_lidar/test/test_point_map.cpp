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
 * \file test_point_map.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include "vtr_lidar/pointmap/pointmap.hpp"
#include "vtr_lidar/pointmap/pointmap_v2.hpp"
#include "vtr_logging/logging_init.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr;
using namespace vtr::logging;
using namespace vtr::lidar;

TEST(LIDAR, point_map_read_write) {
  auto point_map = std::make_shared<PointMap<PointWithInfo>>(0.1);
  // create a test point cloud
  pcl::PointCloud<PointWithInfo> point_cloud;
  for (int i = 0; i < 10; i++) {
    PointWithInfo p;

    p.x = 1 + i;
    p.y = 2 + i;
    p.z = 3 + i;
    p.normal_x = 4 + i;
    p.normal_y = 5 + i;
    p.normal_z = 6 + i;
    p.rho = 7 + i;
    p.theta = 8 + i;
    p.phi = 9 + i;
    p.time = 10 + i;
    p.normal_score = 11 + i;
    p.icp_score = 12 + i;

    point_cloud.push_back(p);
  }
  point_map->update(point_cloud);
  point_map->T_vertex_map() = PointMap<PointWithInfo>::TransformType(true);
  point_map->vertex_id() = tactic::VertexId(1, 1);

  LOG(INFO) << point_map->size();
  LOG(INFO) << point_map->T_vertex_map();
  LOG(INFO) << point_map->vertex_id();

  // clang-format off
  // Get points cartesian coordinates as eigen map
  auto points_cart = point_map->point_map().getMatrixXfMap(/* dim */ 3, /* stride */ 16, /* offset */ 0);
  LOG(INFO) << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
  // Get points normal vector as eigen map
  auto points_normal = point_map->point_map().getMatrixXfMap(/* dim */ 3, /* stride */ 16, /* offset */ 4);
  LOG(INFO) << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
  // Get points polar coordinates as eigen map
  auto points_pol = point_map->point_map().getMatrixXfMap(/* dim */ 3, /* stride */ 16, /* offset */ 8);
  LOG(INFO) << "Polar coordinates: " << "<" << points_pol.rows() << "," << points_pol.cols() << ">" << std::endl << points_pol;
  // Get points normal score
  auto normal_score = point_map->point_map().getMatrixXfMap(/* dim */ 4, /* stride */ 16, /* offset */ 12);
  LOG(INFO) << "Normal scores: " << std::endl << normal_score.row(2);
  // clang-format on

  const auto msg = point_map->toStorable();

  auto point_map2 = PointMap<PointWithInfo>::fromStorable(msg);
  LOG(INFO) << point_map->size();
  LOG(INFO) << point_map->T_vertex_map();
  LOG(INFO) << point_map->vertex_id();

  // clang-format off
  // Get points cartesian coordinates as eigen map
  points_cart = point_map2->point_map().getMatrixXfMap(/* dim */ 3, /* stride */ 16, /* offset */ 0);
  LOG(INFO) << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
  // Get points normal vector as eigen map
  points_normal = point_map2->point_map().getMatrixXfMap(/* dim */ 3, /* stride */ 16, /* offset */ 4);
  LOG(INFO) << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
  // Get points polar coordinates as eigen map
  points_pol = point_map2->point_map().getMatrixXfMap(/* dim */ 3, /* stride */ 16, /* offset */ 8);
  LOG(INFO) << "Polar coordinates: " << "<" << points_pol.rows() << "," << points_pol.cols() << ">" << std::endl << points_pol;
  // Get points normal score
  normal_score = point_map2->point_map().getMatrixXfMap(/* dim */ 4, /* stride */ 16, /* offset */ 12);
  LOG(INFO) << "Normal scores: " << std::endl << normal_score.row(2);
  // clang-format on
}

int main(int argc, char** argv) {
  configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
