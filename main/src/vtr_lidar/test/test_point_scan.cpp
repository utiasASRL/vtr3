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
 * \file test_point_scan.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include "vtr_lidar/data_types/point.hpp"
#include "vtr_lidar/data_types/pointmap.hpp"
#include "vtr_logging/logging_init.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr;
using namespace vtr::logging;
using namespace vtr::lidar;

TEST(LIDAR, point_scan_read_write) {
  auto point_scan = std::make_shared<PointScan<PointWithInfo>>();
  // create a test point cloud
  pcl::PointCloud<PointWithInfo> point_cloud;
  for (int i = 0; i < 5; i++) {
    PointWithInfo p;

    // clang-format off
    p.x = 1 + i; p.y = 2 + i; p.z = 3 + i;
    p.normal_x = 4 + i; p.normal_y = 5 + i; p.normal_z = 6 + i;
    p.flex11 = 7 + i; p.flex12 = 8 + i; p.flex13 = 9 + i; p.flex14 = 10 + i;
    p.time = 11 + i;
    p.radial_velocity = 12 + i;
    p.normal_score = 13 + i;

    point_cloud.push_back(p);
  }
  point_scan->point_cloud() = point_cloud;
  point_scan->T_vertex_this() = tactic::EdgeTransform(true);
  point_scan->vertex_id() = tactic::VertexId(1, 1);

  CLOG(INFO, "test") << point_scan->size();
  CLOG(INFO, "test") << point_scan->vertex_id();
  CLOG(INFO, "test") << point_scan->T_vertex_this();

  // clang-format off
  // Get points cartesian coordinates as eigen map
  auto points_cart = point_scan->point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
  CLOG(INFO, "test") << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
  // Get points normal vector as eigen map
  auto points_normal = point_scan->point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
  CLOG(INFO, "test") << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
  // Get points polar coordinates as eigen map
  auto points_pol = point_scan->point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::polar_offset());
  CLOG(INFO, "test") << "Polar coordinates: " << "<" << points_pol.rows() << "," << points_pol.cols() << ">" << std::endl << points_pol;
  // Get points normal score
  auto scores = point_scan->point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
  CLOG(INFO, "test") << "Normal variances: " << std::endl << scores.row(2);
  CLOG(INFO, "test") << "Normal scores: " << std::endl << scores.row(3);
  // clang-format on

  const auto msg = point_scan->toStorable();
  auto point_scan2 = PointScan<PointWithInfo>::fromStorable(msg);

  EXPECT_EQ(point_scan2->size(), point_scan->size());
  EXPECT_EQ(point_scan2->vertex_id(), point_scan->vertex_id());
  CLOG(INFO, "test") << point_scan2->T_vertex_this();

  // clang-format off
  // Get points cartesian coordinates as eigen map
  points_cart = point_scan2->point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
  CLOG(INFO, "test") << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
  // Get points normal vector as eigen map
  points_normal = point_scan2->point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
  CLOG(INFO, "test") << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
  // Get points polar coordinates as eigen map
  points_pol = point_scan2->point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::polar_offset());
  CLOG(INFO, "test") << "Polar coordinates: " << "<" << points_pol.rows() << "," << points_pol.cols() << ">" << std::endl << points_pol;
  // Get points normal score
  scores = point_scan2->point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
  CLOG(INFO, "test") << "Normal variances: " << std::endl << scores.row(2);
  CLOG(INFO, "test") << "Normal scores: " << std::endl << scores.row(3);
  // clang-format on
}

int main(int argc, char** argv) {
  configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
