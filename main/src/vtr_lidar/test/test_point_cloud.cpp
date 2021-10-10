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
 * \file test_serialization_run.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include "vtr_lidar/types.hpp"
#include "vtr_logging/logging_init.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::lidar;
// clang-format off
TEST(LIDAR, point_cloud_access) {
  pcl::PointCloud<PointWithInfo> point_cloud;
  for (int i = 0; i < 10; i++) {
    PointWithInfo p;

    p.x = 1; p.y = 2; p.z = 3;
    p.normal_x = 4; p.normal_y = 5; p.normal_z = 6;
    p.rho = 7; p.theta = 8; p.phi = 9;
    p.time = 10; p.normal_score = 11; p.icp_score = 12;

    point_cloud.push_back(p);
  }

  // Get points cartesian coordinates as eigen map
  auto points_cart = point_cloud.getMatrixXfMap(/* dim */ 3, /* stride */ 16, /* offset */ 0);
  LOG(INFO) << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
  // Get points normal vector as eigen map
  auto points_normal = point_cloud.getMatrixXfMap(/* dim */ 3, /* stride */ 16, /* offset */ 4);
  LOG(INFO) << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
  // Get points polar coordinates as eigen map
  auto points_pol = point_cloud.getMatrixXfMap(/* dim */ 3, /* stride */ 16, /* offset */ 8);
  LOG(INFO) << "Polar coordinates: " << "<" << points_pol.rows() << "," << points_pol.cols() << ">" << std::endl << points_pol;
  // Get points normal score
  auto normal_score = point_cloud.getMatrixXfMap(/* dim */ 4, /* stride */ 16, /* offset */ 12);
  LOG(INFO) << "Normal scores: " << std::endl << normal_score.row(2);

  // Do something with the mapped data
  points_cart = points_pol;

  // Get points cartesian coordinates as eigen map
  points_cart = point_cloud.getMatrixXfMap(/* dim */ 3, /* stride */ 16, /* offset */ 0);
  LOG(INFO) << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
  // Get points normal vector as eigen map
  points_normal = point_cloud.getMatrixXfMap(/* dim */ 3, /* stride */ 16, /* offset */ 4);
  LOG(INFO) << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
  // Get points polar coordinates as eigen map
  points_pol = point_cloud.getMatrixXfMap(/* dim */ 3, /* stride */ 16, /* offset */ 8);
  LOG(INFO) << "Polar coordinates: " << "<" << points_pol.rows() << "," << points_pol.cols() << ">" << std::endl << points_pol;

}

int main(int argc, char** argv) {
  configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
