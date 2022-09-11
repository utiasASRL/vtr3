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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include "pcl_conversions/pcl_conversions.h"

#include "vtr_lidar/data_types/point.hpp"
#include "vtr_logging/logging_init.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

using PointCloudMsg = sensor_msgs::msg::PointCloud2;

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::lidar;

// clang-format off
TEST(LIDAR, point_cloud_access) {
  pcl::PointCloud<PointWithInfo> point_cloud;
  for (int i = 0; i < 10; i++) {
    PointWithInfo p;

    p.x = 1; p.y = 2; p.z = 3; p.data[3] = 4;
    p.normal_x = 5; p.normal_y = 6; p.normal_z = 7; p.data_n[3] = 8;
    p.flex11 = 9; p.flex12 = 10; p.flex13 = 11; p.flex14 = 12;
    p.timestamp = 1314; p.radial_velocity = 15; p.normal_score = 16;

    point_cloud.push_back(p);
  }

  // Get points cartesian coordinates as eigen map
  auto points_cart = point_cloud.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
  CLOG(INFO, "test") << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
  // Get points normal vector as eigen map
  auto points_normal = point_cloud.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
  CLOG(INFO, "test") << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
  // Get points polar coordinates as eigen map
  auto points_pol = point_cloud.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::polar_offset());
  CLOG(INFO, "test") << "Polar coordinates: " << "<" << points_pol.rows() << "," << points_pol.cols() << ">" << std::endl << points_pol;
  // Get points scores - \note the first two rows should be 0 and 4.6416 -> due to 1 double being interpreted as 2 floats
  auto scores = point_cloud.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
  CLOG(INFO, "test") << "Scores: " << "<" << scores.rows() << "," << scores.cols() << ">" << std::endl << scores;

  // Do something with the mapped data
  points_cart = points_pol;
  points_pol.setZero();

  // Get points cartesian coordinates as eigen map
  points_cart = point_cloud.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
  CLOG(INFO, "test") << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
  // Get points normal vector as eigen map
  points_normal = point_cloud.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
  CLOG(INFO, "test") << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
  // Get points polar coordinates as eigen map
  points_pol = point_cloud.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex1_offset());
  CLOG(INFO, "test") << "Polar coordinates: " << "<" << points_pol.rows() << "," << points_pol.cols() << ">" << std::endl << points_pol;
  // Get points scores - \note the first two rows should be 0 and 4.6416 -> due to 1 double being interpreted as 2 floats
  scores = point_cloud.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
  CLOG(INFO, "test") << "Scores: " << "<" << scores.rows() << "," << scores.cols() << ">" << std::endl << scores;
}

TEST(LIDAR, point_cloud_ros_conversion) {
  pcl::PointCloud<PointWithInfo> point_cloud;
  for (int i = 0; i < 10; i++) {
    PointWithInfo p;

    p.x = 1; p.y = 2; p.z = 3; p.data[3] = 4;
    p.normal_x = 5; p.normal_y = 6; p.normal_z = 7; p.data_n[3] = 8;
    p.flex11 = 9; p.flex12 = 10; p.flex13 = 11; p.flex14 = 12;
    p.timestamp = 1314; p.radial_velocity = 15; p.normal_score = 16;

    point_cloud.push_back(p);
  }

  // Get points cartesian coordinates as eigen map
  auto points_cart = point_cloud.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
  CLOG(INFO, "test") << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
  // Get points normal vector as eigen map
  auto points_normal = point_cloud.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
  CLOG(INFO, "test") << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
  // Get points polar coordinates as eigen map
  auto points_pol = point_cloud.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::polar_offset());
  CLOG(INFO, "test") << "Polar coordinates: " << "<" << points_pol.rows() << "," << points_pol.cols() << ">" << std::endl << points_pol;
  // Get points scores - \note the first two rows should be 0 and 4.6416 -> due to 1 double being interpreted as 2 floats
  auto scores = point_cloud.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
  CLOG(INFO, "test") << "Scores: " << "<" << scores.rows() << "," << scores.cols() << ">" << std::endl << scores;

  // convert to ros point cloud msg
  PointCloudMsg point_cloud_msg;
  pcl::toROSMsg(point_cloud, point_cloud_msg);
  // convert back
  pcl::PointCloud<PointWithInfo> point_cloud_2;
  pcl::fromROSMsg(point_cloud_msg, point_cloud_2);

  // Get points cartesian coordinates as eigen map
  points_cart = point_cloud_2.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
  CLOG(INFO, "test") << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
  // Get points normal vector as eigen map
  points_normal = point_cloud_2.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
  CLOG(INFO, "test") << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
  // Get points polar coordinates as eigen map
  points_pol = point_cloud_2.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex1_offset());
  CLOG(INFO, "test") << "Polar coordinates: " << "<" << points_pol.rows() << "," << points_pol.cols() << ">" << std::endl << points_pol;
  // Get points scores - \note the first two rows should be 0 and 4.6416 -> due to 1 double being interpreted as 2 floats
  scores = point_cloud_2.getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
  CLOG(INFO, "test") << "Scores: " << "<" << scores.rows() << "," << scores.cols() << ">" << std::endl << scores;
}

int main(int argc, char** argv) {
  configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
