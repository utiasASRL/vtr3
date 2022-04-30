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
 * \file test_multi_exp_point_map.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include <bitset>

#include "vtr_lidar/data_types/multi_exp_pointmap.hpp"
#include "vtr_lidar/data_types/point.hpp"
#include "vtr_logging/logging_init.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr;
using namespace vtr::logging;
using namespace vtr::lidar;

TEST(LIDAR, multi_exp_point_map_constructor) {
  EXPECT_THROW(MultiExpPointMap<PointWithInfo>(0.1, 1000), std::runtime_error);
  EXPECT_THROW(MultiExpPointMap<PointWithInfo>(0.1, 0), std::runtime_error);
  EXPECT_NO_THROW(MultiExpPointMap<PointWithInfo>(0.1, 20));
}

TEST(LIDAR, multi_exp_point_map_update) {
  MultiExpPointMap<PointWithInfo> multi_exp_map(0.1, 2);

  // Initial update
  {
    PointMap<PointWithInfo> point_map(0.1,
                                      PointMap<PointWithInfo>::DYNAMIC_REMOVED);
    pcl::PointCloud<PointWithInfo> point_cloud;
    for (int i = 0; i < 3; i++) {
      PointWithInfo p;
      // clang-format off
      p.x = p.y = p.z = i;
      p.normal_x = p.normal_y = p.normal_z = i;
      p.radial_velocity = i; p.normal_score = 1;
      // clang-format on
      point_cloud.push_back(p);
    }
    point_map.update(point_cloud);
    point_map.T_vertex_this() = tactic::EdgeTransform(true);
    point_map.vertex_id() = tactic::VertexId(1, 1);
    // fake dynamic points removal
    for (auto& p : point_map.point_cloud()) p.static_score = 1;
    multi_exp_map.update(point_map);

    EXPECT_EQ(multi_exp_map.size(), (size_t)3);
    EXPECT_EQ(multi_exp_map.experiences().size(), (size_t)1);

    // clang-format off
    // Get points cartesian coordinates as eigen map
    auto points_cart = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
    CLOG(INFO, "test") << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
    // Get points normal vector as eigen map
    auto points_normal = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
    CLOG(INFO, "test") << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
    // Get points polar coordinates as eigen map
    std::stringstream ss;
    ss << "Bit vector: ";
    for (auto& p: multi_exp_map.point_cloud()) {
      ss << std::endl << std::bitset<128>(p.bits);
    }
    CLOG(INFO, "test") << ss.str();
    // Get points normal score
    auto scores = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
    CLOG(INFO, "test") << "Normal variances: " << std::endl << scores.row(2);
    CLOG(INFO, "test") << "Normal scores: " << std::endl << scores.row(3);
    // clang-format on
  }

  // Update with exact same points should only update bit vector
  {
    PointMap<PointWithInfo> point_map(0.1,
                                      PointMap<PointWithInfo>::DYNAMIC_REMOVED);
    pcl::PointCloud<PointWithInfo> point_cloud;
    for (int i = 0; i < 3; i++) {
      PointWithInfo p;
      // clang-format off
      p.x = p.y = p.z = i;
      p.normal_x = p.normal_y = p.normal_z = i;
      p.radial_velocity = i; p.normal_score = 1;
      // clang-format on
      point_cloud.push_back(p);
    }
    point_map.update(point_cloud);
    point_map.T_vertex_this() = tactic::EdgeTransform(true);
    point_map.vertex_id() = tactic::VertexId(1, 1);
    // fake dynamic points removal
    for (auto& p : point_map.point_cloud()) p.static_score = 1;
    multi_exp_map.update(point_map);

    EXPECT_EQ(multi_exp_map.size(), (size_t)3);
    EXPECT_EQ(multi_exp_map.experiences().size(), (size_t)2);

    // clang-format off
    // Get points cartesian coordinates as eigen map
    auto points_cart = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
    CLOG(INFO, "test") << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
    // Get points normal vector as eigen map
    auto points_normal = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
    CLOG(INFO, "test") << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
    // Get points polar coordinates as eigen map
    std::stringstream ss;
    ss << "Bit vector: ";
    for (auto& p: multi_exp_map.point_cloud()) {
      ss <<std::endl << std::bitset<128>(p.bits);
    }
    CLOG(INFO, "test") << ss.str();
    // Get points normal score
    auto scores = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
    CLOG(INFO, "test") << "Normal variances: " << std::endl << scores.row(2);
    CLOG(INFO, "test") << "Normal scores: " << std::endl << scores.row(3);
    // clang-format on
  }

  // Update with completely different points should add more points
  {
    PointMap<PointWithInfo> point_map(0.1,
                                      PointMap<PointWithInfo>::DYNAMIC_REMOVED);
    pcl::PointCloud<PointWithInfo> point_cloud;
    for (int i = 3; i < 6; i++) {
      PointWithInfo p;
      // clang-format off
      p.x = p.y = p.z = i;
      p.normal_x = p.normal_y = p.normal_z = i;
      p.radial_velocity = i; p.normal_score = 1;
      // clang-format on
      point_cloud.push_back(p);
    }
    point_map.update(point_cloud);
    point_map.T_vertex_this() = tactic::EdgeTransform(true);
    point_map.vertex_id() = tactic::VertexId(1, 1);
    // fake dynamic points removal
    for (auto& p : point_map.point_cloud()) p.static_score = 1;
    multi_exp_map.update(point_map);

    EXPECT_EQ(multi_exp_map.size(), (size_t)6);
    EXPECT_EQ(multi_exp_map.experiences().size(), (size_t)2);

    // clang-format off
    // Get points cartesian coordinates as eigen map
    auto points_cart = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
    CLOG(INFO, "test") << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
    // Get points normal vector as eigen map
    auto points_normal = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
    CLOG(INFO, "test") << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
    // Get points polar coordinates as eigen map
    std::stringstream ss;
    ss << "Bit vector: ";
    for (auto& p: multi_exp_map.point_cloud()) {
      ss <<std::endl << std::bitset<128>(p.bits);
    }
    CLOG(INFO, "test") << ss.str();
    // Get points normal score
    auto scores = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
    CLOG(INFO, "test") << "Normal variances: " << std::endl << scores.row(2);
    CLOG(INFO, "test") << "Normal scores: " << std::endl << scores.row(3);
    // clang-format on
  }

  // Update with completely different points should add more points
  {
    PointMap<PointWithInfo> point_map(0.1,
                                      PointMap<PointWithInfo>::DYNAMIC_REMOVED);
    pcl::PointCloud<PointWithInfo> point_cloud;
    for (int i = 3; i < 6; i++) {
      PointWithInfo p;
      // clang-format off
      p.x = p.y = p.z = i;
      p.normal_x = p.normal_y = p.normal_z = i;
      p.radial_velocity = i; p.normal_score = 0;
      // clang-format on
      point_cloud.push_back(p);
    }
    point_map.update(point_cloud);
    point_map.T_vertex_this() = tactic::EdgeTransform(true);
    point_map.vertex_id() = tactic::VertexId(1, 1);
    // fake dynamic points removal
    for (auto& p : point_map.point_cloud()) p.static_score = 1;
    multi_exp_map.update(point_map);

    EXPECT_EQ(multi_exp_map.size(), (size_t)3);
    EXPECT_EQ(multi_exp_map.experiences().size(), (size_t)2);

    // clang-format off
    // Get points cartesian coordinates as eigen map
    auto points_cart = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
    CLOG(INFO, "test") << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
    // Get points normal vector as eigen map
    auto points_normal = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
    CLOG(INFO, "test") << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
    // Get points polar coordinates as eigen map
    std::stringstream ss;
    ss << "Bit vector: ";
    for (auto& p: multi_exp_map.point_cloud()) {
      ss <<std::endl << std::bitset<128>(p.bits);
    }
    CLOG(INFO, "test") << ss.str();
    // Get points normal score
    auto scores = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
    CLOG(INFO, "test") << "Normal variances: " << std::endl << scores.row(2);
    CLOG(INFO, "test") << "Normal scores: " << std::endl << scores.row(3);
    // clang-format on
  }
}

TEST(LIDAR, point_map_read_write) {
  MultiExpPointMap<PointWithInfo> multi_exp_map(0.1, 2);

  PointMap<PointWithInfo> point_map(0.1,
                                    PointMap<PointWithInfo>::DYNAMIC_REMOVED);
  pcl::PointCloud<PointWithInfo> point_cloud;
  for (int i = 0; i < 10; i++) {
    PointWithInfo p;
    // clang-format off
      p.x = p.y = p.z = i;
      p.normal_x = p.normal_y = p.normal_z = i;
      p.radial_velocity = i; p.normal_score = 1;
    // clang-format on
    point_cloud.push_back(p);
  }
  point_map.update(point_cloud);
  point_map.T_vertex_this() = tactic::EdgeTransform(true);
  point_map.vertex_id() = tactic::VertexId(1, 1);
  // fake dynamic points removal
  for (auto& p : point_map.point_cloud()) p.static_score = 0;
  multi_exp_map.update(point_map);

  CLOG(INFO, "test") << multi_exp_map.size();
  CLOG(INFO, "test") << multi_exp_map.T_vertex_this();
  CLOG(INFO, "test") << multi_exp_map.vertex_id();
  CLOG(INFO, "test") << multi_exp_map.dl();
  CLOG(INFO, "test") << multi_exp_map.version();
  CLOG(INFO, "test") << multi_exp_map.experiences().size();
  CLOG(INFO, "test") << multi_exp_map.experiences()[0];
  {
    // clang-format off
    // Get points cartesian coordinates as eigen map
    auto points_cart = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
    CLOG(INFO, "test") << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
    // Get points normal vector as eigen map
    auto points_normal = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
    CLOG(INFO, "test") << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
    // Get points polar coordinates as eigen map
    std::stringstream ss;
    ss << "Bit vector: ";
    for (auto& p: multi_exp_map.point_cloud()) {
      ss <<std::endl << std::bitset<128>(p.bits);
    }
    CLOG(INFO, "test") << ss.str();
    // Get points normal score
    auto scores = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
    CLOG(INFO, "test") << "Normal variances: " << std::endl << scores.row(2);
    CLOG(INFO, "test") << "Normal scores: " << std::endl << scores.row(3);
    // clang-format on
  }
  const auto msg = multi_exp_map.toStorable();
  auto multi_exp_map2 = MultiExpPointMap<PointWithInfo>::fromStorable(msg);

  CLOG(INFO, "test") << multi_exp_map2->size();
  CLOG(INFO, "test") << multi_exp_map2->T_vertex_this();
  CLOG(INFO, "test") << multi_exp_map2->vertex_id();
  CLOG(INFO, "test") << multi_exp_map2->dl();
  CLOG(INFO, "test") << multi_exp_map2->version();
  CLOG(INFO, "test") << multi_exp_map2->experiences().size();
  CLOG(INFO, "test") << multi_exp_map2->experiences()[0];
  {
    // clang-format off
    // Get points cartesian coordinates as eigen map
    auto points_cart = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
    CLOG(INFO, "test") << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
    // Get points normal vector as eigen map
    auto points_normal = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
    CLOG(INFO, "test") << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
    // Get points polar coordinates as eigen map
    std::stringstream ss;
    ss << "Bit vector: ";
    for (auto& p: multi_exp_map.point_cloud()) {
      ss <<std::endl << std::bitset<128>(p.bits);
    }
    CLOG(INFO, "test") << ss.str();
    // Get points normal score
    auto scores = multi_exp_map.point_cloud().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
    CLOG(INFO, "test") << "Normal variances: " << std::endl << scores.row(2);
    CLOG(INFO, "test") << "Normal scores: " << std::endl << scores.row(3);
    // clang-format on
  }
}

int main(int argc, char** argv) {
  configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
