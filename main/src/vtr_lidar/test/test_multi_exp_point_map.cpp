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

#include "vtr_lidar/data_structures/pointmap.hpp"
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
      p.normal_score = i; p.icp_score = 0;
      // clang-format on
      point_cloud.push_back(p);
    }
    point_map.update(point_cloud);
    point_map.T_vertex_map() = tactic::EdgeTransform(true);
    point_map.vertex_id() = tactic::VertexId(1, 1);
    // fake dynamic points removal
    for (auto& p : point_map.point_map()) p.icp_score = 0;
    multi_exp_map.update(point_map);

    EXPECT_EQ(multi_exp_map.size(), (size_t)3);
    EXPECT_EQ(multi_exp_map.experiences().size(), (size_t)1);

    // clang-format off
    // Get points cartesian coordinates as eigen map
    auto points_cart = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
    LOG(INFO) << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
    // Get points normal vector as eigen map
    auto points_normal = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
    LOG(INFO) << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
    // Get points polar coordinates as eigen map
    std::stringstream ss;
    ss << "Bit vector: ";
    for (auto& p: multi_exp_map.point_map()) {
      ss <<std::endl << std::bitset<128>(p.bits);
    }
    LOG(INFO) << ss.str();
    // Get points normal score
    auto normal_score = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
    LOG(INFO) << "Normal scores: " << std::endl << normal_score.row(2);
    LOG(INFO) << "ICP scores: " << std::endl << normal_score.row(3);
    // clang-format on
  }

  // Update with exact same points should only update icp score
  {
    PointMap<PointWithInfo> point_map(0.1,
                                      PointMap<PointWithInfo>::DYNAMIC_REMOVED);
    pcl::PointCloud<PointWithInfo> point_cloud;
    for (int i = 0; i < 3; i++) {
      PointWithInfo p;
      // clang-format off
      p.x = p.y = p.z = i;
      p.normal_x = p.normal_y = p.normal_z = i;
      p.normal_score = i; p.icp_score = 0;
      // clang-format on
      point_cloud.push_back(p);
    }
    point_map.update(point_cloud);
    point_map.T_vertex_map() = tactic::EdgeTransform(true);
    point_map.vertex_id() = tactic::VertexId(1, 1);
    // fake dynamic points removal
    for (auto& p : point_map.point_map()) p.icp_score = 0;
    multi_exp_map.update(point_map);

    EXPECT_EQ(multi_exp_map.size(), (size_t)3);
    EXPECT_EQ(multi_exp_map.experiences().size(), (size_t)2);

    // clang-format off
    // Get points cartesian coordinates as eigen map
    auto points_cart = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
    LOG(INFO) << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
    // Get points normal vector as eigen map
    auto points_normal = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
    LOG(INFO) << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
    // Get points polar coordinates as eigen map
    std::stringstream ss;
    ss << "Bit vector: ";
    for (auto& p: multi_exp_map.point_map()) {
      ss <<std::endl << std::bitset<128>(p.bits);
    }
    LOG(INFO) << ss.str();
    // Get points normal score
    auto normal_score = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
    LOG(INFO) << "Normal scores: " << std::endl << normal_score.row(2);
    LOG(INFO) << "ICP scores: " << std::endl << normal_score.row(3);
    // clang-format on
  }

  // Update with completely different points should add more points while update
  // icp score
  {
    PointMap<PointWithInfo> point_map(0.1,
                                      PointMap<PointWithInfo>::DYNAMIC_REMOVED);
    pcl::PointCloud<PointWithInfo> point_cloud;
    for (int i = 3; i < 6; i++) {
      PointWithInfo p;
      // clang-format off
      p.x = p.y = p.z = i;
      p.normal_x = p.normal_y = p.normal_z = i;
      p.normal_score = i; p.icp_score = 0;
      // clang-format on
      point_cloud.push_back(p);
    }
    point_map.update(point_cloud);
    point_map.T_vertex_map() = tactic::EdgeTransform(true);
    point_map.vertex_id() = tactic::VertexId(1, 1);
    // fake dynamic points removal
    for (auto& p : point_map.point_map()) p.icp_score = 0;
    multi_exp_map.update(point_map);

    EXPECT_EQ(multi_exp_map.size(), (size_t)6);
    EXPECT_EQ(multi_exp_map.experiences().size(), (size_t)2);

    // clang-format off
    // Get points cartesian coordinates as eigen map
    auto points_cart = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
    LOG(INFO) << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
    // Get points normal vector as eigen map
    auto points_normal = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
    LOG(INFO) << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
    // Get points polar coordinates as eigen map
    std::stringstream ss;
    ss << "Bit vector: ";
    for (auto& p: multi_exp_map.point_map()) {
      ss <<std::endl << std::bitset<128>(p.bits);
    }
    LOG(INFO) << ss.str();
    // Get points normal score
    auto normal_score = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
    LOG(INFO) << "Normal scores: " << std::endl << normal_score.row(2);
    LOG(INFO) << "ICP scores: " << std::endl << normal_score.row(3);
    // clang-format on
  }

  // Update with completely different points should add more points while update
  // icp score
  {
    PointMap<PointWithInfo> point_map(0.1,
                                      PointMap<PointWithInfo>::DYNAMIC_REMOVED);
    pcl::PointCloud<PointWithInfo> point_cloud;
    for (int i = 3; i < 6; i++) {
      PointWithInfo p;
      // clang-format off
      p.x = p.y = p.z = i;
      p.normal_x = p.normal_y = p.normal_z = i;
      p.normal_score = i; p.icp_score = 0;
      // clang-format on
      point_cloud.push_back(p);
    }
    point_map.update(point_cloud);
    point_map.T_vertex_map() = tactic::EdgeTransform(true);
    point_map.vertex_id() = tactic::VertexId(1, 1);
    // fake dynamic points removal
    for (auto& p : point_map.point_map()) p.icp_score = 0;
    multi_exp_map.update(point_map);

    EXPECT_EQ(multi_exp_map.size(), (size_t)3);
    EXPECT_EQ(multi_exp_map.experiences().size(), (size_t)2);

    // clang-format off
    // Get points cartesian coordinates as eigen map
    auto points_cart = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
    LOG(INFO) << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
    // Get points normal vector as eigen map
    auto points_normal = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
    LOG(INFO) << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
    // Get points polar coordinates as eigen map
    std::stringstream ss;
    ss << "Bit vector: ";
    for (auto& p: multi_exp_map.point_map()) {
      ss <<std::endl << std::bitset<128>(p.bits);
    }
    LOG(INFO) << ss.str();
    // Get points normal score
    auto normal_score = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
    LOG(INFO) << "Normal scores: " << std::endl << normal_score.row(2);
    LOG(INFO) << "ICP scores: " << std::endl << normal_score.row(3);
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
      p.normal_score = i; p.icp_score = 0;
    // clang-format on
    point_cloud.push_back(p);
  }
  point_map.update(point_cloud);
  point_map.T_vertex_map() = tactic::EdgeTransform(true);
  point_map.vertex_id() = tactic::VertexId(1, 1);
  // fake dynamic points removal
  for (auto& p : point_map.point_map()) p.icp_score = 0;
  multi_exp_map.update(point_map);

  LOG(INFO) << multi_exp_map.size();
  LOG(INFO) << multi_exp_map.T_vertex_map();
  LOG(INFO) << multi_exp_map.vertex_id();
  LOG(INFO) << multi_exp_map.dl();
  LOG(INFO) << multi_exp_map.version();
  LOG(INFO) << multi_exp_map.experiences().size();
  LOG(INFO) << multi_exp_map.experiences()[0];
  {
    // clang-format off
    // Get points cartesian coordinates as eigen map
    auto points_cart = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
    LOG(INFO) << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
    // Get points normal vector as eigen map
    auto points_normal = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
    LOG(INFO) << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
    // Get points polar coordinates as eigen map
    std::stringstream ss;
    ss << "Bit vector: ";
    for (auto& p: multi_exp_map.point_map()) {
      ss <<std::endl << std::bitset<128>(p.bits);
    }
    LOG(INFO) << ss.str();
    // Get points normal score
    auto normal_score = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
    LOG(INFO) << "Normal scores: " << std::endl << normal_score.row(2);
    LOG(INFO) << "ICP scores: " << std::endl << normal_score.row(3);
    // clang-format on
  }
  const auto msg = multi_exp_map.toStorable();
  auto multi_exp_map2 = MultiExpPointMap<PointWithInfo>::fromStorable(msg);

  LOG(INFO) << multi_exp_map2->size();
  LOG(INFO) << multi_exp_map2->T_vertex_map();
  LOG(INFO) << multi_exp_map2->vertex_id();
  LOG(INFO) << multi_exp_map2->dl();
  LOG(INFO) << multi_exp_map2->version();
  LOG(INFO) << multi_exp_map2->experiences().size();
  LOG(INFO) << multi_exp_map2->experiences()[0];
  {
    // clang-format off
    // Get points cartesian coordinates as eigen map
    auto points_cart = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::cartesian_offset());
    LOG(INFO) << "Cartesian coordinates: " << "<" << points_cart.rows() << "," << points_cart.cols() << ">" << std::endl << points_cart;
    // Get points normal vector as eigen map
    auto points_normal = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::normal_offset());
    LOG(INFO) << "Normal vector: " << "<" << points_normal.rows() << "," << points_normal.cols() << ">" << std::endl << points_normal;
    // Get points polar coordinates as eigen map
    std::stringstream ss;
    ss << "Bit vector: ";
    for (auto& p: multi_exp_map.point_map()) {
      ss <<std::endl << std::bitset<128>(p.bits);
    }
    LOG(INFO) << ss.str();
    // Get points normal score
    auto normal_score = multi_exp_map.point_map().getMatrixXfMap(/* dim */ 4, /* stride */ PointWithInfo::size(), /* offset */ PointWithInfo::flex2_offset());
    LOG(INFO) << "Normal scores: " << std::endl << normal_score.row(2);
    LOG(INFO) << "ICP scores: " << std::endl << normal_score.row(3);
    // clang-format on
  }
}

int main(int argc, char** argv) {
  configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
