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
 * \file test_id.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/id/id.hpp"

using namespace vtr::logging;
using namespace vtr::pose_graph;

#define LOWER(a) uint32_t(a & 0x00000000FFFFFFFF)
#define UPPER(a) uint32_t(a >> 32)
#define COMBINE(a, b) (uint64_t(a) << 32) | uint64_t(b)

TEST(PoseGraph, vertex_id) {
  /// Vertex id contains a major id and a minor id. Major id corresponds to the
  /// run/experience this vertex belongs to. Major and minor id are both of type
  /// uint32, so that they can be combined as an uint64.

  /// constructors
  VertexId v;  /// default constructor, to invalid.
  EXPECT_FALSE(v.isValid());

  VertexId v0((CombinedIdType)0);  /// combined id (majorId: 0, minorId: 0)
  VertexId v1(1, 10);              /// (majorId: 1, minorid: 10)
  VertexId v2(COMBINE(1, 11));     /// combined id (majorId: 1, minorId: 11)
  EXPECT_TRUE(v0.isValid() && v1.isValid() && v2.isValid());

  // clang-format off
  VertexId v3(2, 10); EXPECT_TRUE((v3.majorId() == 2 && v3.minorId() == 10));
  VertexId v4(2, 11); EXPECT_TRUE((v4.majorId() == 2 && v4.minorId() == 11));
  VertexId v5(2, 9); EXPECT_TRUE((v5.majorId() == 2 && v5.minorId() == 9));
  VertexId v6(3, 9); EXPECT_TRUE((v6.majorId() == 3 && v6.minorId() == 9));
  VertexId v7(1, 20); EXPECT_TRUE((v7.majorId() == 1 && v7.minorId() == 20));
  // clang-format on

  /// IO stream
  LOG(INFO) << "Vertices: " << v << ", " << v0 << ", " << v1 << ", " << v2
            << ", " << v3 << ", " << v4 << ", " << v5 << ", " << v6 << ", "
            << v7;

  /// comparison operator (compare major id first then minor id)
  EXPECT_TRUE(v1 < v2);
  EXPECT_TRUE(v1 < v3);
  EXPECT_TRUE(v1 < v4);
  EXPECT_TRUE(v1 < v5);
  EXPECT_FALSE(v6 == v5);

  /// incremental operator (only on minor id. does not affect major id)
  v5++;
  EXPECT_EQ(v5.minorId(), BaseIdType(10));
  ++v5;
  EXPECT_EQ(v5.minorId(), BaseIdType(11));
  v5--;
  EXPECT_EQ(v5.minorId(), BaseIdType(10));
  --v5;
  EXPECT_EQ(v5.minorId(), BaseIdType(9));

  /// copy constructor and assignment
  VertexId v8(v5);
  EXPECT_EQ(v8, v5);

  VertexId v9(++v5);
  EXPECT_EQ(v9.minorId(), BaseIdType(10));

  v7 = v6;
  EXPECT_EQ(v7.majorId(), BaseIdType(3));
  EXPECT_EQ(v7.minorId(), BaseIdType(9));
}

TEST(PoseGraph, edge_id) {
  /// constructors
  EdgeId e;  /// default constructor, to invalid and undefined type
  EXPECT_FALSE(e.isValid());

  // from a paired combined id and type - note this does not guarantee the edge
  // is from a higher run to a lower run
  EdgeId e0(COMBINE(1, 10), COMBINE(2, 9));
  // from two combined ids and type - note this guarantees the edge is from a
  // higher run to a lower run
  EdgeId e1(COMBINE(2, 9), COMBINE(1, 10));

  EdgeId e2(COMBINE(0, 1), COMBINE(3, 5));
  EdgeId e3(COMBINE(1, 2), COMBINE(0, 4));
  EdgeId e4(COMBINE(2, 4), COMBINE(1, 3));
  EdgeId e5(COMBINE(3, 8), COMBINE(2, 7));
  EdgeId e6(COMBINE(3, 8), COMBINE(2, 7));

  // IO stream
  LOG(INFO) << "Edges: " << e0 << ", " << e1 << ", " << e2 << ", " << e3 << ", "
            << e4 << ", " << e5 << ", " << e6;

  /// comparison operator "type" then "to" then "from"
  EXPECT_TRUE(e2 < e1);
  EXPECT_TRUE(e2 < e3);
  EXPECT_TRUE(e2 < e4);
  EXPECT_TRUE(e2 < e5);
  EXPECT_TRUE(e5 == e6);

  EdgeId e7(e6);  // increment "to" minor id
  EXPECT_EQ(e7.id1(), VertexId(2, 7));
  EXPECT_EQ(e7.majorId1(), BaseIdType(2));
  EXPECT_EQ(e7.minorId1(), BaseIdType(7));
  EXPECT_EQ(e7.id2(), VertexId(3, 8));
  EXPECT_EQ(e7.majorId2(), BaseIdType(3));
  EXPECT_EQ(e7.minorId2(), BaseIdType(8));

  EdgeId e8 = e6;  // increment "to" minor id
  EXPECT_EQ(e8.id1(), VertexId(2, 7));
  EXPECT_EQ(e8.majorId1(), BaseIdType(2));
  EXPECT_EQ(e8.minorId1(), BaseIdType(7));
  EXPECT_EQ(e8.id2(), VertexId(3, 8));
  EXPECT_EQ(e8.majorId2(), BaseIdType(3));
  EXPECT_EQ(e8.minorId2(), BaseIdType(8));
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}