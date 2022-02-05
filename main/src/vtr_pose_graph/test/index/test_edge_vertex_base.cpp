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
 * \file test_edge_vertex.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/index/edge_base.hpp"
#include "vtr_pose_graph/index/vertex_base.hpp"

using namespace vtr::logging;
using namespace vtr::pose_graph;

TEST(PoseGraph, edge_base_tests) {
  /// constructors
  // follow the convention of from a higher run to a lower run, so the edge is
  // reversed
  EdgeBase edge(VertexId(3, 5), VertexId(1, 10), EdgeType::Temporal, false,
                EdgeTransform(true));
  edge.setTransform(EdgeTransform(true));

  /// getters
  EXPECT_EQ(edge.id(), EdgeId(VertexId(1, 10), VertexId(3, 5)));
  EXPECT_EQ(edge.type(), EdgeType::Temporal);
  EXPECT_EQ(edge.idx(), (size_t)EdgeType::Temporal);
  EXPECT_EQ(edge.from(), VertexId(3, 5));
  EXPECT_EQ(edge.to(), VertexId(1, 10));
  EXPECT_FALSE(edge.isManual());
  EXPECT_TRUE(edge.isAutonomous());
  EXPECT_TRUE(edge.isTemporal());
  EXPECT_FALSE(edge.isSpatial());

  CLOG(INFO, "test") << edge << std::endl << edge.T();  // check output
}

TEST(PoseGraph, vertex_base_tests) {
  /// constructors
  VertexBase vertex(VertexId(1, 1));

  /// getters
  EXPECT_EQ(vertex.id(), VertexId(1, 1));

  CLOG(INFO, "test") << vertex;  // check output
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}