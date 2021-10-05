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
 * \brief edge base and vertex base tests
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/index/edge_base.hpp>
#include <vtr_pose_graph/index/vertex_base.hpp>

using namespace vtr::logging;
using namespace vtr::pose_graph;

TEST(PoseGraph, edge_base_tests) {
  /// constructors
  // follow the convention of from a higher run to a lower run, so the edge is
  // reversed
  EdgeBase edge(VertexId(3, 5), VertexId(1, 10), EdgeId::Type::Temporal, false);
  edge.setTransform(EdgeBase::TransformType(true));

  /// getters
  // clang-format off
  EXPECT_EQ(edge.id(), EdgeId(VertexId(3, 5), VertexId(1, 10), EdgeId::Type::Temporal));
  EXPECT_EQ(edge.simpleId(), EdgeBase::SimpleIdType(COMBINE(1, 10), COMBINE(3, 5)));
  EXPECT_EQ(edge.type(), EdgeId::Type::Temporal);
  EXPECT_EQ(edge.idx(), (size_t)EdgeId::Type::Temporal);
  EXPECT_EQ(edge.from(), VertexId(3, 5));
  EXPECT_EQ(edge.to(), VertexId(1, 10));
  EXPECT_FALSE(edge.isManual());
  EXPECT_TRUE(edge.isAutonomous());
  EXPECT_TRUE(edge.isTemporal());
  EXPECT_FALSE(edge.isSpatial());
  EXPECT_TRUE(edge.isIncident(VertexId(1, 10)));
  LOG(INFO) << edge.T(); // check output
  // clang-format on
}

TEST(PoseGraph, vertex_base_tests) {
  /// constructors
  VertexBase vertex(VertexId(1, 1));
  // addEdge are protected functions -> can only call from graph
#if false
  vertex.addEdge(EdgeId(VertexId(3, 5), VertexId(1, 1), EdgeId::Type::Spatial));
  vertex.addEdge(VertexId(1, 2), EdgeId::Type::Spatial);
#endif

  /// getters
  EXPECT_EQ(vertex.id(), VertexId(1, 1));
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}