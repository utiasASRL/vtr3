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
 * \file evaluator_common_tests.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/evaluator/evaluators.hpp"
#include "vtr_pose_graph/index/graph.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::pose_graph;

EdgeTransform trivialTransform(const VertexId& from, const VertexId& to) {
  Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
  EdgeId eid(from, to);
  mat(0, 3) = eid.majorId1();
  mat(1, 3) = eid.majorId2();
  mat(2, 3) = eid.minorId1();
  EdgeTransform tf(mat);
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity() * 3;
  tf.setCovariance(cov);
  return tf;
}

class EvaluatorTestFixture : public Test {
 public:
  EvaluatorTestFixture() {}

  ~EvaluatorTestFixture() override {}

  void SetUp() override {
    /* Create the following graph
     * R0: 0 --- 1 --- 2
     *       \
     *        \
     *         \
     * R1: 0 --- 1 --- 2
     *                 |
     * R2: 0 --- 1 --- 2
     *           |
     * R3: 0 --- 1 --- 2
     *                 |
     * R4: 0 --- 1 --- 2
     */

    // clang-format off
    for (int idx = 0; idx < 5; ++idx) {
      graph_->addRun();
      graph_->addVertex();
      graph_->addVertex();
      graph_->addVertex();
      graph_->addEdge(VertexId(idx, 0), VertexId(idx, 1), EdgeType::Temporal, false, trivialTransform(VertexId(idx, 0), VertexId(idx, 1)));
      graph_->addEdge(VertexId(idx, 1), VertexId(idx, 2), EdgeType::Temporal, false, trivialTransform(VertexId(idx, 1), VertexId(idx, 2)));
    }
    graph_->addEdge(VertexId(1, 1), VertexId(0, 0), EdgeType::Spatial, false, trivialTransform(VertexId(1, 1), VertexId(0, 0)));
    graph_->addEdge(VertexId(2, 2), VertexId(1, 2), EdgeType::Spatial, false, trivialTransform(VertexId(2, 2), VertexId(1, 2)));
    graph_->addEdge(VertexId(3, 1), VertexId(2, 1), EdgeType::Spatial, false, trivialTransform(VertexId(3, 1), VertexId(2, 1)));
    graph_->addEdge(VertexId(4, 2), VertexId(3, 2), EdgeType::Spatial, false, trivialTransform(VertexId(4, 2), VertexId(3, 2)));
    // clang-format on
  }

  void TearDown() override {}

  BasicGraph::Ptr graph_ = std::make_shared<BasicGraph>();
};

// clang-format off
TEST_F(EvaluatorTestFixture, DistanceEvaluator) {
  using namespace eval::weight;
  // SECTION("Direct", "[direct]")
  {
    auto eval = std::make_shared<distance::Eval<BasicGraph>>(*graph_);

    for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
      EXPECT_EQ((*eval)[it->id()], 0);
    }

    for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
      auto tmpId = it->id();
      double norm = std::sqrt(std::pow(tmpId.majorId1(), 2) + std::pow(tmpId.majorId2(), 2) + std::pow(tmpId.minorId1(), 2));
      EXPECT_EQ((*eval)[it->id()], norm);
    }
  }

  // SECTION("Caching", "[caching]")
  {
    auto eval = std::make_shared<distance::CachedEval<BasicGraph>>(*graph_);

    for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
      EXPECT_EQ((*eval)[it->id()], 0);
    }

    for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
      auto tmpId = it->id();
      double norm = std::sqrt(std::pow(tmpId.majorId1(), 2) + std::pow(tmpId.majorId2(), 2) + std::pow(tmpId.minorId1(), 2));
      EXPECT_EQ((*eval)[it->id()], norm);
    }
  }

  // SECTION("Windowed", "[windowed]")
  {
    auto eval = std::make_shared<distance::WindowedEval<BasicGraph>>(*graph_, 5);

    for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
      EXPECT_EQ((*eval)[it->id()], 0);
    }

    for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
      auto tmpId = it->id();
      double norm = std::sqrt(std::pow(tmpId.majorId1(), 2) + std::pow(tmpId.majorId2(), 2) + std::pow(tmpId.minorId1(), 2));
      EXPECT_EQ((*eval)[it->id()], norm);
    }

  }
}
// clang-format on
int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
