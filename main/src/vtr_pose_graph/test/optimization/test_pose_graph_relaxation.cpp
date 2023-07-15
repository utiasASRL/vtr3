// Copyright 2023, Autonomous Space Robotics Lab (ASRL)
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
 * \file test_pose_graph_relaxation.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <random>

#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/evaluator/evaluators.hpp"
#include "vtr_pose_graph/serializable/rc_graph.hpp"
#include "vtr_pose_graph/optimization/pose_graph_relaxation.hpp"
#include "vtr_pose_graph/optimization/pose_graph_optimizer.hpp"

using namespace ::testing;
using namespace vtr::logging;
using namespace vtr::pose_graph;

using VertexId2TransformMap = std::unordered_map<VertexId, EdgeTransform>;
#define ANGLE_NOISE M_PI / 16.0 / 6.0
#define LINEAR_NOISE 0.2 / 6.0

class EvaluatorTestFixture : public Test {
 protected:
  EvaluatorTestFixture(){}

  ~EvaluatorTestFixture() override {}

  void SetUp() override {
    /* Create the following graph
     * R0: 0 --- 1
     *     |     |
     *     |     |
     *     3 --- 2 
     */

    // clang-format off
    
    graph_->addRun();
    graph_->addVertex();
    graph_->addVertex();
    graph_->addVertex();
    graph_->addVertex();
    graph_->addEdge(VertexId(0, 0), VertexId(0, 1), EdgeType::Temporal, false, EdgeTransform(Eigen::Matrix3d::Identity(), Eigen::Vector3d{1.0, 0.0, 0.0}));
    graph_->addEdge(VertexId(0, 1), VertexId(0, 2), EdgeType::Temporal, false,  EdgeTransform(Eigen::Matrix3d::Identity(), Eigen::Vector3d{0.0, 1.0, 0.0}));
    graph_->addEdge(VertexId(0, 2), VertexId(0, 3), EdgeType::Temporal, false,  EdgeTransform(Eigen::Matrix3d::Identity(), Eigen::Vector3d{-1.0, 0.0, 0.0}));
    graph_->addEdge(VertexId(0, 3), VertexId(0, 0), EdgeType::Temporal, false,  EdgeTransform(Eigen::Matrix3d::Identity(), Eigen::Vector3d{0.0, -0.75, 0.0}));

    map_[VertexId(0, 0)] = EdgeTransform(true);
    map_[VertexId(0, 1)] = EdgeTransform(Eigen::Matrix3d::Identity(), Eigen::Vector3d{1.0, 0.0, 0.0});
    map_[VertexId(0, 2)] = EdgeTransform(Eigen::Matrix3d::Identity(), Eigen::Vector3d{1.0, 1.0, 0.0});
    map_[VertexId(0, 3)] = EdgeTransform(Eigen::Matrix3d::Identity(), Eigen::Vector3d{0.0, 1.0, 0.0});
    // clang-format on
  }

  void TearDown() override {}

  BasicGraph::Ptr graph_ = std::make_shared<BasicGraph>();
  VertexId2TransformMap map_;
};

// clang-format off


TEST_F(EvaluatorTestFixture, RelaxationDoesntChangePoseGraph) {
  const auto root_vid = VertexId(0, 0);

  std::vector<EdgeTransform> original_tfs;

  for(auto & tf: map_) {
    CLOG(DEBUG, "test") << tf.second.matrix();
  }

  for(auto itr = graph_->begin(); itr != graph_->end(); itr++) {
    original_tfs.push_back(itr->T());
  }
  

  PoseGraphOptimizer<BasicGraph> optimizer(
      graph_, root_vid, map_);

  // add pose graph relaxation factors
  // default covariance to use
  Eigen::Matrix<double, 6, 6> cov(Eigen::Matrix<double, 6, 6>::Identity());
  cov.topLeftCorner<3, 3>() *= LINEAR_NOISE * LINEAR_NOISE;
  cov.bottomRightCorner<3, 3>() *= ANGLE_NOISE * ANGLE_NOISE;
  auto relaxation_factor =
      std::make_shared<PoseGraphRelaxation<BasicGraph>>(cov);
  optimizer.addFactor(relaxation_factor);

  // udpates the tf map
  using SolverType = steam::DoglegGaussNewtonSolver;
  optimizer.optimize<SolverType>();

  auto itr = graph_->begin();
  for(int i = 0; itr != graph_->end(); itr++) { 
    CLOG(INFO, "test") << itr->T();

    ASSERT_EQ(itr->T().matrix(), original_tfs.at(i++).matrix());
  }

  CLOG(DEBUG, "test") << "After optimization";
  for(auto & tf: map_) {
    CLOG(DEBUG, "test") << tf.second.matrix();
  }

}

// clang-format on

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
