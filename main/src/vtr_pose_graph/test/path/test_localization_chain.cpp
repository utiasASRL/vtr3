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
 * \file test_localization_chain_.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/evaluator/evaluators.hpp"
#include "vtr_pose_graph/index/graph.hpp"
#include "vtr_pose_graph/path/localization_chain.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::pose_graph;

void print(const LocalizationChain<BasicGraph>& chain) {
  CLOG(INFO, "test") << "trunk sid: " << chain.trunkSequenceId()
                     << ", trunk vid: " << chain.trunkVertexId();
  CLOG(INFO, "test") << "T_branch_trunk: "
                     << chain.T_branch_trunk().vec().transpose();
  CLOG(INFO, "test") << "branch sid: " << chain.branchSequenceId()
                     << ", branch vid: " << chain.branchVertexId();
  CLOG(INFO, "test") << "T_twig_branch: "
                     << chain.T_twig_branch().vec().transpose();
  CLOG(INFO, "test") << "twig vid: " << chain.twigVertexId();
  CLOG(INFO, "test") << "T_petiole_twig: "
                     << chain.T_petiole_twig().vec().transpose();
  CLOG(INFO, "test") << "petiole vid: " << chain.petioleVertexId();
  CLOG(INFO, "test") << "T_leaf_petiole: "
                     << chain.T_leaf_petiole().vec().transpose();
}

class ChainTest : public Test {
 public:
  ChainTest() : graph_(new BasicGraph()), chain_(graph_) {}
  ~ChainTest() override {}

  void SetUp() override {
    /**
     * Create the following graph, no need to create spatial edges
     * R0: 0 --- 1 --- 2 --- ...
     * R1: 0 --- 1 --- 2 --- ...
     * R0 is the teach privilege, R1 is the repeat non-privilege
     */
    // R0
    graph_->addRun();
    graph_->addVertex();
    for (unsigned i = 0; i < num_vertices_ - 1; ++i) {
      graph_->addVertex();
      Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
      transform(2, 3) = -1;
      EdgeTransform edge_transform(transform);
      edge_transform.setZeroCovariance();
      graph_->addEdge(VertexId(0, i), VertexId(0, i + 1), EdgeType::Temporal,
                      true, edge_transform);
    }

    // R1
    graph_->addRun();
    graph_->addVertex();
    for (unsigned i = 0; i < num_vertices_ - 1; ++i) {
      graph_->addVertex();
      Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
      transform(2, 3) = -0.4;
      EdgeTransform edge_transform(transform);
      edge_transform.setZeroCovariance();
      graph_->addEdge(VertexId(1, i), VertexId(1, i + 1), EdgeType::Temporal,
                      false, edge_transform);
    }

    using PrivEvaluator = eval::mask::privileged::CachedEval<BasicGraph>;
    auto eval = std::make_shared<PrivEvaluator>(*graph_);
    auto path = graph_->getSubgraph(0ul, eval);
    VertexId::Vector sequence;
    for (auto it = path->begin(0ul); it != path->end(); ++it)
      sequence.push_back(it->v()->id());

    // initialize the localization chain
    chain_.setSequence(sequence);
    chain_.expand();
    chain_.setPetiole(VertexId(1, 0));
    auto live_id = chain_.petioleVertexId();
    auto map_id = chain_.trunkVertexId();
    auto map_sid = chain_.trunkSequenceId();
    auto T_petiole_trunk = chain_.T_petiole_trunk();
    chain_.updateBranchToTwigTransform(live_id, map_id, map_sid,
                                       T_petiole_trunk, true, false);
    print(chain_);
  }

  void TearDown() override {}

 protected:
  size_t num_vertices_ = 20;
  BasicGraph::Ptr graph_;
  LocalizationChain<BasicGraph> chain_;
};

TEST_F(ChainTest, simulate_localization_every_vertex) {
  // assume we have advanced multiple vertices and update trunk
  chain_.setPetiole(VertexId(1, 4));
  chain_.updatePetioleToLeafTransform(EdgeTransform(true), true, false);
  print(chain_);

  {
    auto live_id = chain_.petioleVertexId();
    auto map_id = chain_.trunkVertexId();
    auto map_sid = chain_.trunkSequenceId();
    auto T_petiole_trunk = chain_.T_petiole_trunk();
    chain_.updateBranchToTwigTransform(live_id, map_id, map_sid,
                                       T_petiole_trunk, true, false);
    print(chain_);
  }

  // advanced another vertex
  chain_.setPetiole(VertexId(1, 5));
  chain_.updatePetioleToLeafTransform(EdgeTransform(true), true, false);
  print(chain_);

  {
    auto live_id = chain_.petioleVertexId();
    auto map_id = chain_.trunkVertexId();
    auto map_sid = chain_.trunkSequenceId();
    auto T_petiole_trunk = chain_.T_petiole_trunk();
    chain_.updateBranchToTwigTransform(live_id, map_id, map_sid,
                                       T_petiole_trunk, true, false);
    print(chain_);
  }
}

TEST_F(ChainTest, simulate_localization_every_frame) {
  // assume we have advanced multiple vertices and update trunk
  chain_.setPetiole(VertexId(1, 4));
  chain_.updatePetioleToLeafTransform(EdgeTransform(true), true, false);
  print(chain_);

  {
    auto live_id = chain_.petioleVertexId();
    auto map_id = chain_.trunkVertexId();
    auto map_sid = chain_.trunkSequenceId();
    auto T_petiole_trunk = chain_.T_petiole_trunk();
    chain_.updateBranchToTwigTransform(live_id, map_id, map_sid,
                                       T_petiole_trunk, true, false);
    print(chain_);
  }

  // advanced another vertex
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform(2, 3) = -3;
  EdgeTransform edge_transform(transform);
  edge_transform.setZeroCovariance();
  chain_.updatePetioleToLeafTransform(edge_transform, true, false);

  {
    auto live_id = chain_.petioleVertexId();
    auto map_id = chain_.trunkVertexId();
    auto map_sid = chain_.trunkSequenceId();
    auto T_petiole_trunk = chain_.T_petiole_trunk();
    chain_.updateBranchToTwigTransform(live_id, map_id, map_sid,
                                       T_petiole_trunk, true, false);
    print(chain_);
  }
}

TEST_F(ChainTest, simulate_localization_skipped_frames) {
  // assume we have advanced multiple vertices and update trunk
  chain_.setPetiole(VertexId(1, 4));
  chain_.updatePetioleToLeafTransform(EdgeTransform(true), true, false);
  print(chain_);

  // store localization information
  auto live_id = chain_.petioleVertexId();
  auto map_id = chain_.trunkVertexId();
  auto map_sid = chain_.trunkSequenceId();
  auto T_petiole_trunk = chain_.T_petiole_trunk();

  // advanced another vertex
  chain_.setPetiole(VertexId(1, 10));
  chain_.updatePetioleToLeafTransform(EdgeTransform(true), true, false);
  print(chain_);

  // localize again
  chain_.updateBranchToTwigTransform(live_id, map_id, map_sid, T_petiole_trunk,
                                     true, false);
  print(chain_);
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
