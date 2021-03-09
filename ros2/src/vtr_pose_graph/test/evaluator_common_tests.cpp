#include <gtest/gtest.h>

#include <iostream>
#include <random>

#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/evaluator/common.hpp>
#include <vtr_pose_graph/index/graph.hpp>

using namespace vtr::pose_graph;

using SimpleVertex = uint64_t;
using SimpleEdge = std::pair<uint64_t, uint64_t>;

using IntRandType =
    decltype(std::bind(std::uniform_int_distribution<int64_t>{0, 1000},
                       std::mt19937(std::random_device{}())));
using DoubleRandType =
    decltype(std::bind(std::uniform_real_distribution<double>{0.f, 100.f},
                       std::mt19937(std::random_device{}())));

class EvaluatorTest : public ::testing::Test {
 public:
  EvaluatorTest()
      : graph_(new BasicGraph(0)),
        irnd_(std::bind(std::uniform_int_distribution<int64_t>{0, 1000},
                        std::mt19937(std::random_device{}()))),
        drnd_(std::bind(std::uniform_real_distribution<double>{0.f, 100.f},
                        std::mt19937(std::random_device{}()))) {
  }

  ~EvaluatorTest() override {
  }

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

    // Add a graph with 5 runs and 3 vertices per run.
    for (int idx = 0; idx < 5; ++idx) {
      graph_->addRun();
      graph_->addVertex();
      graph_->addVertex();
      graph_->addVertex();
      graph_->addEdge(VertexId(idx, 0), VertexId(idx, 1));
      graph_->addEdge(VertexId(idx, 1), VertexId(idx, 2));
    }
    // Add spatial edges across runs.
    graph_->addEdge(VertexId(1, 1), VertexId(0, 0), Spatial);
    graph_->addEdge(VertexId(2, 2), VertexId(1, 2), Spatial);
    graph_->addEdge(VertexId(3, 1), VertexId(2, 1), Spatial);
    graph_->addEdge(VertexId(4, 2), VertexId(3, 2), Spatial);

    // set the edge's transform to something special;
    auto edge_map = graph_->edges();
    for (auto itr = edge_map->begin(); itr != edge_map->end(); ++itr) {
      Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
      transform(0, 3) = itr->second->id().majorId();
      transform(1, 3) = itr->second->id().minorId();
      transform(2, 3) = itr->second->id().type();
      itr->second->setTransform(lgmath::se3::Transformation(transform));
    }
  }
  void TearDown() override {
  }

 protected:
  BasicGraph::UniquePtr graph_;
  IntRandType irnd_;
  DoubleRandType drnd_;
};

TEST_F(EvaluatorTest, DistanceEvaluator) {
  using namespace eval::Weight;

  // SECTION("Direct", "[direct]")
  {
    auto eval = Distance<BasicGraph>::Direct::MakeShared();

    eval->setGraph(graph_.get());

    for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
         ++it) {
      EXPECT_EQ(eval->at(it->first), 0);
      EXPECT_EQ(eval->at(it->second->id()), 0);

      EXPECT_EQ((*eval)[it->first], 0);
      EXPECT_EQ((*eval)[it->second->id()], 0);
    }
    for (auto it = graph_->edges()->begin(); it != graph_->edges()->end();
         ++it) {
      auto tmpId = it->second->id();
      double norm =
          std::sqrt(std::pow(tmpId.majorId(), 2) +
                    std::pow(tmpId.minorId(), 2) + std::pow(tmpId.idx(), 2));

      EXPECT_EQ(eval->at(it->first), norm);
      EXPECT_EQ(eval->at(it->second->id()), norm);

      EXPECT_EQ((*eval)[it->first], norm);
      EXPECT_EQ((*eval)[it->second->id()], norm);
    }
  }

  // SECTION("Caching", "[caching]")
  {
    auto eval = Distance<BasicGraph>::Caching::MakeShared();

    eval->setGraph(graph_.get());

    for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
         ++it) {
      // Const access before computation should throw
      EXPECT_THROW(eval->at(it->first), std::out_of_range);
      EXPECT_THROW(eval->at(it->second->id()), std::out_of_range);

      // Non-const access before computation should compute and cache
      EXPECT_EQ((*eval)[it->first], 0);
      EXPECT_EQ((*eval)[it->second->id()], 0);

      // Const access after computation should succeed
      EXPECT_EQ(eval->at(it->first), 0);
      EXPECT_EQ(eval->at(it->second->id()), 0);
    }
    for (auto it = graph_->edges()->begin(); it != graph_->edges()->end();
         ++it) {
      auto tmpId = it->second->id();
      double norm =
          std::sqrt(std::pow(tmpId.majorId(), 2) +
                    std::pow(tmpId.minorId(), 2) + std::pow(tmpId.idx(), 2));

      // Const access before computation should throw
      EXPECT_THROW(eval->at(it->first), std::out_of_range);
      EXPECT_THROW(eval->at(it->second->id()), std::out_of_range);

      // Non-const access before computation should compute and cache
      EXPECT_EQ((*eval)[it->first], norm);
      EXPECT_EQ((*eval)[it->second->id()], norm);

      // Const access after computation should succeed
      EXPECT_EQ(eval->at(it->first), norm);
      EXPECT_EQ(eval->at(it->second->id()), norm);
    }
  }

  // SECTION("Windowed", "[windowed]")
  {
    auto eval = Distance<BasicGraph>::Windowed::MakeShared(5);

    eval->setGraph(graph_.get());

    for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
         ++it) {
      // Const access before computation should throw
      EXPECT_THROW(eval->at(it->first), std::out_of_range);
      EXPECT_THROW(eval->at(it->second->id()), std::out_of_range);

      // Non-const access before computation should compute and cache
      EXPECT_EQ((*eval)[it->first], 0);
      EXPECT_EQ((*eval)[it->second->id()], 0);

      // Const access after computation should succeed
      EXPECT_EQ(eval->at(it->first), 0);
      EXPECT_EQ(eval->at(it->second->id()), 0);
    }
    for (auto it = graph_->edges()->begin(); it != graph_->edges()->end();
         ++it) {
      auto tmpId = it->second->id();
      double norm =
          std::sqrt(std::pow(tmpId.majorId(), 2) +
                    std::pow(tmpId.minorId(), 2) + std::pow(tmpId.idx(), 2));

      // Const access before computation should throw
      EXPECT_THROW(eval->at(it->first), std::out_of_range);
      EXPECT_THROW(eval->at(it->second->id()), std::out_of_range);

      // Non-const access before computation should compute and cache
      EXPECT_EQ((*eval)[it->first], norm);
      EXPECT_EQ((*eval)[it->second->id()], norm);

      // Const access after computation should succeed
      EXPECT_EQ(eval->at(it->first), norm);
      EXPECT_EQ(eval->at(it->second->id()), norm);
    }

    for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
         ++it) {
      // Const access should fail as the vertex will be out of the cache by now
      EXPECT_THROW(eval->at(it->first), std::out_of_range);
      EXPECT_THROW(eval->at(it->second->id()), std::out_of_range);

      // Move the cache forward
      EXPECT_EQ((*eval)[it->first], 0);
    }
    for (auto it = graph_->edges()->begin(); it != graph_->edges()->end();
         ++it) {
      EdgeId tmpId = it->second->id();
      double norm =
          std::sqrt(std::pow(tmpId.majorId(), 2) +
                    std::pow(tmpId.minorId(), 2) + std::pow(tmpId.idx(), 2));

      // Const access should fail as the vertex will be out of the cache by now
      EXPECT_THROW(eval->at(it->first), std::out_of_range);
      EXPECT_THROW(eval->at(it->second->id()), std::out_of_range);

      // Move the cache forward
      EXPECT_EQ((*eval)[it->first], norm);
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
