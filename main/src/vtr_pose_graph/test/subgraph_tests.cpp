#include <gtest/gtest.h>

#include <iostream>

#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/evaluator/common.hpp>
#include <vtr_pose_graph/evaluator/mask_evaluator.hpp>
#include <vtr_pose_graph/evaluator/weight_evaluator.hpp>
/// #include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_pose_graph/index/graph.hpp>

using namespace vtr::pose_graph;
/// using namespace vtr::common;

using SimpleVertex = uint64_t;
using SimpleEdge = std::pair<uint64_t, uint64_t>;

class SubGraphTest : public ::testing::Test {
 public:
  SubGraphTest() : graph_(new BasicGraph(0)) {
  }

  ~SubGraphTest() {
  }

  void SetUp() override {
    /* Create the following graph
     * R0: 0 --- 1 --- 2
     *       \
     *        \
     *         \
     * R1: 0 --- 1 --- 2 -- 250
     *                 |
     * R2: 0 --- 1 --- 2 -- 250
     *           |
     * R3: 0 --- 1 --- 2 -- 250
     *                 |
     * R4: 0 --- 1 --- 2 -- 250
     */

    // Add a graph with 5 runs and 3 vertices per run.
    /// graph_.reset(new RCGraph("/tmp/graph_test/graph_index", 0));
    /// for (int idx = 0; idx < 5; ++idx) {
    ///   // Create the robochunk directories
    ///   graph_->addRun(0);
    ///   graph_->addVertex(robochunk::std_msgs::TimeStamp());
    ///   for (int vidx = 1; vidx < 250; ++vidx) {
    ///     graph_->addVertex(robochunk::std_msgs::TimeStamp());
    ///     auto edge =
    ///         graph_->addEdge(VertexId(idx, vidx - 1), VertexId(idx, vidx));
    ///     if (idx == 0) edge->setManual(true);
    ///   }
    /// }
    // Add a graph with 5 runs and 3 vertices per run.
    for (int idx = 0; idx < 5; ++idx) {
      // Create the robochunk directories
      graph_->addRun();
      graph_->addVertex();
      for (int vidx = 1; vidx < 250; ++vidx) {
        graph_->addVertex();
        auto edge =
            graph_->addEdge(VertexId(idx, vidx - 1), VertexId(idx, vidx));
        if (idx == 0)
          edge->setManual(true);
      }
    }
    // Add spatial edge across runs.
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
  /// std::unique_ptr<RCGraph> graph_;
  BasicGraph::Ptr graph_;
};

TEST_F(SubGraphTest, SubGraphFromVertexList) {
  std::vector<VertexId> vertices;
  vertices.push_back(VertexId(0, 0));
  vertices.push_back(VertexId(0, 1));
  vertices.push_back(VertexId(0, 2));
  auto sub_graph = graph_->getSubgraph(vertices);
  int count = 0;
  for (auto itr = sub_graph->beginEdge(); itr != sub_graph->endEdge(); ++itr)
    count++;
  EXPECT_EQ(count, 2);
}

TEST_F(SubGraphTest, SubGraphPrivilgedMask) {
  using PrivilegedEvaluator = eval::Mask::Privileged<BasicGraph>::Direct;
  PrivilegedEvaluator::Ptr evaluator(new PrivilegedEvaluator());
  evaluator->setGraph(graph_.get());
  auto sub_graph = graph_->getSubgraph(VertexId(0, 0), evaluator);

  int count = 0;
  for (auto itr = sub_graph->beginEdge(); itr != sub_graph->endEdge(); ++itr) {
    count++;
  }
  EXPECT_EQ(count, 249);
}

TEST_F(SubGraphTest, SubGraphPrivilegedPath) {
  // Get only the privileged edges.
  using PrivilegedEvaluator = eval::Mask::Privileged<BasicGraph>::Direct;
  PrivilegedEvaluator::Ptr evaluator(new PrivilegedEvaluator());
  evaluator->setGraph(graph_.get());
  auto sub_graph = graph_->getSubgraph(VertexId(0, 0), evaluator);

  // extract a path from the sub graph
  auto path = sub_graph->breadthFirstSearch(VertexId(0, 15), VertexId(0, 25));
  // start at the begining.
  auto itr = path->begin(VertexId(0, 15));
  EXPECT_EQ(itr->v()->id(), VertexId(0, 15));
  itr++;
  int count = 0;
  for (; itr != path->end(); ++itr) {
    count++;
    std::cout << itr->e()->id() << std::endl;
  }
  EXPECT_EQ(count, 10);
}

TEST_F(SubGraphTest, SubGraphUnion) {
  std::vector<VertexId> v1, v2;
  v1.push_back(VertexId(0, 0));
  v1.push_back(VertexId(0, 1));
  v1.push_back(VertexId(0, 2));

  v2.push_back(VertexId(0, 2));
  v2.push_back(VertexId(0, 3));
  v2.push_back(VertexId(0, 4));

  auto sub_graph1 = graph_->getSubgraph(v1);
  auto sub_graph2 = graph_->getSubgraph(v2);
  auto sub_graph = sub_graph1->setUnion(sub_graph2);

  EXPECT_EQ(sub_graph->numberOfVertices(), (unsigned)5);
  EXPECT_EQ(sub_graph->numberOfEdges(), (unsigned)4);

  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(sub_graph->contains(VertexId(0, i)));
  }

  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(sub_graph->contains(VertexId(0, i), VertexId(0, i + 1)));
  }
}

TEST_F(SubGraphTest, InducedSubGraphs) {
  std::vector<VertexId> v1, v2;
  v1.push_back(VertexId(0, 0));
  v1.push_back(VertexId(0, 1));
  v1.push_back(VertexId(0, 2));

  v2.push_back(VertexId(1, 0));
  v2.push_back(VertexId(1, 1));
  v2.push_back(VertexId(1, 2));

  auto sub_graph1 = graph_->getSubgraph(v1);
  auto sub_graph2 = graph_->getSubgraph(v2);
  auto sub_graph = sub_graph1->setUnion(sub_graph2);

  EXPECT_EQ(sub_graph->numberOfVertices(), (unsigned)6);
  EXPECT_EQ(sub_graph->numberOfEdges(), (unsigned)4);
  EXPECT_FALSE(sub_graph->contains(VertexId(0, 0), VertexId(1, 1)));

  auto ind_graph = graph_->induced(*sub_graph);

  EXPECT_EQ(ind_graph->numberOfVertices(), (unsigned)6);
  EXPECT_EQ(ind_graph->numberOfEdges(), (unsigned)5);
  EXPECT_TRUE(ind_graph->contains(VertexId(0, 0), VertexId(1, 1)));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}