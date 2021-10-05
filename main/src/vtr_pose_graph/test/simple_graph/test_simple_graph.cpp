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
 * \file test_simple_graph.hpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/simple_graph/simple_graph.hpp>
#include <vtr_pose_graph/simple_graph/simple_iterator.hpp>

using namespace vtr::logging;
using namespace vtr::pose_graph;

TEST(PoseGraph, construction_and_getters) {
  /**
   * Let us create following weighted graph
      0--------1
      |  \     |
      |    \   |
      |      \ |
      2--------3
   */
  using simple::SimpleGraph;

  {
    /// default constructor - empty graph
    LOG(INFO) << "Construct from default constructor.";
    SimpleGraph graph;
    graph.print();
    EXPECT_EQ(graph.numberOfNodes(), (unsigned)0);
    EXPECT_EQ(graph.numberOfEdges(), (unsigned)0);
    EXPECT_TRUE(graph.getNodeIds().empty());
    EXPECT_TRUE(graph.getEdges().empty());
    // now add edges to construct the graph
    graph.addEdge(0, 1);
    graph.addEdge(0, 2);
    graph.addEdge(0, 3);
    graph.addEdge(1, 3);
    graph.addEdge(2, 3);
    // check that the graph has the correct structure
    graph.print();
    EXPECT_EQ(graph.numberOfNodes(), (unsigned)4);
    EXPECT_EQ(graph.numberOfEdges(), (unsigned)5);
    EXPECT_EQ(graph.getNodeIds().size(), (size_t)4);
    EXPECT_EQ(graph.getEdges().size(), (size_t)5);
  }
  {
    /// construct from edge list
    LOG(INFO) << "Construct from list of edges.";
    SimpleGraph graph({{0, 1}, {0, 2}, {0, 3}, {1, 3}, {2, 3}});
    graph.print();
    EXPECT_EQ(graph.numberOfNodes(), (unsigned)4);
    EXPECT_EQ(graph.numberOfEdges(), (unsigned)5);
    EXPECT_EQ(graph.getNodeIds().size(), (size_t)4);
    EXPECT_EQ(graph.getEdges().size(), (size_t)5);
  }
  {
    /// construct from vertex list
    LOG(INFO) << "Construct from list of vertices.";
    SimpleGraph graph({0, 1, 3, 2}, true);  // cyclic = true
    graph.addEdge(0, 3);
    graph.print();
    EXPECT_EQ(graph.numberOfNodes(), (unsigned)4);
    EXPECT_EQ(graph.numberOfEdges(), (unsigned)5);
    EXPECT_EQ(graph.getNodeIds().size(), (size_t)4);
    EXPECT_EQ(graph.getEdges().size(), (size_t)5);
  }
}

TEST(PoseGraph, traversal_and_search) {
  /**
   * Let us create following weighted graph
          7         4
      0--------1---------4
      |  \     |   \     |
     6|   5\   |3   2\   |1
      |      \ |       \ |
      2--------3---------5
          4         3
   */
  using simple::SimpleGraph;

  SimpleGraph graph(
      {{0, 1}, {0, 2}, {0, 3}, {1, 3}, {1, 4}, {1, 5}, {2, 3}, {3, 5}, {4, 5}});

  const auto weight_eval = eval::Weight::Map::MakeShared();
  weight_eval->ref(SimpleGraph::getEdge(0, 1)) = 7;
  weight_eval->ref(SimpleGraph::getEdge(0, 2)) = 6;
  weight_eval->ref(SimpleGraph::getEdge(0, 3)) = 5;
  weight_eval->ref(SimpleGraph::getEdge(1, 3)) = 3;
  weight_eval->ref(SimpleGraph::getEdge(1, 4)) = 4;
  weight_eval->ref(SimpleGraph::getEdge(1, 5)) = 2;
  weight_eval->ref(SimpleGraph::getEdge(2, 3)) = 4;
  weight_eval->ref(SimpleGraph::getEdge(3, 5)) = 3;
  weight_eval->ref(SimpleGraph::getEdge(4, 5)) = 1;

  LOG(INFO) << "graph: ";
  graph.print();

  LOG(INFO) << "dij-trav: ";
  SimpleGraph dijtrav = graph.dijkstraTraverseToDepth(0, 8.0, weight_eval);
  dijtrav.print();

  LOG(INFO) << "dij-search: ";
  SimpleGraph dijs = graph.dijkstraSearch(0, 5, weight_eval);
  dijs.print();

  LOG(INFO) << "dij-multisearch: ";
  SimpleGraph::VertexVec searches;
  searches.push_back(4);
  searches.push_back(3);
  SimpleGraph dijms = graph.dijkstraMultiSearch(0, searches, weight_eval);
  dijms.print();

  LOG(INFO) << "bft: ";
  SimpleGraph bft = graph.breadthFirstTraversal(0, 2);
  bft.print();

  LOG(INFO) << "bft+mask: ";
  const auto maskEval = eval::Mask::Const::MakeShared(true, true);
  SimpleGraph bft2 = graph.breadthFirstTraversal(0, 2, maskEval);
  bft2.print();

  LOG(INFO) << "bfs: ";
  SimpleGraph bfs = graph.breadthFirstSearch(0, 4);
  bfs.print();

  LOG(INFO) << "bfms: ";
  SimpleGraph::VertexVec searches2;
  searches2.push_back(4);
  searches2.push_back(3);
  SimpleGraph bfms = graph.breadthFirstMultiSearch(0, searches2);
  bfms.print();

  LOG(INFO) << "mst: ";
  SimpleGraph mst = graph.getMinimalSpanningTree(weight_eval);
  mst.print();

  LOG(INFO) << "subgraph of dij: ";
  SimpleGraph sub = graph.getSubgraph(dijtrav.getNodeIds());
  sub.print();
}

TEST(PoseGraph, iterators) {
  /**
   * Let us create following weighted graph
          7         4
      0--------1---------4
      |  \     |   \     |
     6|   5\   |3   2\   |1
      |      \ |       \ |
      2--------3---------5
          4         3
   */
  using simple::SimpleGraph;

  SimpleGraph graph(
      {{0, 1}, {0, 2}, {0, 3}, {1, 3}, {1, 4}, {1, 5}, {2, 3}, {3, 5}, {4, 5}});

  const auto weight_eval = eval::Weight::Map::MakeShared();
  weight_eval->ref(SimpleGraph::getEdge(0, 1)) = 7;
  weight_eval->ref(SimpleGraph::getEdge(0, 2)) = 6;
  weight_eval->ref(SimpleGraph::getEdge(0, 3)) = 5;
  weight_eval->ref(SimpleGraph::getEdge(1, 3)) = 3;
  weight_eval->ref(SimpleGraph::getEdge(1, 4)) = 4;
  weight_eval->ref(SimpleGraph::getEdge(1, 5)) = 2;
  weight_eval->ref(SimpleGraph::getEdge(2, 3)) = 4;
  weight_eval->ref(SimpleGraph::getEdge(3, 5)) = 3;
  weight_eval->ref(SimpleGraph::getEdge(4, 5)) = 1;

  const auto mask_eval = eval::Mask::Const::MakeShared(true, true);

  LOG(INFO) << "graph: ";
  graph.print();

  LOG(INFO) << "dij-iterator (depth 8): ";
  for (auto it = graph.beginDijkstra(0, 8, mask_eval, weight_eval);
       it != graph.end(); it++) {
    LOG(INFO) << "  child: " << it->v()
              << ", edge b/t parent & child: " << it->e();
  }

  LOG(INFO) << "dfs-iterator (depth 2): ";
  for (auto it = graph.beginDfs(0, 2, mask_eval); it != graph.end(); it++) {
    LOG(INFO) << "  child: " << it->v()
              << ", edge b/t parent & child: " << it->e();
  }

  LOG(INFO) << "bfs-iterator (depth 2): ";
  for (auto it = graph.beginBfs(0, 2, mask_eval); it != graph.end(); it++) {
    LOG(INFO) << "  child: " << it->v()
              << ", edge b/t parent & child: " << it->e();
  }

  LOG(INFO) << "default(bfs)-iterator (depth 2): ";
  for (auto it = graph.begin(0, 8, mask_eval); it != graph.end(); it++) {
    LOG(INFO) << "  child: " << it->v()
              << ", edge b/t parent & child: " << it->e();
  }
}

TEST(PoseGraph, subgraph) {
  /**
   * Let us create following weighted graph
          7         4
      0--------1---------4
      |  \     |   \     |
     6|   5\   |3   2\   |1
      |      \ |       \ |
      2--------3---------5
          4         3
   */
  using simple::SimpleGraph;

  SimpleGraph graph1({{0, 1}, {0, 2}, {0, 3}, {1, 3}, {1, 4}});
  LOG(INFO) << "graph1: ";
  graph1.print();
  SimpleGraph graph2({{1, 4}, {1, 5}, {2, 3}, {3, 5}, {4, 5}});
  LOG(INFO) << "graph2: ";
  graph2.print();
  auto graph = graph1 + graph2;
  LOG(INFO) << "graph: ";
  graph.print();
  EXPECT_EQ(graph1.numberOfNodes(), (unsigned)5);
  EXPECT_EQ(graph1.numberOfEdges(), (unsigned)5);
  EXPECT_EQ(graph2.numberOfNodes(), (unsigned)5);
  EXPECT_EQ(graph2.numberOfEdges(), (unsigned)5);
  EXPECT_EQ(graph.numberOfNodes(), (unsigned)6);
  EXPECT_EQ(graph.numberOfEdges(), (unsigned)9);

  graph1 += graph2;
  LOG(INFO) << "graph1 updated: ";
  graph1.print();
  EXPECT_EQ(graph1.numberOfNodes(), (unsigned)6);
  EXPECT_EQ(graph1.numberOfEdges(), (unsigned)9);
  EXPECT_EQ(graph2.numberOfNodes(), (unsigned)5);
  EXPECT_EQ(graph2.numberOfEdges(), (unsigned)5);

  graph2 += graph1;
  LOG(INFO) << "graph2 updated: ";
  graph2.print();
  EXPECT_EQ(graph1.numberOfNodes(), (unsigned)6);
  EXPECT_EQ(graph1.numberOfEdges(), (unsigned)9);
  EXPECT_EQ(graph2.numberOfNodes(), (unsigned)6);
  EXPECT_EQ(graph2.numberOfEdges(), (unsigned)9);

  const auto weight_eval = eval::Weight::Map::MakeShared();
  weight_eval->ref(SimpleGraph::getEdge(0, 1)) = 7;
  weight_eval->ref(SimpleGraph::getEdge(0, 2)) = 6;
  weight_eval->ref(SimpleGraph::getEdge(0, 3)) = 5;
  weight_eval->ref(SimpleGraph::getEdge(1, 3)) = 3;
  weight_eval->ref(SimpleGraph::getEdge(1, 4)) = 4;
  weight_eval->ref(SimpleGraph::getEdge(1, 5)) = 2;
  weight_eval->ref(SimpleGraph::getEdge(2, 3)) = 4;
  weight_eval->ref(SimpleGraph::getEdge(3, 5)) = 3;
  weight_eval->ref(SimpleGraph::getEdge(4, 5)) = 1;

  const auto mask_eval = eval::Mask::Map::MakeShared();
  // masked edges
  mask_eval->ref(SimpleGraph::getEdge(0, 3)) = false;
  mask_eval->ref(SimpleGraph::getEdge(1, 3)) = false;
  mask_eval->ref(SimpleGraph::getEdge(1, 5)) = false;
  mask_eval->ref(SimpleGraph::getEdge(0, 1)) = true;
  mask_eval->ref(SimpleGraph::getEdge(0, 2)) = true;
  mask_eval->ref(SimpleGraph::getEdge(1, 4)) = true;
  mask_eval->ref(SimpleGraph::getEdge(2, 3)) = true;
  mask_eval->ref(SimpleGraph::getEdge(3, 5)) = true;
  mask_eval->ref(SimpleGraph::getEdge(4, 5)) = true;
  // masked vertices
  mask_eval->ref(0) = false;
  mask_eval->ref(1) = true;
  mask_eval->ref(2) = true;
  mask_eval->ref(3) = true;
  mask_eval->ref(4) = true;
  mask_eval->ref(5) = true;

  /// get subgraph containing all nodes \ masked edges
  /// \note the following results in a disconnected graph but is ok
  auto subgraph1 = graph.getSubgraph({0, 1, 2, 3, 4}, mask_eval);
  LOG(INFO) << "subgraph1: ";
  subgraph1.print();

  /// get subgraph using bfs method
  auto subgraph2 = graph.getSubgraph(1, mask_eval);
  LOG(INFO) << "subgraph2: ";
  subgraph2.print();

  /// get subgraph using bfs method to depth
  auto subgraph3 = graph.getSubgraph(1, 2, mask_eval);
  LOG(INFO) << "subgraph3: ";
  subgraph3.print();
}

TEST(PoseGraph, path_decomposition) {
  /// \note path decomposition needs more investigation, the function may not
  /// return what's expected
  /**
   * Let us create following weighted graph
          7         4
      0--------1---------4
      |  \     |
     6|   5\   |3
      |      \ |
      2--------3---------5
          4         3
   */
  using simple::SimpleGraph;

  SimpleGraph graph({{0, 1}, {0, 2}, {0, 3}, {1, 3}, {1, 4}, {2, 3}, {3, 5}});
  LOG(INFO) << "graph: ";
  graph.print();

  SimpleGraph::ComponentList paths, cycles;
  const auto junctions = graph.pathDecomposition(&paths, &cycles);

  std::stringstream ss;
  ss << std::endl;
  for (auto junc : junctions) ss << junc << ", ";
  ss << std::endl;
  ss << "paths: " << std::endl;
  for (auto&& path : paths) {
    for (auto it = path.begin(); it != path.end(); it++) ss << *it << ", ";
    ss << std::endl;
  }
  ss << "cycles: " << std::endl;
  for (auto&& path : cycles) {
    for (auto it = path.begin(); it != path.end(); it++) ss << *it << ", ";
    ss << std::endl;
  }
  LOG(INFO) << ss.str();
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}