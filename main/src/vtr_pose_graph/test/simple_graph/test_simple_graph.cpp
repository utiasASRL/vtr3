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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/simple_graph/simple_graph.hpp"
#include "vtr_pose_graph/simple_graph/simple_iterator.hpp"

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
  VertexId v0(0, 0), v1(0, 1), v2(0, 2), v3(0, 3);

  {
    /// default constructor - empty graph
    CLOG(INFO, "test") << "Construct from default constructor.";
    SimpleGraph graph;
    graph.print();
    EXPECT_EQ(graph.numberOfNodes(), (unsigned)0);
    EXPECT_EQ(graph.numberOfEdges(), (unsigned)0);
    // now add edges to construct the graph
    graph.addEdge(v0, v1);
    graph.addEdge(v0, v2);
    graph.addEdge(v0, v3);
    graph.addEdge(v1, v3);
    graph.addEdge(v2, v3);
    // check that the graph has the correct structure
    graph.print();
    EXPECT_EQ(graph.numberOfNodes(), (unsigned)4);
    EXPECT_EQ(graph.numberOfEdges(), (unsigned)5);
  }
  {
    /// construct from edge list
    CLOG(INFO, "test") << "Construct from list of edges.";
    SimpleGraph graph({EdgeId{v0, v1}, EdgeId{v0, v2}, EdgeId{v0, v3},
                       EdgeId{v1, v3}, EdgeId{v2, v3}});
    graph.print();
    EXPECT_EQ(graph.numberOfNodes(), (unsigned)4);
    EXPECT_EQ(graph.numberOfEdges(), (unsigned)5);
  }
  {
    /// construct from vertex list
    CLOG(INFO, "test") << "Construct from list of vertices.";
    SimpleGraph graph({v0, v1, v3, v2}, /* cyclic */ true);
    graph.addEdge(v0, v3);
    graph.print();
    EXPECT_EQ(graph.numberOfNodes(), (unsigned)4);
    EXPECT_EQ(graph.numberOfEdges(), (unsigned)5);
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
  VertexId v0(0, 0), v1(0, 1), v2(0, 2), v3(0, 3), v4(0, 4), v5(0, 5);

  SimpleGraph graph({EdgeId{v0, v1}, EdgeId{v0, v2}, EdgeId{v0, v3},
                     EdgeId{v1, v3}, EdgeId{v1, v4}, EdgeId{v1, v5},
                     EdgeId{v2, v3}, EdgeId{v3, v5}, EdgeId{v4, v5}});

  const auto weight_eval = std::make_shared<eval::weight::MapEval>();
  weight_eval->ref(EdgeId(v0, v1)) = 7;
  weight_eval->ref(EdgeId(v0, v2)) = 6;
  weight_eval->ref(EdgeId(v0, v3)) = 5;
  weight_eval->ref(EdgeId(v1, v3)) = 3;
  weight_eval->ref(EdgeId(v1, v4)) = 4;
  weight_eval->ref(EdgeId(v1, v5)) = 2;
  weight_eval->ref(EdgeId(v2, v3)) = 4;
  weight_eval->ref(EdgeId(v3, v5)) = 3;
  weight_eval->ref(EdgeId(v4, v5)) = 1;

  CLOG(INFO, "test") << "graph: ";
  graph.print();

  CLOG(INFO, "test") << "dij-trav: ";
  SimpleGraph dijtrav = graph.dijkstraTraverseToDepth(v0, 8.0, weight_eval);
  dijtrav.print();

  CLOG(INFO, "test") << "dij-search: ";
  SimpleGraph dijs = graph.dijkstraSearch(v0, v5, weight_eval);
  dijs.print();

  CLOG(INFO, "test") << "dij-multisearch: ";
  SimpleGraph::VertexVec searches;
  searches.push_back(v4);
  searches.push_back(v3);
  SimpleGraph dijms = graph.dijkstraMultiSearch(v0, searches, weight_eval);
  dijms.print();

  CLOG(INFO, "test") << "bft: ";
  SimpleGraph bft = graph.breadthFirstTraversal(v0, 2);
  bft.print();

  CLOG(INFO, "test") << "bft+mask: ";
  const auto maskEval = std::make_shared<eval::mask::ConstEval>(true, true);
  SimpleGraph bft2 = graph.breadthFirstTraversal(v0, v2, maskEval);
  bft2.print();

  CLOG(INFO, "test") << "bfs: ";
  SimpleGraph bfs = graph.breadthFirstSearch(v0, v4);
  bfs.print();

  CLOG(INFO, "test") << "bfms: ";
  SimpleGraph::VertexVec searches2;
  searches2.push_back(v4);
  searches2.push_back(v3);
  SimpleGraph bfms = graph.breadthFirstMultiSearch(v0, searches2);
  bfms.print();

  CLOG(INFO, "test") << "mst: ";
  SimpleGraph mst = graph.getMinimalSpanningTree(weight_eval);
  mst.print();
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
  VertexId v0(0, 0), v1(0, 1), v2(0, 2), v3(0, 3), v4(0, 4), v5(0, 5);

  SimpleGraph graph({EdgeId{v0, v1}, EdgeId{v0, v2}, EdgeId{v0, v3},
                     EdgeId{v1, v3}, EdgeId{v1, v4}, EdgeId{v1, v5},
                     EdgeId{v2, v3}, EdgeId{v3, v5}, EdgeId{v4, v5}});

  const auto weight_eval = std::make_shared<eval::weight::MapEval>();
  weight_eval->ref(EdgeId(v0, v1)) = 7;
  weight_eval->ref(EdgeId(v0, v2)) = 6;
  weight_eval->ref(EdgeId(v0, v3)) = 5;
  weight_eval->ref(EdgeId(v1, v3)) = 3;
  weight_eval->ref(EdgeId(v1, v4)) = 4;
  weight_eval->ref(EdgeId(v1, v5)) = 2;
  weight_eval->ref(EdgeId(v2, v3)) = 4;
  weight_eval->ref(EdgeId(v3, v5)) = 3;
  weight_eval->ref(EdgeId(v4, v5)) = 1;

  const auto mask_eval = std::make_shared<eval::mask::ConstEval>(true, true);

  CLOG(INFO, "test") << "graph: ";
  graph.print();

  CLOG(INFO, "test") << "dij-iterator (depth 8): ";
  for (auto it = graph.beginDijkstra(v0, 8.0, mask_eval, weight_eval);
       it != graph.end(); it++) {
    CLOG(INFO, "test") << "  child: " << it->v()
                       << ", edge b/t parent & child: " << it->e();
  }

  CLOG(INFO, "test") << "dfs-iterator (depth 2): ";
  for (auto it = graph.beginDfs(v0, 2.0, mask_eval); it != graph.end(); it++) {
    CLOG(INFO, "test") << "  child: " << it->v()
                       << ", edge b/t parent & child: " << it->e();
  }

  CLOG(INFO, "test") << "bfs-iterator (depth 2): ";
  for (auto it = graph.beginBfs(v0, 2.0, mask_eval); it != graph.end(); it++) {
    CLOG(INFO, "test") << "  child: " << it->v()
                       << ", edge b/t parent & child: " << it->e();
  }

  CLOG(INFO, "test") << "default(bfs)-iterator (depth 2): ";
  for (auto it = graph.begin(v0, 8.0, mask_eval); it != graph.end(); it++) {
    CLOG(INFO, "test") << "  child: " << it->v()
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
  VertexId v0(0, 0), v1(0, 1), v2(0, 2), v3(0, 3), v4(0, 4), v5(0, 5);

  SimpleGraph graph1({EdgeId{v0, v1}, EdgeId{v0, v2}, EdgeId{v0, v3},
                      EdgeId{v1, v3}, EdgeId{v1, v4}});
  CLOG(INFO, "test") << "graph1: ";
  graph1.print();
  SimpleGraph graph2({EdgeId{v1, v4}, EdgeId{v1, v5}, EdgeId{v2, v3},
                      EdgeId{v3, v5}, EdgeId{v4, v5}});
  CLOG(INFO, "test") << "graph2: ";
  graph2.print();
  auto graph = graph1 + graph2;
  CLOG(INFO, "test") << "graph: ";
  graph.print();
  EXPECT_EQ(graph1.numberOfNodes(), (unsigned)5);
  EXPECT_EQ(graph1.numberOfEdges(), (unsigned)5);
  EXPECT_EQ(graph2.numberOfNodes(), (unsigned)5);
  EXPECT_EQ(graph2.numberOfEdges(), (unsigned)5);
  EXPECT_EQ(graph.numberOfNodes(), (unsigned)6);
  EXPECT_EQ(graph.numberOfEdges(), (unsigned)9);

  graph1 += graph2;
  CLOG(INFO, "test") << "graph1 updated: ";
  graph1.print();
  EXPECT_EQ(graph1.numberOfNodes(), (unsigned)6);
  EXPECT_EQ(graph1.numberOfEdges(), (unsigned)9);
  EXPECT_EQ(graph2.numberOfNodes(), (unsigned)5);
  EXPECT_EQ(graph2.numberOfEdges(), (unsigned)5);

  graph2 += graph1;
  CLOG(INFO, "test") << "graph2 updated: ";
  graph2.print();
  EXPECT_EQ(graph1.numberOfNodes(), (unsigned)6);
  EXPECT_EQ(graph1.numberOfEdges(), (unsigned)9);
  EXPECT_EQ(graph2.numberOfNodes(), (unsigned)6);
  EXPECT_EQ(graph2.numberOfEdges(), (unsigned)9);

  const auto weight_eval = std::make_shared<eval::weight::MapEval>();
  weight_eval->ref(EdgeId(v0, v1)) = 7;
  weight_eval->ref(EdgeId(v0, v2)) = 6;
  weight_eval->ref(EdgeId(v0, v3)) = 5;
  weight_eval->ref(EdgeId(v1, v3)) = 3;
  weight_eval->ref(EdgeId(v1, v4)) = 4;
  weight_eval->ref(EdgeId(v1, v5)) = 2;
  weight_eval->ref(EdgeId(v2, v3)) = 4;
  weight_eval->ref(EdgeId(v3, v5)) = 3;
  weight_eval->ref(EdgeId(v4, v5)) = 1;

  const auto mask_eval = std::make_shared<eval::mask::MapEval>();
  // masked edges
  mask_eval->ref(EdgeId(v0, v3)) = false;
  mask_eval->ref(EdgeId(v1, v3)) = false;
  mask_eval->ref(EdgeId(v1, v5)) = false;
  mask_eval->ref(EdgeId(v0, v1)) = true;
  mask_eval->ref(EdgeId(v0, v2)) = true;
  mask_eval->ref(EdgeId(v1, v4)) = true;
  mask_eval->ref(EdgeId(v2, v3)) = true;
  mask_eval->ref(EdgeId(v3, v5)) = true;
  mask_eval->ref(EdgeId(v4, v5)) = true;
  // masked vertices
  mask_eval->ref(v0) = false;
  mask_eval->ref(v1) = true;
  mask_eval->ref(v2) = true;
  mask_eval->ref(v3) = true;
  mask_eval->ref(v4) = true;
  mask_eval->ref(v5) = true;

  /// get subgraph containing all nodes \ masked edges
  /// \note the following results in a disconnected graph but is ok
  auto subgraph1 = graph.getSubgraph({v0, v1, v2, v3, v4}, mask_eval);
  CLOG(INFO, "test") << "subgraph1: ";
  subgraph1.print();

  /// get subgraph using bfs method
  auto subgraph2 = graph.getSubgraph(v1, mask_eval);
  CLOG(INFO, "test") << "subgraph2: ";
  subgraph2.print();

  /// get subgraph using bfs method to depth
  auto subgraph3 = graph.getSubgraph(v1, 2.0, mask_eval);
  CLOG(INFO, "test") << "subgraph3: ";
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
  VertexId v0(0, 0), v1(0, 1), v2(0, 2), v3(0, 3), v4(0, 4), v5(0, 5);

  SimpleGraph graph({EdgeId{v0, v1}, EdgeId{v0, v2}, EdgeId{v0, v3},
                     EdgeId{v1, v3}, EdgeId{v1, v4}, EdgeId{v2, v3},
                     EdgeId{v3, v5}});
  CLOG(INFO, "test") << "graph: ";
  graph.print();

  SimpleGraph::ComponentList paths, cycles;
  const auto junctions = graph.pathDecomposition(paths, cycles);

  std::stringstream ss;
  ss << std::endl;
  ss << "junctions: " << std::endl;
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
  CLOG(INFO, "test") << ss.str();
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}