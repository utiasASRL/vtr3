#include <gtest/gtest.h>

#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/simple_graph/simple_graph.hpp>

TEST(PoseGraph, simpleGraph1) {
  /* Let us create following weighted graph
          10
      0--------1
      |  \     |
     6|   5\   |15
      |      \ |
      2--------3
          4
  */
  using namespace vtr::pose_graph;
  using simple::SimpleGraph;

  SimpleGraph graph;
  graph.addEdge(0, 1);
  graph.addEdge(0, 2);
  graph.addEdge(0, 3);
  graph.addEdge(1, 3);
  graph.addEdge(2, 3);

  eval::Weight::Map::Ptr weightEval(eval::Weight::Map::MakeShared());

  weightEval->ref(SimpleGraph::getEdge(0, 1)) = 10;
  weightEval->ref(SimpleGraph::getEdge(0, 2)) = 6;
  weightEval->ref(SimpleGraph::getEdge(0, 3)) = 5;
  weightEval->ref(SimpleGraph::getEdge(1, 3)) = 15;
  weightEval->ref(SimpleGraph::getEdge(2, 3)) = 4;

  std::cout << std::endl << "graph" << std::endl;
  graph.print();

  std::cout << std::endl << "mst" << std::endl;
  SimpleGraph mst = graph.getMinimalSpanningTree(weightEval);
  mst.print();

  std::cout << std::endl << "dij-trav" << std::endl;
  SimpleGraph dijtrav = graph.dijkstraTraverseToDepth(0, 6.0, weightEval);
  dijtrav.print();

  std::cout << std::endl << "subgraph of dij" << std::endl;
  SimpleGraph sub = graph.getSubgraph(dijtrav.getNodeIds());
  sub.print();

  std::cout << std::endl << "dij-ms" << std::endl;
  SimpleGraph::VertexVec searches;
  searches.push_back(2);
  searches.push_back(3);
  SimpleGraph dijms = graph.dijkstraMultiSearch(0, searches, weightEval);
  dijms.print();

  // Dummy test placeholder.
  EXPECT_EQ(true, true);
}

TEST(PoseGraph, simpleGraph2) {
  using namespace vtr::pose_graph;
  using simple::SimpleGraph;

  SimpleGraph graph;
  eval::Weight::Map::Ptr weightEval(eval::Weight::Map::MakeShared());
  for (unsigned int i = 2; i <= 20; i++) {
    graph.addEdge(i, i - 1);
    weightEval->ref(SimpleGraph::getEdge(i, i - 1)) = 0;
  }
  graph.addEdge(4, 14);
  weightEval->ref(SimpleGraph::getEdge(4, 14)) = 1;
  graph.addEdge(5, 15);
  weightEval->ref(SimpleGraph::getEdge(5, 15)) = 1;
  graph.addEdge(6, 16);
  weightEval->ref(SimpleGraph::getEdge(6, 16)) = 1;

  std::cout << std::endl << "graph" << std::endl;
  graph.print();

  std::cout << std::endl << "bft" << std::endl;
  SimpleGraph bft = graph.breadthFirstTraversal(6, 3);
  bft.print();

  std::cout << std::endl << "bfms" << std::endl;
  SimpleGraph::VertexVec searches = bft.getNodeIds();
  searches.push_back(12);
  SimpleGraph bfms = graph.breadthFirstMultiSearch(6, searches);
  bfms.print();

  std::cout << std::endl << "subgraph" << std::endl;
  SimpleGraph sub = graph.getSubgraph(bfms.getNodeIds());
  sub.print();

  std::cout << std::endl << "mst of sub" << std::endl;
  SimpleGraph mst = sub.getMinimalSpanningTree(weightEval);
  mst.print();

  std::cout << std::endl << "disconnected" << std::endl;
  SimpleGraph::VertexVec disconnected;
  disconnected.push_back(1);
  disconnected.push_back(2);
  disconnected.push_back(4);
  disconnected.push_back(5);
  SimpleGraph subDisc = graph.getSubgraph(disconnected);
  subDisc.print();

  // Dummy test placeholder.
  EXPECT_EQ(true, true);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}