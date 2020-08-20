#include <gtest/gtest.h>

#include <iostream>
#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/simple_graph/simple_graph.hpp>

TEST(PoseGraph, simpleGraph)
{
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

#if 0
  Eval::Weight::Map::Ptr weightEval(Eval::Weight::Map::MakeShared());
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
  SimpleGraph dijtrav = graph.dijkstraTraverseToDepth(0, 0.0, weightEval);
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
#endif
  EXPECT_EQ(1, 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}