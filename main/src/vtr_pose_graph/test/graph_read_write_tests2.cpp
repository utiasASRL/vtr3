#include <gtest/gtest.h>

#include <vtr_logging/logging_init.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

namespace fs = std::filesystem;
using namespace vtr::pose_graph;

TEST(PoseGraph, readWriteIndex) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_pose_graph_test"};
  fs::remove_all(working_dir);  // make sure the directoy is empty.
  fs::path graph_index_file{"graph_index"};
  int robot_id{666};

  // Initialize pose graph
  std::unique_ptr<RCGraph> graph{
      new RCGraph((working_dir / graph_index_file).string(), 0)};
  std::cout << "Created graph: " << graph->filePath() << std::endl;

  vtr_messages::msg::TimeStamp stamp;
  stamp.nanoseconds_since_epoch = 0;

  auto run_id = graph->addRun(robot_id);
  std::cout << "Added run " << run_id << std::endl;

  std::cout << "Building test graph..." << std::endl;
  std::vector<RCVertex::Ptr> v;
  for (int i = 0; i < 8; i++) {
    v.push_back(graph->addVertex(stamp));
    stamp.nanoseconds_since_epoch++;
  }

  std::vector<RCEdge::Ptr> e;
  e.push_back(graph->addEdge(v[0]->id(), v[1]->id()));
  e.push_back(graph->addEdge(v[0]->id(), v[2]->id()));
  e.push_back(graph->addEdge(v[0]->id(), v[3]->id()));
  e.push_back(graph->addEdge(v[1]->id(), v[3]->id()));
  e.push_back(graph->addEdge(v[2]->id(), v[3]->id()));

  e.push_back(graph->addEdge(v[4]->id(), v[5]->id()));
  e.push_back(graph->addEdge(v[4]->id(), v[6]->id()));
  e.push_back(graph->addEdge(v[4]->id(), v[7]->id()));
  e.push_back(graph->addEdge(v[5]->id(), v[7]->id()));
  e.push_back(graph->addEdge(v[6]->id(), v[7]->id()));

  e.push_back(
      graph->addEdge(v[4]->id(), v[0]->id(), RCEdge::EnumType::Spatial));
  e.push_back(
      graph->addEdge(v[5]->id(), v[1]->id(), RCEdge::EnumType::Spatial));
  e.push_back(
      graph->addEdge(v[6]->id(), v[2]->id(), RCEdge::EnumType::Spatial));
  e.push_back(
      graph->addEdge(v[7]->id(), v[3]->id(), RCEdge::EnumType::Spatial));

  auto run_id2 = graph->addRun(robot_id);
  std::cout << "Added run " << run_id2 << std::endl;

  std::cout << "Building second run of test graph..." << std::endl;
  for (int i = 0; i < 4; i++) {
    v.push_back(graph->addVertex(stamp));
    stamp.nanoseconds_since_epoch++;
  }

  e.push_back(graph->addEdge(v[8]->id(), v[9]->id()));
  e.push_back(graph->addEdge(v[8]->id(), v[10]->id()));
  e.push_back(graph->addEdge(v[8]->id(), v[11]->id()));
  e.push_back(graph->addEdge(v[9]->id(), v[11]->id()));
  e.push_back(graph->addEdge(v[10]->id(), v[11]->id()));

  e.push_back(
      graph->addEdge(v[8]->id(), v[0]->id(), RCEdge::EnumType::Spatial));
  e.push_back(
      graph->addEdge(v[9]->id(), v[1]->id(), RCEdge::EnumType::Spatial));
  e.push_back(
      graph->addEdge(v[10]->id(), v[2]->id(), RCEdge::EnumType::Spatial));
  e.push_back(
      graph->addEdge(v[11]->id(), v[3]->id(), RCEdge::EnumType::Spatial));
  e.push_back(
      graph->addEdge(v[8]->id(), v[4]->id(), RCEdge::EnumType::Spatial));
  e.push_back(
      graph->addEdge(v[9]->id(), v[5]->id(), RCEdge::EnumType::Spatial));
  e.push_back(
      graph->addEdge(v[10]->id(), v[6]->id(), RCEdge::EnumType::Spatial));
  e.push_back(
      graph->addEdge(v[11]->id(), v[7]->id(), RCEdge::EnumType::Spatial));

  std::cout << "Saving graph..." << std::endl;
  graph->save();

  std::cout << "Loading saved graph..." << std::endl;
  RCGraph::Ptr graph2 = RCGraph::MakeShared(graph->filePath());
  graph2->load();

  bool passed = true;
  std::cout << std::endl << "Edges: " << std::endl;
  std::cout << "   Original  \t    Loaded   " << std::endl;
  std::cout << "======================================" << std::endl;
  for (auto it = e.begin(); it != e.end(); ++it) {
    auto tmp = graph2->at((*it)->id());
    std::cout << *(*it) << "\t" << *tmp << std::endl;

    passed =
        passed && ((*it)->from() == tmp->from()) && ((*it)->to() == tmp->to());
  }

  EXPECT_TRUE(passed);
  std::cout << "======================================" << std::endl;
  std::cout << (passed ? "PASSED" : "!!!!FAILED!!!!") << std::endl;

  passed = true;
  std::cout << std::endl << "Vertices: " << std::endl;
  std::cout << "======================================" << std::endl;
  for (auto it = v.begin(); it != v.end(); ++it) {
    auto tmp = graph2->at((*it)->id());

    auto eold = (*it)->incident();
    auto enew = (tmp)->incident();

    std::cout << "Vertex " << *(*it) << ": " << std::endl;

    auto jt = eold.begin();
    std::cout << *jt;
    jt++;

    for (; jt != eold.end(); ++jt) {
      std::cout << ", " << *jt;
    }

    passed = passed && (*eold.begin() == *enew.begin());
    std::cout << std::endl << *enew.begin();

    jt = eold.begin()++;
    auto kt = enew.begin()++;
    for (unsigned int i = 1; i < enew.size(); ++i) {
      std::cout << ", " << *kt;
      passed = passed && (*jt == *kt);
    }

    std::cout << std::endl << std::endl;
  }

  EXPECT_TRUE(passed);
  std::cout << "======================================" << std::endl;
  std::cout << (passed ? "PASSED" : "!!!!FAILED!!!!") << std::endl;

  // Cleanup
  fs::remove_all(working_dir);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
