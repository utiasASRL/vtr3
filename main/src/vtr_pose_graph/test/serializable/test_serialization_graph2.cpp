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
 * \file graph_read_write_tests2.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/serializable/rc_graph.hpp>

namespace fs = std::filesystem;
using namespace vtr::pose_graph;
using namespace vtr::logging;

TEST(PoseGraph, ReadWriteIndex) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_pose_graph_test"};
  fs::remove_all(working_dir);  // make sure the directoy is empty.

  // Initialize pose graph
  LOG(INFO) << "Creating graph: " << working_dir;
  std::unique_ptr<RCGraph> graph{new RCGraph(working_dir.string(), false)};

  LOG(INFO) << "Adding run the first run";
  graph->addRun();

  LOG(INFO) << "Building test graph...";
  Timestamp stamp = 0;
  std::vector<RCVertex::Ptr> v;
  for (int i = 0; i < 8; i++) v.push_back(graph->addVertex(stamp++));

  std::vector<RCEdge::Ptr> e;
  e.push_back(graph->addEdge(v[0]->id(), v[1]->id(), Temporal));
  e.push_back(graph->addEdge(v[0]->id(), v[2]->id(), Temporal));
  e.push_back(graph->addEdge(v[0]->id(), v[3]->id(), Temporal));
  e.push_back(graph->addEdge(v[1]->id(), v[3]->id(), Temporal));
  e.push_back(graph->addEdge(v[2]->id(), v[3]->id(), Temporal));

  e.push_back(graph->addEdge(v[4]->id(), v[5]->id(), Temporal));
  e.push_back(graph->addEdge(v[4]->id(), v[6]->id(), Temporal));
  e.push_back(graph->addEdge(v[4]->id(), v[7]->id(), Temporal));
  e.push_back(graph->addEdge(v[5]->id(), v[7]->id(), Temporal));
  e.push_back(graph->addEdge(v[6]->id(), v[7]->id(), Temporal));

  e.push_back(graph->addEdge(v[4]->id(), v[0]->id(), Spatial));
  e.push_back(graph->addEdge(v[5]->id(), v[1]->id(), Spatial));
  e.push_back(graph->addEdge(v[6]->id(), v[2]->id(), Spatial));
  e.push_back(graph->addEdge(v[7]->id(), v[3]->id(), Spatial));

  LOG(INFO) << "Adding the second run";
  graph->addRun();

  LOG(INFO) << "Building second run of test graph...";
  for (int i = 0; i < 4; i++) v.push_back(graph->addVertex(stamp++));

  e.push_back(graph->addEdge(v[8]->id(), v[9]->id(), Temporal));
  e.push_back(graph->addEdge(v[8]->id(), v[10]->id(), Temporal));
  e.push_back(graph->addEdge(v[8]->id(), v[11]->id(), Temporal));
  e.push_back(graph->addEdge(v[9]->id(), v[11]->id(), Temporal));
  e.push_back(graph->addEdge(v[10]->id(), v[11]->id(), Temporal));

  e.push_back(graph->addEdge(v[8]->id(), v[0]->id(), Spatial));
  e.push_back(graph->addEdge(v[9]->id(), v[1]->id(), Spatial));
  e.push_back(graph->addEdge(v[10]->id(), v[2]->id(), Spatial));
  e.push_back(graph->addEdge(v[11]->id(), v[3]->id(), Spatial));
  e.push_back(graph->addEdge(v[8]->id(), v[4]->id(), Spatial));
  e.push_back(graph->addEdge(v[9]->id(), v[5]->id(), Spatial));
  e.push_back(graph->addEdge(v[10]->id(), v[6]->id(), Spatial));
  e.push_back(graph->addEdge(v[11]->id(), v[7]->id(), Spatial));

  LOG(INFO) << "Saving graph...";
  graph->save();

  LOG(INFO) << "Loading saved graph...";
  auto graph2 = RCGraph::MakeShared(working_dir);

  bool passed = true;
  LOG(INFO) << "Edges: ";
  LOG(INFO) << "   Original  \t    Loaded   ";
  LOG(INFO) << "======================================";
  for (auto it = e.begin(); it != e.end(); ++it) {
    auto tmp = graph2->at((*it)->id());
    LOG(INFO) << *(*it) << "\t" << *tmp;
    passed =
        passed && ((*it)->from() == tmp->from()) && ((*it)->to() == tmp->to());
  }

  EXPECT_TRUE(passed);
  LOG(INFO) << "======================================";
  LOG(INFO) << (passed ? "PASSED" : "!!!!FAILED!!!!");

  passed = true;
  LOG(INFO) << "Vertices: ";
  LOG(INFO) << "======================================";
  for (auto it = v.begin(); it != v.end(); ++it) {
    auto tmp = graph2->at((*it)->id());

    auto eold = (*it)->incident();
    auto enew = (tmp)->incident();

    LOG(INFO) << "Vertex " << *(*it) << ": ";
    std::stringstream ss;
    std::stringstream ss2;

    auto jt = eold.begin();
    auto kt = enew.begin();
    for (; jt != eold.end(); jt++, kt++) {
      passed = passed && (*jt == *kt);
      ss << *jt << ", ";
      ss2 << *kt << ", ";
    }
    EXPECT_EQ(kt, enew.end());

    LOG(INFO) << ss.str();
    LOG(INFO) << ss2.str();
  }

  EXPECT_TRUE(passed);
  LOG(INFO) << "======================================";
  LOG(INFO) << (passed ? "PASSED" : "!!!!FAILED!!!!");

  // Cleanup
  fs::remove_all(working_dir);
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
