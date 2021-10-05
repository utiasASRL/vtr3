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
 * \file test_serialization_run.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include "rcpputils/filesystem_helper.hpp"

#include <vtr_logging/logging_init.hpp>

#include <vtr_pose_graph/serializable/rc_run.hpp>

#include "std_msgs/msg/string.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::pose_graph;
using namespace vtr::storage;

using StringMsg = std_msgs::msg::String;

class TemporaryDirectoryFixture : public Test {
 public:
  TemporaryDirectoryFixture() {
    temp_dir_ = rcpputils::fs::create_temp_directory("tmp_test_dir_").string();
    // temp_dir_ = "/home/yuchen/ASRL/temp/test_posegraph";
    // (void)rcpputils::fs::create_directories(temp_dir_);
    graph_dir_ = (rcpputils::fs::path(temp_dir_) / "graph").string();
  }

  ~TemporaryDirectoryFixture() override {
    rcpputils::fs::remove_all(rcpputils::fs::path(temp_dir_));
  }

  std::string temp_dir_;
  std::string graph_dir_;
};

TEST_F(TemporaryDirectoryFixture, construct_run_directly) {
  RCRun::IdType id = 0;

  RCRun run(id, temp_dir_);

  // get the ROS message for the first time
  {
    const auto run_msg_ptr = run.serialize();
    auto& run_msg = run_msg_ptr->locked().get();

    EXPECT_EQ(run_msg.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(run_msg.getIndex(), NO_INDEX_VALUE);
    EXPECT_FALSE(run_msg.getSaved());
    // check cata
    const auto data = run_msg.getData<RCRun::RunMsg>();
    EXPECT_EQ(data.id, id);
    EXPECT_EQ(data.vertex_rpath, "vertex_index");
    EXPECT_EQ(data.edge_rpaths[0], "temporal_edge_index");
    EXPECT_EQ(data.edge_rpaths[1], "spatial_edge_index");

    // assume that we save the message (setting index)
    run_msg.setIndex(1);
  }

  // get the ROS message for the second time (save without modification)
  {
    const auto run_msg_ptr = run.serialize();
    auto& run_msg = run_msg_ptr->locked().get();

    EXPECT_EQ(run_msg.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(run_msg.getIndex(), 1);
    EXPECT_TRUE(run_msg.getSaved());
    // check cata
    const auto data = run_msg.getData<RCRun::RunMsg>();
    EXPECT_EQ(data.id, id);
    EXPECT_EQ(data.vertex_rpath, "vertex_index");
    EXPECT_EQ(data.edge_rpaths[0], "temporal_edge_index");
    EXPECT_EQ(data.edge_rpaths[1], "spatial_edge_index");
  }
}

TEST_F(TemporaryDirectoryFixture, simulate_load_run_from_disk) {
  RCRun::IdType id = 0;

  RCRun::VertexPtrMapExtern vertex_map;
  RCRun::EdgePtrMapExtern edge_map;
  RCRun::RunFilter run_filter;

  const auto data = std::make_shared<RCRun::RunMsg>();
  data->id = id;
  data->vertex_rpath = "vertex_index";
  data->edge_rpaths.push_back("temporal_edge_index");
  data->edge_rpaths.push_back("spatial_edge_index");

  // assume the message is saved
  auto msg = std::make_shared<LockableMessage>(data, NO_TIMESTAMP_VALUE, 1);

  RCRun run(temp_dir_, *data, vertex_map, edge_map, run_filter, msg);

  // get the ROS message for the second time (saved without modification)
  {
    const auto run_msg_ptr = run.serialize();
    auto& run_msg = run_msg_ptr->locked().get();

    EXPECT_EQ(run_msg.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(run_msg.getIndex(), 1);
    EXPECT_TRUE(run_msg.getSaved());
    // check cata
    const auto data = run_msg.getData<RCRun::RunMsg>();
    EXPECT_EQ(data.id, id);
    EXPECT_EQ(data.vertex_rpath, "vertex_index");
    EXPECT_EQ(data.edge_rpaths[0], "temporal_edge_index");
    EXPECT_EQ(data.edge_rpaths[1], "spatial_edge_index");
  }
}

TEST_F(TemporaryDirectoryFixture, construct_run_with_vertex_edge) {
  RCRun::IdType id = 0;

  // create a run
  {
    RCRun run(id, temp_dir_);

    RCVertex::Ptr last_vertex = nullptr;
    for (Timestamp keyframe_time = 0; keyframe_time < 10; keyframe_time++) {
      const auto vertex = run.addVertex(keyframe_time);
      if (last_vertex)
        run.addEdge(vertex->id(), last_vertex->id(), EdgeId::Type::Temporal,
                    true);
      last_vertex = vertex;
    }

    run.serialize();
  }

  // load and save again
  {
    RCRun::VertexPtrMapExtern vertex_map;
    RCRun::EdgePtrMapExtern edge_map;
    RCRun::RunFilter run_filter;

    const auto data = std::make_shared<RCRun::RunMsg>();
    data->id = id;
    data->vertex_rpath = "vertex_index";
    data->edge_rpaths.push_back("temporal_edge_index");
    data->edge_rpaths.push_back("spatial_edge_index");

    auto msg = std::make_shared<LockableMessage>(data, NO_TIMESTAMP_VALUE, 1);

    RCRun run(temp_dir_, *data, vertex_map, edge_map, run_filter, msg);

    run.serialize();
  }
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
