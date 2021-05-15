#include <gtest/gtest.h>

#include <iostream>
#include <random>

#include <vtr_logging/logging_init.hpp>
#include <vtr_messages/msg/rig_landmarks.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_pose_graph/path/path.hpp>

using namespace vtr::pose_graph;

class RetrieveTest : public ::testing::Test {
 public:
  RetrieveTest()
      : test_stream_name_("test_stream"),
        working_dir_(fs::temp_directory_path() / "vtr_loc_retrieve_test"),
        graph_index_file_("graph_index"),
        robot_id_(666) {
  }

  ~RetrieveTest() override = default;

  void SetUp() override {
    // Add a run and 5 vertices to graph
    graph_ = vtr::pose_graph::RCGraph::LoadOrCreate(
        working_dir_ / graph_index_file_, 0);
    LOG(INFO) << "Loaded graph has " << graph_->vertices()->size()
              << " vertices";

    graph_->addRun(robot_id_);

    unsigned major_idx = graph_->runs().size() - 1;
    for (int minor_idx = 0; minor_idx < 5; ++minor_idx) {
      time_stamp_.nanoseconds_since_epoch = 10e9 * major_idx + minor_idx;
      graph_->addVertex(time_stamp_);

      auto curr_vertex = graph_->at(VertexId(major_idx, minor_idx));
      auto run = graph_->run(major_idx);
      run->registerVertexStream<vtr_messages::msg::RigLandmarks>(
          test_stream_name_, true, RegisterMode::Create);
      vtr_messages::msg::RigLandmarks test_msg;
      test_msg.name = "rig-test";
      curr_vertex->insert(test_stream_name_, test_msg, time_stamp_);
    }
    graph_->save();
  }

 protected:
  std::string test_stream_name_;
  fs::path working_dir_;
  fs::path graph_index_file_;
  std::shared_ptr<RCGraph> graph_;
  int robot_id_;
  vtr_messages::msg::TimeStamp time_stamp_;
};

TEST_F(RetrieveTest, RetrieveTest1) {
  for (const auto& r : graph_->runs())
    r.second->registerVertexStream<vtr_messages::msg::RigLandmarks>(
        test_stream_name_, true, RegisterMode::Existing);

  for (int i = graph_->runs().size() - 1; i >= 0; --i) {
    auto v = graph_->at(VertexId(i, 2));
    v->load(test_stream_name_);
    LOG(INFO) << "Retrieving test msg from run " << i << ".";
    auto msg = v->retrieveKeyframeData<vtr_messages::msg::RigLandmarks>(
        test_stream_name_);
    ASSERT_NE(v, nullptr);
    EXPECT_EQ(msg->name, "rig-test");
    LOG(INFO) << "Retrieval successful.";
  }
}

// Run this twice. Second time tests retrieval from disk.
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}