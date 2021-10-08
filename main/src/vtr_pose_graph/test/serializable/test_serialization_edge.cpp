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
 * \file test_serialization.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include "rcpputils/filesystem_helper.hpp"

#include <vtr_logging/logging_init.hpp>

#include <vtr_pose_graph/serializable/rc_edge.hpp>

#include "std_msgs/msg/string.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::pose_graph;
using namespace vtr::storage;

using StringMsg = std_msgs::msg::String;

TEST(TestSerializationEdge, construct_spatial_edge_directly) {
  VertexId to(0, 1), from(1, 3);

  RCEdge edge(from, to, EdgeId::Type::Spatial, false);

  // get the ROS message for the first time
  {
    const auto edge_msg_ptr = edge.serialize();
    auto& edge_msg = edge_msg_ptr->locked().get();

    EXPECT_EQ(edge_msg.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(edge_msg.getIndex(), NO_INDEX_VALUE);
    EXPECT_FALSE(edge_msg.getSaved());
    // check cata
    const auto data = edge_msg.getData();
    EXPECT_EQ(data.type.type, RCEdge::EdgeTypeMsg::SPATIAL);
    EXPECT_EQ(data.mode.mode, RCEdge::EdgeModeMsg::AUTONOMOUS);
    EXPECT_EQ(data.from_id, from.minorId());
    EXPECT_EQ(data.to_id, to.minorId());
    EXPECT_EQ(data.to_run_id, (int)to.majorId());
    EXPECT_EQ(data.t_to_from.entries.size(), (size_t)6);
    EXPECT_EQ(data.t_to_from_cov.entries.size(), (size_t)0);

    // assume that we save the message (setting index)
    edge_msg.setIndex(1);
  }

  // get the ROS message for the second time (save without modification)
  {
    const auto edge_msg_ptr = edge.serialize();
    auto& edge_msg = edge_msg_ptr->locked().get();

    EXPECT_EQ(edge_msg.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(edge_msg.getIndex(), 1);
    EXPECT_TRUE(edge_msg.getSaved());
    // check cata
    const auto data = edge_msg.getData();
    EXPECT_EQ(data.type.type, RCEdge::EdgeTypeMsg::SPATIAL);
    EXPECT_EQ(data.mode.mode, RCEdge::EdgeModeMsg::AUTONOMOUS);
    EXPECT_EQ(data.from_id, from.minorId());
    EXPECT_EQ(data.to_id, to.minorId());
    EXPECT_EQ(data.to_run_id, (int)to.majorId());
    EXPECT_EQ(data.t_to_from.entries.size(), (size_t)6);
    EXPECT_EQ(data.t_to_from_cov.entries.size(), (size_t)0);
  }
}

TEST(TestSerializationEdge, simulate_load_spatial_edge_from_disk) {
  VertexId to(0, 1), from(1, 3);
  EdgeId id(to, from, EdgeId::Type::Spatial);

  const auto data = std::make_shared<RCEdge::EdgeMsg>();
  data->type.type = RCEdge::EdgeTypeMsg::SPATIAL;
  data->mode.mode = RCEdge::EdgeModeMsg::AUTONOMOUS;
  data->from_id = from.minorId();
  data->to_id = to.minorId();
  data->to_run_id = (int)to.majorId();
  data->t_to_from.entries = {0, 0, 0, 0, 0, 0};
  data->t_to_from_cov.entries = {};

  // assume the message is saved
  auto msg = std::make_shared<LockableMessage<RCEdge::EdgeMsg>>(
      data, NO_TIMESTAMP_VALUE, 1);

  RCEdge edge(*data, from.majorId(), msg);

  // get the ROS message for the second time (saved without modification)
  {
    const auto edge_msg_ptr = edge.serialize();
    auto& edge_msg = edge_msg_ptr->locked().get();

    EXPECT_EQ(edge_msg.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(edge_msg.getIndex(), 1);
    EXPECT_TRUE(edge_msg.getSaved());
    // check data
    const auto data = edge_msg.getData();
    EXPECT_EQ(data.type.type, RCEdge::EdgeTypeMsg::SPATIAL);
    EXPECT_EQ(data.mode.mode, RCEdge::EdgeModeMsg::AUTONOMOUS);
    EXPECT_EQ(data.from_id, from.minorId());
    EXPECT_EQ(data.to_id, to.minorId());
    EXPECT_EQ(data.to_run_id, (int)to.majorId());
    EXPECT_EQ(data.t_to_from.entries.size(), (size_t)6);
    EXPECT_EQ(data.t_to_from_cov.entries.size(), (size_t)0);
  }
}

TEST(TestSerializationEdge, construct_temporal_edge_directly) {
  VertexId to(0, 1), from(0, 3);

  RCEdge edge(from, to, EdgeId::Type::Temporal, true);

  // get the ROS message for the first time
  {
    const auto edge_msg_ptr = edge.serialize();
    auto& edge_msg = edge_msg_ptr->locked().get();

    EXPECT_EQ(edge_msg.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(edge_msg.getIndex(), NO_INDEX_VALUE);
    EXPECT_FALSE(edge_msg.getSaved());
    // check cata
    const auto data = edge_msg.getData();
    EXPECT_EQ(data.type.type, RCEdge::EdgeTypeMsg::TEMPORAL);
    EXPECT_EQ(data.mode.mode, RCEdge::EdgeModeMsg::MANUAL);
    EXPECT_EQ(data.from_id, from.minorId());
    EXPECT_EQ(data.to_id, to.minorId());
    EXPECT_EQ(data.to_run_id, -1);
    EXPECT_EQ(data.t_to_from.entries.size(), (size_t)6);
    EXPECT_EQ(data.t_to_from_cov.entries.size(), (size_t)0);

    // assume that we save the message (setting index)
    edge_msg.setIndex(1);
  }

  // get the ROS message for the second time (save without modification)
  {
    const auto edge_msg_ptr = edge.serialize();
    auto& edge_msg = edge_msg_ptr->locked().get();

    EXPECT_EQ(edge_msg.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(edge_msg.getIndex(), 1);
    EXPECT_TRUE(edge_msg.getSaved());
    // check cata
    const auto data = edge_msg.getData();
    EXPECT_EQ(data.type.type, RCEdge::EdgeTypeMsg::TEMPORAL);
    EXPECT_EQ(data.mode.mode, RCEdge::EdgeModeMsg::MANUAL);
    EXPECT_EQ(data.from_id, from.minorId());
    EXPECT_EQ(data.to_id, to.minorId());
    EXPECT_EQ(data.to_run_id, -1);
    EXPECT_EQ(data.t_to_from.entries.size(), (size_t)6);
    EXPECT_EQ(data.t_to_from_cov.entries.size(), (size_t)0);
  }
}

TEST(TestSerializationEdge, simulate_load_temporal_edge_from_disk) {
  VertexId to(0, 1), from(0, 3);
  EdgeId id(to, from, EdgeId::Type::Temporal);

  const auto data = std::make_shared<RCEdge::EdgeMsg>();
  data->type.type = RCEdge::EdgeTypeMsg::TEMPORAL;
  data->mode.mode = RCEdge::EdgeModeMsg::MANUAL;
  data->from_id = from.minorId();
  data->to_id = to.minorId();
  // data->to_run_id = -1; // default is -1
  data->t_to_from.entries = {0, 0, 0, 0, 0, 0};
  data->t_to_from_cov.entries = {};

  // assume the message is saved
  auto msg = std::make_shared<LockableMessage<RCEdge::EdgeMsg>>(
      data, NO_TIMESTAMP_VALUE, 1);

  RCEdge edge(*data, from.majorId(), msg);

  // get the ROS message for the second time (saved without modification)
  {
    const auto edge_msg_ptr = edge.serialize();
    auto& edge_msg = edge_msg_ptr->locked().get();

    EXPECT_EQ(edge_msg.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(edge_msg.getIndex(), 1);
    EXPECT_TRUE(edge_msg.getSaved());
    // check cata
    const auto data = edge_msg.getData();
    EXPECT_EQ(data.type.type, RCEdge::EdgeTypeMsg::TEMPORAL);
    EXPECT_EQ(data.mode.mode, RCEdge::EdgeModeMsg::MANUAL);
    EXPECT_EQ(data.from_id, from.minorId());
    EXPECT_EQ(data.to_id, to.minorId());
    EXPECT_EQ(data.to_run_id, -1);
    EXPECT_EQ(data.t_to_from.entries.size(), (size_t)6);
    EXPECT_EQ(data.t_to_from_cov.entries.size(), (size_t)0);
  }
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
