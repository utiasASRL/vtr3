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
 * \file test_serialization_vertex.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include "rcpputils/filesystem_helper.hpp"

#include <vtr_logging/logging_init.hpp>

#include <vtr_pose_graph/serializable/rc_vertex.hpp>

#include "std_msgs/msg/string.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::pose_graph;
using namespace vtr::storage;

using StringMsg = std_msgs::msg::String;

TEST(TestSerializationVertex, construct_vertex_directly) {
  VertexId id(0, 1);
  Timestamp keyframe_time(666);
  const auto name2accessor_map =
      std::make_shared<BubbleInterface::Name2AccessorMap>();

  RCVertex vertex(id, keyframe_time, name2accessor_map);

  // get the ROS message for the first time
  {
    const auto vertex_msg_ptr = vertex.serialize();
    auto& vertex_msg = vertex_msg_ptr->locked().get();

    EXPECT_EQ(vertex_msg.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(vertex_msg.getIndex(), NO_INDEX_VALUE);
    EXPECT_FALSE(vertex_msg.getSaved());
    // check cata
    const auto data = vertex_msg.getData();
    EXPECT_EQ(data.id, id.minorId());
    EXPECT_EQ(data.keyframe_time.nanoseconds_since_epoch, keyframe_time);
    EXPECT_EQ(data.time_range.t1, keyframe_time);
    EXPECT_EQ(data.time_range.t2, keyframe_time);
    EXPECT_TRUE(data.t_vertex_world.entries.empty());
    EXPECT_TRUE(data.t_vertex_world_cov.entries.empty());

    // assume that we save the message (setting index)
    vertex_msg.setIndex(1);
  }

  // get the ROS message for the second time (save without modification)
  {
    const auto vertex_msg_ptr = vertex.serialize();
    auto& vertex_msg = vertex_msg_ptr->locked().get();

    EXPECT_EQ(vertex_msg.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(vertex_msg.getIndex(), 1);
    EXPECT_TRUE(vertex_msg.getSaved());  // nothing changed
    // check cata
    const auto data = vertex_msg.getData();
    EXPECT_EQ(data.id, id.minorId());
    EXPECT_EQ(data.keyframe_time.nanoseconds_since_epoch, keyframe_time);
    EXPECT_EQ(data.time_range.t1, keyframe_time);
    EXPECT_EQ(data.time_range.t2, keyframe_time);
    EXPECT_TRUE(data.t_vertex_world.entries.empty());
    EXPECT_TRUE(data.t_vertex_world_cov.entries.empty());
  }
}

TEST(TestSerializationVertex, simulate_load_vertex_from_disk) {
  VertexId id(0, 1);
  Timestamp keyframe_time(666);
  const auto name2accessor_map =
      std::make_shared<BubbleInterface::Name2AccessorMap>();

  const auto data = std::make_shared<RCVertex::VertexMsg>();
  data->id = id.minorId();
  data->keyframe_time.nanoseconds_since_epoch = keyframe_time;
  data->time_range.t1 = keyframe_time;
  data->time_range.t2 = keyframe_time;

  // assume the message is saved
  auto msg = std::make_shared<LockableMessage<RCVertex::VertexMsg>>(
      data, NO_TIMESTAMP_VALUE, 1);

  RCVertex vertex(*data, id.majorId(), name2accessor_map, msg);

  // get the ROS message for the second time (saved without modification)
  {
    const auto vertex_msg_ptr = vertex.serialize();
    auto& vertex_msg = vertex_msg_ptr->locked().get();

    EXPECT_EQ(vertex_msg.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(vertex_msg.getIndex(), 1);
    EXPECT_TRUE(vertex_msg.getSaved());  // nothing changed
    // check cata
    const auto data = vertex_msg.getData();
    EXPECT_EQ(data.id, id.minorId());
    EXPECT_EQ(data.keyframe_time.nanoseconds_since_epoch, keyframe_time);
    EXPECT_EQ(data.time_range.t1, keyframe_time);
    EXPECT_EQ(data.time_range.t2, keyframe_time);
    EXPECT_TRUE(data.t_vertex_world.entries.empty());
    EXPECT_TRUE(data.t_vertex_world_cov.entries.empty());
  }
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
