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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include "rcpputils/filesystem_helper.hpp"

#include "std_msgs/msg/string.hpp"

#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/serializable/rc_edge.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::pose_graph;
using namespace vtr::storage;

using StringMsg = std_msgs::msg::String;

namespace {

using TransformT = EdgeTransform;
using TransformMsg = vtr_common_msgs::msg::LieGroupTransform;

TransformMsg toMsg(const TransformT& T) {
  TransformMsg msg;

  // transform
  msg.xi.clear();
  msg.xi.reserve(6);
  auto vec = T.vec();
  for (int row = 0; row < 6; ++row) msg.xi.push_back(vec(row));

  // covariance
  msg.cov.clear();
  msg.cov.reserve(36);
  if (!T.covarianceSet()) {
    msg.cov_set = false;
  } else {
    auto cov = T.cov();
    for (int row = 0; row < 6; row++)
      for (int col = 0; col < 6; col++) msg.cov.push_back(cov(row, col));
    msg.cov_set = true;
  }

  return msg;
}

}  // namespace

TEST(TestSerializationEdge, construct_spatial_edge_directly) {
  VertexId to(0, 1), from(1, 3);

  using TransformVecT = Eigen::Matrix<double, 6, 1>;
  TransformVecT transform_vec;
  transform_vec << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  EdgeTransform transform(transform_vec);
  transform.setCovariance(Eigen::Matrix<double, 6, 6>::Identity());

  RCEdge edge(from, to, EdgeType::Spatial, false, transform);

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
    EXPECT_EQ(data.from_id, from);
    EXPECT_EQ(data.to_id, to);
    EXPECT_EQ(data.t_to_from.xi.size(), (size_t)6);
    CLOG(INFO, "test") << "t_to_from xi: " << data.t_to_from.xi;
    EXPECT_TRUE(data.t_to_from.cov_set);
    EXPECT_EQ(data.t_to_from.cov.size(), (size_t)36);
    CLOG(INFO, "test") << "t_to_from cov: " << data.t_to_from.cov;

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
    EXPECT_EQ(data.from_id, from);
    EXPECT_EQ(data.to_id, to);
    EXPECT_EQ(data.t_to_from.xi.size(), (size_t)6);
    CLOG(INFO, "test") << "t_to_from xi: " << data.t_to_from.xi;
    EXPECT_TRUE(data.t_to_from.cov_set);
    EXPECT_EQ(data.t_to_from.cov.size(), (size_t)36);
    CLOG(INFO, "test") << "t_to_from cov: " << data.t_to_from.cov;
  }
}

TEST(TestSerializationEdge, simulate_load_spatial_edge_from_disk) {
  VertexId to(0, 1), from(1, 3);

  using TransformVecT = Eigen::Matrix<double, 6, 1>;
  TransformVecT transform_vec;
  transform_vec << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  EdgeTransform transform(transform_vec);
  transform.setCovariance(Eigen::Matrix<double, 6, 6>::Identity());

  const auto data = std::make_shared<RCEdge::EdgeMsg>();
  data->type.type = RCEdge::EdgeTypeMsg::SPATIAL;
  data->mode.mode = RCEdge::EdgeModeMsg::AUTONOMOUS;
  data->from_id = from;
  data->to_id = to;
  data->t_to_from = toMsg(transform);

  // assume the message is saved
  auto msg = std::make_shared<LockableMessage<RCEdge::EdgeMsg>>(
      data, NO_TIMESTAMP_VALUE, 1);

  RCEdge edge(*data, msg);

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
    EXPECT_EQ(data.from_id, from);
    EXPECT_EQ(data.to_id, to);
    EXPECT_EQ(data.t_to_from.xi.size(), (size_t)6);
    CLOG(INFO, "test") << "t_to_from xi: " << data.t_to_from.xi;
    EXPECT_TRUE(data.t_to_from.cov_set);
    EXPECT_EQ(data.t_to_from.cov.size(), (size_t)36);
    CLOG(INFO, "test") << "t_to_from cov: " << data.t_to_from.cov;
  }
}

TEST(TestSerializationEdge, construct_temporal_edge_directly) {
  VertexId from(3, 3), to(3, 4);

  using TransformVecT = Eigen::Matrix<double, 6, 1>;
  TransformVecT transform_vec;
  transform_vec << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  EdgeTransform transform(transform_vec);
  transform.setCovariance(Eigen::Matrix<double, 6, 6>::Identity());

  RCEdge edge(from, to, EdgeType::Temporal, true, transform);

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
    EXPECT_EQ(data.from_id, from);
    EXPECT_EQ(data.to_id, to);
    EXPECT_EQ(data.t_to_from.xi.size(), (size_t)6);
    CLOG(INFO, "test") << "t_to_from xi: " << data.t_to_from.xi;
    EXPECT_TRUE(data.t_to_from.cov_set);
    EXPECT_EQ(data.t_to_from.cov.size(), (size_t)36);
    CLOG(INFO, "test") << "t_to_from cov: " << data.t_to_from.cov;

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
    EXPECT_EQ(data.from_id, from);
    EXPECT_EQ(data.to_id, to);
    EXPECT_EQ(data.t_to_from.xi.size(), (size_t)6);
    CLOG(INFO, "test") << "t_to_from xi: " << data.t_to_from.xi;
    EXPECT_TRUE(data.t_to_from.cov_set);
    EXPECT_EQ(data.t_to_from.cov.size(), (size_t)36);
    CLOG(INFO, "test") << "t_to_from cov: " << data.t_to_from.cov;
  }
}

TEST(TestSerializationEdge, simulate_load_temporal_edge_from_disk) {
  VertexId from(3, 3), to(3, 4);

  using TransformVecT = Eigen::Matrix<double, 6, 1>;
  TransformVecT transform_vec;
  transform_vec << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  EdgeTransform transform(transform_vec);
  transform.setCovariance(Eigen::Matrix<double, 6, 6>::Identity());

  const auto data = std::make_shared<RCEdge::EdgeMsg>();
  data->type.type = RCEdge::EdgeTypeMsg::TEMPORAL;
  data->mode.mode = RCEdge::EdgeModeMsg::MANUAL;
  data->from_id = from;
  data->to_id = to;
  data->t_to_from = toMsg(transform);

  // assume the message is saved
  auto msg = std::make_shared<LockableMessage<RCEdge::EdgeMsg>>(
      data, NO_TIMESTAMP_VALUE, 1);

  RCEdge edge(*data, msg);

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
    EXPECT_EQ(data.from_id, from);
    EXPECT_EQ(data.to_id, to);
    EXPECT_EQ(data.t_to_from.xi.size(), (size_t)6);
    CLOG(INFO, "test") << "t_to_from xi: " << data.t_to_from.xi;
    EXPECT_TRUE(data.t_to_from.cov_set);
    EXPECT_EQ(data.t_to_from.cov.size(), (size_t)36);
    CLOG(INFO, "test") << "t_to_from cov: " << data.t_to_from.cov;
  }
}

TEST(TestSerializationEdge, change_edge_transform) {
  VertexId from(3, 3), to(3, 4);

  using TransformVecT = Eigen::Matrix<double, 6, 1>;
  TransformVecT transform_vec;
  transform_vec << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  EdgeTransform transform(transform_vec);
  transform.setCovariance(Eigen::Matrix<double, 6, 6>::Identity());

  RCEdge edge(from, to, EdgeType::Temporal, true, transform);

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
    EXPECT_EQ(data.from_id, from);
    EXPECT_EQ(data.to_id, to);
    EXPECT_EQ(data.t_to_from.xi.size(), (size_t)6);
    CLOG(INFO, "test") << "t_to_from xi: " << data.t_to_from.xi;
    EXPECT_TRUE(data.t_to_from.cov_set);
    EXPECT_EQ(data.t_to_from.cov.size(), (size_t)36);
    CLOG(INFO, "test") << "t_to_from cov: " << data.t_to_from.cov;

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
    EXPECT_EQ(data.from_id, from);
    EXPECT_EQ(data.to_id, to);
    EXPECT_EQ(data.t_to_from.xi.size(), (size_t)6);
    CLOG(INFO, "test") << "t_to_from xi: " << data.t_to_from.xi;
    EXPECT_TRUE(data.t_to_from.cov_set);
    EXPECT_EQ(data.t_to_from.cov.size(), (size_t)36);
    CLOG(INFO, "test") << "t_to_from cov: " << data.t_to_from.cov;
  }

  edge.setTransform(EdgeTransform(true));

  // get the ROS message for the third time (save with modification)
  {
    const auto edge_msg_ptr = edge.serialize();
    auto& edge_msg = edge_msg_ptr->locked().get();

    EXPECT_EQ(edge_msg.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(edge_msg.getIndex(), 1);
    EXPECT_FALSE(edge_msg.getSaved());
    // check cata
    const auto data = edge_msg.getData();
    EXPECT_EQ(data.type.type, RCEdge::EdgeTypeMsg::TEMPORAL);
    EXPECT_EQ(data.mode.mode, RCEdge::EdgeModeMsg::MANUAL);
    EXPECT_EQ(data.from_id, from);
    EXPECT_EQ(data.to_id, to);
    EXPECT_EQ(data.t_to_from.xi.size(), (size_t)6);
    CLOG(INFO, "test") << "t_to_from xi: " << data.t_to_from.xi;
    EXPECT_TRUE(data.t_to_from.cov_set);
    EXPECT_EQ(data.t_to_from.cov.size(), (size_t)36);
    CLOG(INFO, "test") << "t_to_from cov: " << data.t_to_from.cov;

    // assume that we save the message (setting index, ok since same index)
    edge_msg.setIndex(1);
  }

  // get the ROS message for the forth time (save without modification)
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
    EXPECT_EQ(data.from_id, from);
    EXPECT_EQ(data.to_id, to);
    EXPECT_EQ(data.t_to_from.xi.size(), (size_t)6);
    CLOG(INFO, "test") << "t_to_from xi: " << data.t_to_from.xi;
    EXPECT_TRUE(data.t_to_from.cov_set);
    EXPECT_EQ(data.t_to_from.cov.size(), (size_t)36);
    CLOG(INFO, "test") << "t_to_from cov: " << data.t_to_from.cov;
  }
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
