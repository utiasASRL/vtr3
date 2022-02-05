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
 * \file rc_edge.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/index/edge_base.hpp"
#include "vtr_pose_graph_msgs/msg/edge.hpp"
#include "vtr_storage/stream/message.hpp"

namespace vtr {
namespace pose_graph {

class RCEdge : public EdgeBase {
 public:
  PTR_TYPEDEFS(RCEdge);

  // Helper typedef to find the base class corresponding to edge data
  using Base = EdgeBase;

  // ROS message for serialization
  using EdgeMsg = vtr_pose_graph_msgs::msg::Edge;
  using EdgeModeMsg = vtr_pose_graph_msgs::msg::EdgeMode;
  using EdgeTypeMsg = vtr_pose_graph_msgs::msg::EdgeType;
  using EdgeTransformMsg = vtr_common_msgs::msg::LieGroupTransform;

  static Ptr MakeShared(const VertexId& from_id, const VertexId& to_id,
                        const EdgeType& type, const bool manual,
                        const EdgeTransform& T_to_from) {
    return std::make_shared<RCEdge>(from_id, to_id, type, manual, T_to_from);
  }

  static Ptr MakeShared(const EdgeMsg& msg,
                        const storage::LockableMessage<EdgeMsg>::Ptr& msg_ptr) {
    return std::make_shared<RCEdge>(msg, msg_ptr);
  }

  RCEdge(const VertexId& from_id, const VertexId& to_id, const EdgeType& type,
         const bool manual, const EdgeTransform& T_to_from);

  RCEdge(const EdgeMsg& msg,
         const storage::LockableMessage<EdgeMsg>::Ptr& msg_ptr);

  virtual ~RCEdge() = default;

  /** \brief serializes to a ros message, as an edge */
  storage::LockableMessage<EdgeMsg>::Ptr serialize();

 private:
  storage::LockableMessage<EdgeMsg>::Ptr msg_;
};
}  // namespace pose_graph
}  // namespace vtr
