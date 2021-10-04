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
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_common/utils/container_tools.hpp>
#include <vtr_pose_graph/index/edge_base.hpp>
#include <vtr_storage/stream/message.hpp>

#include <vtr_pose_graph_msgs/msg/edge.hpp>
#include <vtr_pose_graph_msgs/msg/edge_header.hpp>

namespace vtr {
namespace pose_graph {

class RCEdge : public EdgeBase {
 public:
  // Helper typedef to find the base class corresponding to edge data
  using Base = EdgeBase;

  // ROS message for serialization
  using EdgeMsg = vtr_pose_graph_msgs::msg::Edge;
  using EdgeModeMsg = vtr_pose_graph_msgs::msg::EdgeMode;
  using EdgeTypeMsg = vtr_pose_graph_msgs::msg::EdgeType;

  // when loading from disk
  using RunFilter = std::unordered_set<BaseIdType>;

  /** \brief Typedefs for shared pointers to edges */
  PTR_TYPEDEFS(RCEdge)

  /**
   * \brief Interface to downcast base class pointers
   * \details This allows us to do DerivedPtrType = Type::Cast(BasePtrType)
   */
  PTR_DOWNCAST_OPS(RCEdge, EdgeBase)

  /** \brief Typedefs for containers of edges */
  CONTAINER_TYPEDEFS(RCEdge)

  static Ptr MakeShared(const VertexId& from_id, const VertexId& to_id,
                        const EnumType& type, bool manual = false) {
    return Ptr(new RCEdge(from_id, to_id, type, manual));
  }
  static Ptr MakeShared(const VertexId& from_id, const VertexId& to_id,
                        const EnumType& type, const TransformType& T_to_from,
                        bool manual = false) {
    return Ptr(new RCEdge(from_id, to_id, type, T_to_from, manual));
  }
  static Ptr MakeShared(const EdgeMsg& msg, BaseIdType run_id,
                        const storage::LockableMessage::Ptr& msg_ptr) {
    return Ptr(new RCEdge(msg, run_id, msg_ptr));
  }

  RCEdge(const VertexId& from_id, const VertexId& to_id, const EnumType& type,
         bool manual = false);
  RCEdge(const VertexId& from_id, const VertexId& to_id, const EnumType& type,
         const TransformType& T_to_from, bool manual = false);
  RCEdge(const EdgeMsg& msg, BaseIdType run_id,
         const storage::LockableMessage::Ptr& msg_ptr);

  virtual ~RCEdge() = default;

  /** \brief serializes to a ros message, as an edge */
  storage::LockableMessage::Ptr serialize();

  /** \brief Helper for run filtering while loading */
  static inline bool MeetsFilter(const EdgeMsg& m, const RunFilter& r) {
    return (m.to_run_id == -1) || common::utils::contains(r, m.to_run_id);
  }

  /** \brief String name for file saving */
  const std::string name() const;

 private:
  storage::LockableMessage::Ptr msg_;
};
}  // namespace pose_graph
}  // namespace vtr
