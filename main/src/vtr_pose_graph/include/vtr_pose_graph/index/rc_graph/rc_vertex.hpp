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
 * \file rc_vertex.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_messages/msg/graph_persistent_id.hpp>
#include <vtr_messages/msg/graph_vertex.hpp>
#include <vtr_messages/msg/graph_vertex_header.hpp>
#include <vtr_pose_graph/index/vertex_base.hpp>
#include <vtr_pose_graph/interface/rc_stream_interface.hpp>

namespace vtr {
namespace pose_graph {

class RCVertex : public VertexBase, public RCStreamInterface {
 public:
  // Helper typedef to find the base class corresponding to edge data
  using Base = VertexBase;

  // Message typedefs, used for retreiving a message type for an arbitrary graph
  // object
  using Msg = vtr_messages::msg::GraphVertex;
  using HeaderMsg = vtr_messages::msg::GraphVertexHeader;

  using GraphPersistentIdMsg = vtr_messages::msg::GraphPersistentId;

  // Filter runs when loading
  using RunFilter = std::unordered_set<BaseIdType>;

  // We must explicitly pull one copy of these typedefs from the parent
  // namespaces to prevent ambiguity
  using LockableFieldMap = RCStreamInterface::LockableFieldMap;
  using LockableFieldMapPtr = RCStreamInterface::LockableFieldMapPtr;
  using LockableDataStreamMapPtr = RCStreamInterface::LockableDataStreamMapPtr;

  /** \brief Typedefs for shared pointers to vertices */
  PTR_TYPEDEFS(RCVertex);

  /**
   * \brief Interface to downcast base class pointers
   * \details This allows us to do DerivedPtrType = Type::Cast(BasePtrType)
   */
  PTR_DOWNCAST_OPS(RCVertex, VertexBase);

  /** \brief Typedefs for containers of vertices */
  CONTAINER_TYPEDEFS(RCVertex);

  /** \brief Pseudo-constructor for making shared pointers to vertices */
  static Ptr MakeShared() { return Ptr(new RCVertex()); }
  static Ptr MakeShared(const IdType& id) { return Ptr(new RCVertex(id)); }
  static Ptr MakeShared(const Msg& msg, const BaseIdType& runId,
                        const LockableFieldMapPtr& streamNames,
                        const LockableDataStreamMapPtr& streamMap) {
    return Ptr(new RCVertex(msg, runId, streamNames, streamMap));
  }

  /** \brief Default constructor */
  RCVertex() : VertexBase(), RCStreamInterface(){};
  RCVertex(const IdType& id) : VertexBase(id), RCStreamInterface(){};
  RCVertex(const Msg& msg, const BaseIdType& runId,
           const LockableFieldMapPtr& streamNames,
           const LockableDataStreamMapPtr& streamMap);

  /** \brief Destructor */
  virtual ~RCVertex() = default;

  /** \brief Serialize to a ros message */
  Msg toRosMsg();

  /** \brief Helper for run filtering while loading */
  static inline bool MeetsFilter(const Msg&, const RunFilter&) { return true; }

  /** \brief String name for file saving */
  const std::string name() const { return "vertex"; }

  /** \brief Get the persistent id that can survive graph refactors */
  const GraphPersistentIdMsg persistentId() const { return persistent_id_; }

  /** \brief Sets the persistent id that can survive graph refactors */
  void setPersistentId(const uint64_t& stamp, const uint32_t& robot) {
    persistent_id_.stamp = stamp;
    persistent_id_.robot = robot;
  }

 protected:
  // The persistent vertex id that can survive graph refactors
  GraphPersistentIdMsg persistent_id_;

  friend class RCGraph;
  template <typename V, typename E, typename R>
  friend class Graph;
};

}  // namespace pose_graph
}  // namespace vtr
