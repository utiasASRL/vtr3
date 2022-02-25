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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/index/vertex_base.hpp"
#include "vtr_pose_graph/serializable/bubble_interface.hpp"
#include "vtr_storage/stream/message.hpp"

#include "vtr_pose_graph_msgs/msg/vertex.hpp"

#include "vtr_pose_graph_msgs/msg/timestamp.hpp"
#include "vtr_pose_graph_msgs/msg/timestamp_range.hpp"

namespace vtr {
namespace pose_graph {

using TimestampMsg = vtr_pose_graph_msgs::msg::Timestamp;
using TimestampRangeMsg = vtr_pose_graph_msgs::msg::TimestampRange;

inline Timestamp toTimestamp(const TimestampMsg& time) {
  return static_cast<Timestamp>(time.nanoseconds_since_epoch);
}

inline TimestampMsg toTimestampMsg(const Timestamp& time) {
  TimestampMsg msg;
  msg.nanoseconds_since_epoch = time;
  return msg;
}

inline TimestampRange toTimestampRange(const TimestampRangeMsg& time_range) {
  return {static_cast<Timestamp>(time_range.t1),
          static_cast<Timestamp>(time_range.t2)};
}

inline TimestampRangeMsg toTimestampRangeMsg(const TimestampRange& time_range) {
  TimestampRangeMsg msg;
  msg.t1 = time_range.first;
  msg.t2 = time_range.second;
  return msg;
}

class RCVertex : public VertexBase, public BubbleInterface {
 public:
  /** \brief Typedefs for shared pointers to vertices */
  PTR_TYPEDEFS(RCVertex);

  // Helper typedef to find the base class corresponding to edge data
  using Base = VertexBase;

  // ROS message for serialization
  using VertexMsg = vtr_pose_graph_msgs::msg::Vertex;

  static Ptr MakeShared(const VertexId& id, const Timestamp& vertex_time,
                        const Name2AccessorMapPtr& name2accessor_map) {
    return std::make_shared<RCVertex>(id, vertex_time, name2accessor_map);
  }

  static Ptr MakeShared(
      const VertexMsg& msg, const Name2AccessorMapPtr& name2accessor_map,
      const storage::LockableMessage<VertexMsg>::Ptr& msg_ptr) {
    return std::make_shared<RCVertex>(msg, name2accessor_map, msg_ptr);
  }

  /** \brief Construct a new serializable vertex. */
  RCVertex(const VertexId& id, const Timestamp& vertex_time,
           const Name2AccessorMapPtr& name2accessor_map);

  /** \brief Load a vertex from disk. */
  RCVertex(const VertexMsg& msg, const Name2AccessorMapPtr& name2accessor_map,
           const storage::LockableMessage<VertexMsg>::Ptr& msg_ptr);

  /** \brief Destructor */
  virtual ~RCVertex() = default;

  /** \brief Serialize to a ros message */
  storage::LockableMessage<VertexMsg>::Ptr serialize();

  Timestamp vertexTime() const {
    std::shared_lock lock(mutex_);
    return vertex_time_;
  }

  TimestampRange timeRange() const {
    std::shared_lock lock(mutex_);
    return time_range_;
  }

  /// Stream interface
  template <typename DataType>
  typename storage::LockableMessage<DataType>::Ptr retrieve(
      const std::string& stream_name, const std::string& stream_type) {
    std::shared_lock lock(mutex_);
    if (vertex_time_ == storage::NO_TIMESTAMP_VALUE) return nullptr;
    return BubbleInterface::retrieve<DataType>(stream_name, stream_type,
                                               vertex_time_);
  }

  template <typename DataType>
  typename storage::LockableMessage<DataType>::Ptr retrieve(
      const std::string& stream_name, const std::string& stream_type,
      const Timestamp& time) {
    return BubbleInterface::retrieve<DataType>(stream_name, stream_type, time);
  }

  template <typename DataType>
  std::vector<typename storage::LockableMessage<DataType>::Ptr> retrieve(
      const std::string& stream_name, const std::string& stream_type,
      const Timestamp& start, const Timestamp& stop) {
    return BubbleInterface::retrieve<DataType>(stream_name, stream_type, start,
                                               stop);
  }

  template <typename DataType>
  bool insert(const std::string& stream_name, const std::string& stream_type,
              const typename storage::LockableMessage<DataType>::Ptr& message) {
    std::unique_lock lock(mutex_);
    updateTimestampRange(message->locked().get().getTimestamp());
    return BubbleInterface::insert<DataType>(stream_name, stream_type, message);
  }

 private:
  void updateTimestampRange(const Timestamp& time) {
    time_range_.first = time_range_.first == storage::NO_TIMESTAMP_VALUE
                            ? time
                            : std::min(time_range_.first, time);
    time_range_.second = time_range_.second == storage::NO_TIMESTAMP_VALUE
                             ? time
                             : std::max(time_range_.second, time);
  }

 private:
  /** \brief protects access to vertex time, time range */
  using VertexBase::mutex_;

  /** \brief The vertex time associated with this vertex. */
  Timestamp vertex_time_ = storage::NO_TIMESTAMP_VALUE;

  /** \brief Time range associated with this vertex for all data. */
  TimestampRange time_range_{storage::NO_TIMESTAMP_VALUE,
                             storage::NO_TIMESTAMP_VALUE};

  storage::LockableMessage<VertexMsg>::Ptr msg_;
};

}  // namespace pose_graph
}  // namespace vtr
