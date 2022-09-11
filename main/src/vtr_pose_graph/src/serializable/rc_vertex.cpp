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
 * \file rc_vertex.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_pose_graph/serializable/rc_vertex.hpp"

namespace vtr {
namespace pose_graph {

RCVertex::RCVertex(const VertexId &id, const Timestamp &vertex_time,
                   const Name2AccessorMapPtr &name2accessor_map)
    : VertexBase(id),
      BubbleInterface(name2accessor_map),
      vertex_time_(vertex_time),
      time_range_({vertex_time, vertex_time}) {
  const auto data = std::make_shared<VertexMsg>();
  msg_ = std::make_shared<storage::LockableMessage<VertexMsg>>(data);
}

RCVertex::RCVertex(const VertexMsg &msg,
                   const Name2AccessorMapPtr &name2accessor_map,
                   const storage::LockableMessage<VertexMsg>::Ptr &msg_ptr)
    : VertexBase(msg.id),
      BubbleInterface(name2accessor_map),
      vertex_time_(toTimestamp(msg.vertex_time)),
      time_range_(toTimestampRange(msg.time_range)),
      msg_(msg_ptr) {}

storage::LockableMessage<RCVertex::VertexMsg>::Ptr RCVertex::serialize() {
  std::stringstream ss;
  ss << "Vertex " << id_ << " -> ROS msg: ";

  bool changed = false;

  const auto msg_locked = msg_->locked();
  auto &msg_ref = msg_locked.get();
  auto data = msg_ref.getData();  // copy of current data

  if (data.id != (uint64_t)id_) {
    data.id = (uint64_t)id_;
    changed = true;
  }
  ss << "id: " << id_;

  std::shared_lock lock(mutex_);

  if (data.time_range.t1 != time_range_.first ||
      data.time_range.t2 != time_range_.second) {
    data.time_range.t1 = time_range_.first;
    data.time_range.t2 = time_range_.second;
    changed = true;
  }
  ss << ", stream time range set to <" << data.time_range.t1 << ","
     << data.time_range.t2 << ">";

  if (data.vertex_time.nanoseconds_since_epoch != vertex_time_) {
    data.vertex_time.nanoseconds_since_epoch = vertex_time_;
    changed = true;
  }
  ss << ", stream vertex time set to "
     << data.vertex_time.nanoseconds_since_epoch;

  lock.unlock();

  if (changed) msg_ref.setData(data);
  ss << ", vertex changed  " << changed;

  CLOG(DEBUG, "pose_graph") << ss.str();

  return msg_;
}

}  // namespace pose_graph
}  // namespace vtr
