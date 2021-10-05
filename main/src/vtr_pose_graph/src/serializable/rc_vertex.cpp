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
 * \brief
 * \details
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_pose_graph/serializable/rc_vertex.hpp>

namespace vtr {
namespace pose_graph {

RCVertex::RCVertex(const IdType &id, const Timestamp &keyframe_time,
                   const Name2AccessorMapPtr &name2accessor_map)
    : VertexBase(id),
      BubbleInterface(name2accessor_map),
      keyframe_time_(keyframe_time),
      time_range_({keyframe_time, keyframe_time}) {
  const auto data = std::make_shared<VertexMsg>();
  msg_ = std::make_shared<storage::LockableMessage>(data);
}

RCVertex::RCVertex(const VertexMsg &msg, const BaseIdType &runId,
                   const Name2AccessorMapPtr &name2accessor_map,
                   const storage::LockableMessage::Ptr &msg_ptr)
    : VertexBase(IdType(runId, msg.id)),
      BubbleInterface(name2accessor_map),
      keyframe_time_(toTimestamp(msg.keyframe_time)),
      time_range_(toTimestampRange(msg.time_range)),
      msg_(msg_ptr) {
  const auto &transform = msg.t_vertex_world;
  if (!transform.entries.size()) return;
  if (transform.entries.size() != transform_vdim) {
    CLOG(ERROR, "pose_graph")
        << "Expected serialized transform vector to be of size "
        << transform_vdim << " actual: " << transform.entries.size();
    return;
  }

  if (!msg.t_vertex_world_cov.entries.size()) {
    setTransform(TransformType(TransformVecType(transform.entries.data())));
    return;
  }

  const auto &transform_cov = msg.t_vertex_world_cov;
  Eigen::Matrix<double, transform_vdim, transform_vdim> cov;
  if (transform_cov.entries.size() != (unsigned)cov.size()) {
    CLOG(ERROR, "pose_graph")
        << "Expected serialized covariance to be of size " << cov.size();
    return;
  }
  for (int row = 0; row < transform_vdim; ++row)
    for (int col = 0; col < transform_vdim; ++col)
      cov(row, col) = transform_cov.entries[row * transform_vdim + col];
  setTransform(TransformType(TransformVecType(transform.entries.data()), cov));
}

storage::LockableMessage::Ptr RCVertex::serialize() {
  std::stringstream ss;
  ss << "Vertex " << id_ << " -> ROS msg: ";

  bool changed = false;

  const auto msg_locked = msg_->locked();
  auto &msg_ref = msg_locked.get();
  auto data = msg_ref.getData<VertexMsg>();  // copy of current data

  if (data.id != id_.minorId()) {
    data.id = id_.minorId();
    changed = true;
  }
  ss << "id: " << id_.minorId();

  std::shared_lock lock(T_vertex_world_mutex_);
  if (T_vertex_world_) {
    // set the transform
    TransformVecType vec(T_vertex_world_->vec());
    std::vector<double> T_vertex_world_vec;
    for (int row = 0; row < transform_vdim; ++row)
      T_vertex_world_vec.push_back(vec(row));
    if (data.t_vertex_world.entries != T_vertex_world_vec) {
      data.t_vertex_world.entries = T_vertex_world_vec;
      changed = true;
    }
    ss << ", T_vertex_world set";

    // save the covariance
    if (T_vertex_world_->covarianceSet() == true) {
      std::vector<double> T_vertex_world_cov_vec;
      for (int row = 0; row < 6; row++)
        for (int col = 0; col < 6; col++)
          T_vertex_world_cov_vec.push_back(T_vertex_world_->cov()(row, col));

      if (data.t_vertex_world_cov.entries != T_vertex_world_cov_vec) {
        data.t_vertex_world_cov.entries = T_vertex_world_cov_vec;
        changed = true;
      }
      ss << ", T_vertex_world_cov set";
    }
  }
  lock.unlock();

  std::shared_lock lock2(data_time_mutex_);
  if (data.time_range.t1 != time_range_.first ||
      data.time_range.t2 != time_range_.second) {
    data.time_range.t1 = time_range_.first;
    data.time_range.t2 = time_range_.second;
    changed = true;
  }
  ss << ", stream time range set to <" << data.time_range.t1 << ","
     << data.time_range.t2 << ">";

  if (data.keyframe_time.nanoseconds_since_epoch != keyframe_time_) {
    data.keyframe_time.nanoseconds_since_epoch = keyframe_time_;
    changed = true;
  }
  ss << ", stream keyframe time set to "
     << data.keyframe_time.nanoseconds_since_epoch;
  lock2.unlock();

  if (changed) msg_ref.setData<VertexMsg>(data);
  ss << ", vertex changed  " << changed;

  CLOG(DEBUG, "pose_graph") << ss.str();

  return msg_;
}

}  // namespace pose_graph
}  // namespace vtr
