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
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_pose_graph/index/rc_graph/rc_vertex.hpp>

namespace vtr {
namespace pose_graph {

RCVertex::RCVertex(const Msg &msg, const BaseIdType &runId,
                   const LockableFieldMapPtr &stream_names,
                   const LockableDataStreamMapPtr &stream_map)
    : VertexBase(IdType(runId, msg.id)),
      RCStreamInterface(msg.stream_time, stream_names, stream_map,
                        msg.stream_idx) {
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

RCVertex::Msg RCVertex::toRosMsg() {
  std::stringstream ss;
  ss << "Vertex " << id_ << " -> ROS msg: ";
  ss << "id: " << id_.minorId();

  Msg msg;

  msg.id = id_.minorId();

  /// serializeStreams(msg->mutable_streamtime(), msg->mutable_streamidx());
  auto [stream_time, stream_idx] = serializeStreams();
  msg.stream_time = stream_time;
  msg.stream_idx = stream_idx;

  ss << ", stream time and index set";

  // set the transform
  if (T_vertex_world_ != nullptr) {
    TransformVecType vec(T_vertex_world_->vec());
    // \todo make this an eigen map somehow...
    for (int row = 0; row < transform_vdim; ++row)
      msg.t_vertex_world.entries.push_back(vec(row));

    ss << ", T_vertex_world set";

    // save the covariance
    if (T_vertex_world_->covarianceSet() == true) {
      for (int row = 0; row < 6; row++)
        for (int col = 0; col < 6; col++)
          msg.t_vertex_world_cov.entries.push_back(
              T_vertex_world_->cov()(row, col));

      ss << ", T_vertex_world_cov set";
    }

    modified_ = false;
  }

  CLOG(DEBUG, "pose_graph") << ss.str();

  return msg;
}

}  // namespace pose_graph
}  // namespace vtr
