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
 * \file rc_edge.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_logging/logging.hpp>
#include <vtr_pose_graph/serializable/rc_edge.hpp>

namespace vtr {
namespace pose_graph {

RCEdge::RCEdge(const VertexId& from_id, const VertexId& to_id,
               const EnumType& type, bool manual)
    : EdgeBase(from_id, to_id, type, manual) {
  const auto data = std::make_shared<EdgeMsg>();
  msg_ = std::make_shared<storage::LockableMessage>(data);
}

RCEdge::RCEdge(const VertexId& from_id, const VertexId& to_id,
               const EnumType& type, const TransformType& T_to_from,
               bool manual)
    : EdgeBase(from_id, to_id, type, T_to_from, manual) {
  const auto data = std::make_shared<EdgeMsg>();
  msg_ = std::make_shared<storage::LockableMessage>(data);
}

RCEdge::RCEdge(const EdgeMsg& msg, BaseIdType run_id,
               const storage::LockableMessage::Ptr& msg_ptr)
    : EdgeBase(
          VertexId(run_id, msg.from_id),
          VertexId(msg.to_run_id == -1 ? run_id : msg.to_run_id, msg.to_id),
          msg.to_run_id == -1 ? IdType::Type::Temporal : IdType::Type::Spatial,
          msg.mode.mode == EdgeModeMsg::MANUAL),
      msg_(msg_ptr) {
  const auto& transform = msg.t_to_from;
  if (!transform.entries.size()) return;
  if (transform.entries.size() != transform_vdim) {
    CLOG(ERROR, "pose_graph")
        << "Expected serialized transform vector to be of size "
        << transform_vdim << " actual: " << transform.entries.size();
    throw;
  }

  if (!msg.t_to_from_cov.entries.size()) {
    setTransform(TransformType(TransformVecType(transform.entries.data())));
    return;
  }

  const auto& transform_cov = msg.t_to_from_cov;
  Eigen::Matrix<double, transform_vdim, transform_vdim> cov;
  if (transform_cov.entries.size() != (unsigned)cov.size()) {
    CLOG(ERROR, "pose_graph")
        << "Expected serialized covariance to be of size " << cov.size();
    throw;
  }
  for (int row = 0; row < transform_vdim; ++row)
    for (int col = 0; col < transform_vdim; ++col)
      cov(row, col) = transform_cov.entries[row * transform_vdim + col];
  setTransform(TransformType(TransformVecType(transform.entries.data()), cov));
}

storage::LockableMessage::Ptr RCEdge::serialize() {
  std::stringstream ss;
  ss << "Edge " << id_ << " -> ROS msg: ";

  bool changed = false;

  const auto msg_locked = msg_->locked();
  auto& msg_ref = msg_locked.get();
  auto data = msg_ref.getData<EdgeMsg>();  // copy of current data

  // potentially updated info
  const auto type = static_cast<unsigned>(id_.type());
  const auto mode = manual_ ? EdgeModeMsg::MANUAL : EdgeModeMsg::AUTONOMOUS;
  const auto from_id = from_.minorId();
  const auto to_id = to_.minorId();
  const auto to_run_id = id_.type() == IdType::Type::Spatial
                             ? static_cast<int>(to_.majorId())
                             : -1;

  if (data.type.type != type || data.mode.mode != mode ||
      data.from_id != from_id || data.to_id != to_id ||
      data.to_run_id != to_run_id) {
    data.type.type = type;
    data.mode.mode = mode;
    data.from_id = from_id;
    data.to_id = to_id;
    data.to_run_id = to_run_id;
    changed = true;
  }
  ss << "from_id: " << from_.minorId() << ", to_id: " << to_.minorId()
     << ", mode (0:auto, 1:manual): " << manual_
     << ", type (0:temporal, 1:spatial): " << type;

  std::shared_lock lock(mutex_);
  // set the transform
  TransformVecType vec(T_to_from_.vec());
  std::vector<double> T_to_from_vec;
  for (int row = 0; row < transform_vdim; ++row)
    T_to_from_vec.push_back(vec(row));
  if (data.t_to_from.entries != T_to_from_vec) {
    data.t_to_from.entries = T_to_from_vec;
    changed = true;
  }
  ss << ", T_to_from set";

  // save the covariance
  if (T_to_from_.covarianceSet() == true) {
    std::vector<double> T_to_from_cov_vec;
    for (int row = 0; row < 6; row++)
      for (int col = 0; col < 6; col++)
        T_to_from_cov_vec.push_back(T_to_from_.cov()(row, col));

    if (data.t_to_from_cov.entries != T_to_from_cov_vec) {
      data.t_to_from_cov.entries = T_to_from_cov_vec;
      changed = true;
    }
    ss << ", T_to_from_cov set";
  }
  lock.unlock();

  if (changed) msg_ref.setData<EdgeMsg>(data);
  ss << ", edge changed  " << changed;

#if false
  CLOG(DEBUG, "pose_graph") << ss.str();
#else
  CLOG(DEBUG, "pose_graph")
      << "Edge " << id_ << " -> ROS msg: "
      << "from_id: " << data.from_id << ", to_id: " << data.to_id
      << ", to_run_id: " << data.to_run_id
      << ", mode (0:auto, 1:manual): " << manual_
      << ", type (0:temporal, 1:spatial): " << type
      << ", T_to_from: " << data.t_to_from.entries << ", edge changed "
      << changed;
#endif
  return msg_;
}

const std::string RCEdge::name() const {
  if (id_.type() == IdType::Type::Temporal)
    return "temporal";
  else if (id_.type() == IdType::Type::Spatial)
    return "spatial";
  else
    return "unknown";
}

}  // namespace pose_graph
}  // namespace vtr
