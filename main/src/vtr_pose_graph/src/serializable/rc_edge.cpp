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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_pose_graph/serializable/rc_edge.hpp"

#include "vtr_logging/logging.hpp"

namespace vtr {
namespace pose_graph {

namespace {

// Cursor: Helper aliases and converters used only in this translation unit.
// Cursor: Rationale: keep the ROS <-> internal transform conversion logic local,
// Cursor: avoid exposing it in the public header, and ensure the implementation
// Cursor: remains lightweight (no heavy data is touched here).

using TransformT = EdgeTransform;
using TransformMsg = vtr_common_msgs::msg::LieGroupTransform;

// Cursor: Convert a ROS LieGroupTransform message to the lightweight internal
// Cursor: EdgeTransform representation.
// Cursor: - xi: 6D minimal representation (se(3)): [rx, ry, rz, tx, ty, tz]
// Cursor: - cov (optional): 6x6 covariance, flattened row-major
// Cursor: If covariance is not provided (cov_set=false), construct a transform
// Cursor: without covariance to keep memory/compute minimal.
TransformT fromMsg(const TransformMsg& msg) {
  using TransformVecT = Eigen::Matrix<double, 6, 1>;

  if (!msg.cov_set)
    return TransformT(TransformVecT(msg.xi.data()));
  else {
    Eigen::Matrix<double, 6, 6> cov;
    for (int row = 0; row < 6; ++row)
      for (int col = 0; col < 6; ++col) cov(row, col) = msg.cov[row * 6 + col];
    return TransformT(TransformVecT(msg.xi.data()), cov);
  }
}

// Cursor: Convert the internal EdgeTransform to a ROS LieGroupTransform message.
// Cursor: Only emit covariance if it has been set on the transform. This reduces
// Cursor: message size and downstream processing when uncertainties are not needed.
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

RCEdge::RCEdge(const VertexId& from_id, const VertexId& to_id,
               const EdgeType& type, const bool manual,
               const EdgeTransform& T_to_from)
    : EdgeBase(from_id, to_id, type, manual, T_to_from) {
  // Cursor: When constructing from in-memory values, initialize a lockable message
  // Cursor: holder with an empty ROS message. The actual message content is lazily
  // Cursor: synchronized in serialize(). This keeps the edge object lightweight and
  // Cursor: thread-safe for later streaming/storage.
  const auto data = std::make_shared<EdgeMsg>();
  msg_ = std::make_shared<storage::LockableMessage<EdgeMsg>>(data);
}

RCEdge::RCEdge(const EdgeMsg& msg,
               const storage::LockableMessage<EdgeMsg>::Ptr& msg_ptr)
    : EdgeBase(msg.from_id, msg.to_id,
               msg.type.type == EdgeTypeMsg::TEMPORAL ? EdgeType::Temporal
                                                      : EdgeType::Spatial,
               msg.mode.mode == EdgeModeMsg::MANUAL, fromMsg(msg.t_to_from)),
      msg_(msg_ptr) {}

storage::LockableMessage<RCEdge::EdgeMsg>::Ptr RCEdge::serialize() {
  // Cursor: Synchronize the internal edge state to its ROS message representation.
  // Cursor: This method is change-aware: it only writes back to the message if any
  // Cursor: field differs. This avoids unnecessary churn in storage/transport layers.
  bool changed = false;

  // Cursor: Acquire the lockable message for safe read-modify-write of its payload.
  const auto msg_locked = msg_->locked();
  auto& msg_ref = msg_locked.get();
  auto data = msg_ref.getData();  // copy of current data

  // potentially updated info
  // Cursor: Convert internal enum/bool types to ROS message enums/IDs.
  const auto type = static_cast<unsigned>(type_);
  const auto mode = manual_ ? EdgeModeMsg::MANUAL : EdgeModeMsg::AUTONOMOUS;
  const auto from_id = (uint64_t)from_;
  const auto to_id = (uint64_t)to_;

  if (data.type.type != type || data.mode.mode != mode ||
      data.from_id != from_id || data.to_id != to_id) {
    data.type.type = type;
    data.mode.mode = mode;
    data.from_id = from_id;
    data.to_id = to_id;
    changed = true;
  }

  // Cursor: Read lock the edge's internal state while examining the transform.
  // Cursor: Multiple readers can concurrently inspect the state; writers elsewhere
  // Cursor: will take an exclusive lock when modifying T_to_from_.
  std::shared_lock lock(mutex_);

  if (data.t_to_from.xi.empty() ||
      (data.t_to_from.cov_set != T_to_from_.covarianceSet())) {
    // Cursor: No transform yet, or covariance presence changed — write full transform.
    data.t_to_from = toMsg(T_to_from_);
    changed = true;
  } else {
    auto t_to_from = toMsg(T_to_from_);

    bool tf_approx_equal = true;

    // check if the transform is approximately equal
    using TFMap = Eigen::Map<Eigen::Matrix<double, 6, 1>>;
    tf_approx_equal &=
        TFMap(t_to_from.xi.data()).isApprox(TFMap(data.t_to_from.xi.data()));

    // check if the covariance is approximately equal
    if (T_to_from_.covarianceSet()) {
      using CovMap = Eigen::Map<Eigen::Matrix<double, 6, 6>>;
      tf_approx_equal &= CovMap(t_to_from.cov.data())
                             .isApprox(CovMap(data.t_to_from.cov.data()));
    }

    if (!tf_approx_equal) {
      // Cursor: Only update the message if values materially changed to reduce I/O.
      data.t_to_from = t_to_from;
      changed = true;
    }
  }

  lock.unlock();

  // Cursor: Commit the modified payload once, if anything changed under our lock.
  if (changed) msg_ref.setData(data);

  CLOG(DEBUG, "pose_graph") << "Edge " << id_ << " -> ROS msg: "
                            << "from: " << from_ << ", to: " << to_
                            << ", mode (0:auto, 1:manual): " << manual_
                            << ", type (0:temporal, 1:spatial): " << type
                            << ", T_to_from: " << T_to_from_.vec().transpose()
                            << ", edge changed " << changed;

  return msg_;
}

}  // namespace pose_graph
}  // namespace vtr
