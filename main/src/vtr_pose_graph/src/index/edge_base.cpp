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
 * \file edge_base.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_pose_graph/index/edge_base.hpp"

#include "vtr_logging/logging.hpp"

namespace vtr {
namespace pose_graph {

EdgeBase::Ptr EdgeBase::MakeShared(const VertexId& from_id,
                                   const VertexId& to_id, const EdgeType& type,
                                   const EdgeMode& mode,
                                   const EdgeTransform& T_to_from) {
  return std::make_shared<EdgeBase>(from_id, to_id, type, mode, T_to_from);
}

EdgeBase::EdgeBase(const VertexId& from_id, const VertexId& to_id,
                   const EdgeType& type, const EdgeMode& mode,
                   const EdgeTransform& T_to_from)
    : id_(from_id, to_id),
      from_(from_id),
      to_(to_id),
      type_(type),
      mode_(mode),
      T_to_from_(T_to_from) {
  // if (from_.majorId() < to_.majorId()) {
  //   CLOG(ERROR, "pose_graph")
  //       << "Cannot create edge from " << from_ << " to " << to_
  //       << " since the major id of the from vertex is smaller than the to "
  //          "vertex";
  //   throw std::invalid_argument(
  //       "Spatial edges may only be added from higher run numbers to lower "
  //       "ones");
  // }
}

EdgeId EdgeBase::id() const { return id_; }
VertexId EdgeBase::from() const { return from_; }
VertexId EdgeBase::to() const { return to_; }
EdgeType EdgeBase::type() const { return type_; }
size_t EdgeBase::idx() const { return (size_t)type_; }
bool EdgeBase::isManual() const { return mode_ == EdgeMode::Manual; }
bool EdgeBase::isAutonomous() const { return mode_ == EdgeMode::Autonomous; }
bool EdgeBase::isUnknown() const { return mode_ == EdgeMode::Unknown; }
bool EdgeBase::isTemporal() const { return type_ == EdgeType::Temporal; }
bool EdgeBase::isSpatial() const { return type_ == EdgeType::Spatial; }

EdgeTransform EdgeBase::T() const {
  std::shared_lock lock(mutex_);
  return T_to_from_;
}

void EdgeBase::setTransform(const EdgeTransform& T_to_from) {
  std::unique_lock lock(mutex_);
  T_to_from_ = T_to_from;
}

std::ostream& operator<<(std::ostream& out, const EdgeBase& e) {
  return out << "{" << e.from() << "--" << e.to() << "}" << " mode " << static_cast<unsigned>(e.mode_) << " type " << static_cast<unsigned>(e.type_);
}

}  // namespace pose_graph
}  // namespace vtr
