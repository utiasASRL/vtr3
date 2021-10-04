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
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_pose_graph/index/edge_base.hpp>

namespace vtr {
namespace pose_graph {

const int EdgeBase::transform_rows;
const int EdgeBase::transform_cols;
const int EdgeBase::transform_vdim;

EdgeBase::Ptr EdgeBase::MakeShared(const VertexIdType& from_id,
                                   const VertexIdType& to_id,
                                   const EnumType& type, bool manual) {
  return Ptr(new EdgeBase(from_id, to_id, type, manual));
}

EdgeBase::Ptr EdgeBase::MakeShared(const VertexIdType& from_id,
                                   const VertexIdType& to_id,
                                   const EnumType& type,
                                   const TransformType& T_to_from,
                                   bool manual) {
  return Ptr(new EdgeBase(from_id, to_id, type, T_to_from, manual));
}

EdgeBase::EdgeBase(const VertexIdType& from_id, const VertexIdType& to_id,
                   const EnumType& type, bool manual)
    : id_(IdType(from_id, to_id, type)),
      from_(from_id),
      to_(to_id),
      manual_(manual) {}

EdgeBase::EdgeBase(const VertexIdType& from_id, const VertexIdType& to_id,
                   const EnumType& type, const TransformType& T_to_from,
                   bool manual)
    : id_(IdType(from_id, to_id, type)),
      from_(from_id),
      to_(to_id),
      T_to_from_(T_to_from),
      manual_(manual) {}

EdgeBase::IdType EdgeBase::id() const { return id_; }
EdgeBase::SimpleIdType EdgeBase::simpleId() const { return id_; }
EdgeBase::IdType::Type EdgeBase::type() const { return id_.type(); }
size_t EdgeBase::idx() const { return id_.idx(); }

EdgeBase::VertexIdType::Pair EdgeBase::incident() const {
  return VertexIdType::Pair(from_, to_);
}

EdgeBase::VertexIdType EdgeBase::from() const { return from_; }

EdgeBase::VertexIdType EdgeBase::to() const { return to_; }

EdgeBase::TransformType EdgeBase::T() const {
  std::shared_lock lock(mutex_);
  return T_to_from_;
}

bool EdgeBase::isManual() const { return manual_; }

bool EdgeBase::isAutonomous() const { return !manual_; }

bool EdgeBase::isTemporal() const {
  return id_.type() == IdType::Type::Temporal;
}

bool EdgeBase::isSpatial() const { return id_.type() == IdType::Type::Spatial; }

bool EdgeBase::isIncident(const VertexIdType& v) const {
  return (from_ == v) || (to_ == v);
}

void EdgeBase::setTransform(const TransformType& transform) {
  std::unique_lock lock(mutex_);
  T_to_from_ = transform;
}

#if false
void EdgeBase::setManual(bool manual) { manual_ = manual; }

void EdgeBase::setAutonomous(bool autonomous) { manual_ = !autonomous; }

void EdgeBase::setFrom(const VertexIdType& from_id) {
  from_ = from_id;
  id_ = IdType(from_, to_, id_.type());
}

void EdgeBase::setTo(const VertexIdType& to_id) {
  to_ = to_id;
  id_ = IdType(from_, to_, id_.type());
}
#endif

std::ostream& operator<<(std::ostream& out, const EdgeBase& e) {
  if (e.type() == EdgeBase::IdType::Type::Spatial)
    return out << "{" << e.from() << "--" << e.to() << "}";
  else if (e.type() == EdgeBase::IdType::Type::Temporal)
    return out << "{" << e.from() << "==" << e.to() << "}";
  else
    return out << "{" << e.from() << "??" << e.to() << "}";
}

}  // namespace pose_graph
}  // namespace vtr
