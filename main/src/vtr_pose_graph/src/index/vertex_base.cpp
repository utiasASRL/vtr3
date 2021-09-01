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
 * \file vertex_base.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_pose_graph/index/vertex_base.hpp>

namespace vtr {
namespace pose_graph {

const int VertexBase::transform_rows;
const int VertexBase::transform_cols;
const int VertexBase::transform_vdim;

VertexBase::Ptr VertexBase::MakeShared() { return Ptr(new VertexBase()); }

VertexBase::Ptr VertexBase::MakeShared(const IdType& id) {
  return Ptr(new VertexBase(id));
}

VertexBase::VertexBase()
    : id_(IdType()), neighbours_(VertexIdSetArray()), modified_(true) {}

VertexBase::VertexBase(const IdType& id)
    : id_(id), neighbours_(VertexIdSetArray()), modified_(true) {}

void VertexBase::addEdge(const EdgeIdType& e) {
  (void)neighbours_[size_t(e.type())].insert(e.id1() == id_ ? e.id2()
                                                            : e.id1());
  modified_ = modified_ || (e.type() == EdgeIdType::Type::Temporal);
}

void VertexBase::addEdge(const IdType& to, const EdgeIdType::Type& etype) {
  (void)neighbours_[size_t(etype)].insert(to);
  modified_ = modified_ || (etype == EdgeIdType::Type::Temporal);
}

void VertexBase::deleteEdge(const EdgeIdType& e) {
  (void)neighbours_[size_t(e.type())].erase(e.id1() == id_ ? e.id2() : e.id1());
  modified_ = modified_ || (e.type() == EdgeIdType::Type::Temporal);
}

void VertexBase::deleteEdge(const IdType& to, const EdgeIdType::Type& etype) {
  (void)neighbours_[size_t(etype)].erase(to);
  modified_ = modified_ || (etype == EdgeIdType::Type::Temporal);
}

VertexBase::EdgeIdType::Set VertexBase::incident() const {
  EdgeId::Set tmp;
  for (size_t i = 0; i < neighbours_.size(); ++i)
    for (auto jt = neighbours_[i].begin(); jt != neighbours_[i].end(); ++jt)
      tmp.insert(EdgeIdType(id_, *jt, EdgeIdEnumType(i)));

  return tmp;
}

VertexBase::EdgeIdType::Set VertexBase::incident(
    const EdgeIdType::Type& type) const {
  EdgeId::Set tmp;
  for (auto jt = neighbours_[size_t(type)].begin();
       jt != neighbours_[size_t(type)].end(); ++jt)
    tmp.insert(EdgeIdType(id_, *jt, type));

  return tmp;
}

VertexBase::EdgeIdType::Set VertexBase::temporalEdges() const {
  EdgeId::Set tmp;
  for (auto jt = neighbours_[size_t(EdgeIdType::Type::Temporal)].begin();
       jt != neighbours_[size_t(EdgeIdType::Type::Temporal)].end(); ++jt)
    tmp.insert(EdgeIdType(id_, *jt, EdgeIdType::Type::Temporal));

  return tmp;
}

VertexBase::EdgeIdType::Set VertexBase::spatialEdges() const {
  EdgeId::Set tmp;
  for (auto jt = neighbours_[size_t(EdgeIdType::Type::Spatial)].begin();
       jt != neighbours_[size_t(EdgeIdType::Type::Spatial)].end(); ++jt)
    tmp.insert(EdgeIdType(id_, *jt, EdgeIdType::Type::Spatial));

  return tmp;
}

bool VertexBase::isIncident(const EdgeIdType& e) const {
  return (
      neighbours_[size_t(e.type())].find(e.id1() == id_ ? e.id2() : e.id1()) !=
      neighbours_[size_t(e.type())].end());
}

VertexBase::IdType::Set VertexBase::neighbours() const {
  VertexId::Set tmp;
  for (auto it = neighbours_.begin(); it != neighbours_.end(); ++it)
    tmp.insert(it->begin(), it->end());

  return tmp;
}

const VertexBase::IdType::Set& VertexBase::neighbours(
    const EdgeIdType::Type& type) const {
  return neighbours_[size_t(type)];
}

const VertexBase::IdType::Set& VertexBase::temporalNeighbours() const {
  return neighbours_[size_t(EdgeIdType::Type::Temporal)];
}

const VertexBase::IdType::Set& VertexBase::spatialNeighbours() const {
  return neighbours_[size_t(EdgeIdType::Type::Spatial)];
}

bool VertexBase::isNeighbour(const IdType& v) const {
  for (auto it = neighbours_.begin(); it != neighbours_.end(); ++it)
    if (it->find(v) != it->end()) return true;
  return false;
}

bool VertexBase::isSpatialNeighbour(const IdType& v) const {
  return (neighbours_[size_t(EdgeIdType::Type::Spatial)].find(v) !=
          neighbours_[size_t(EdgeIdType::Type::Spatial)].end());
}

bool VertexBase::isTemporalNeighbour(const IdType& v) const {
  return (neighbours_[size_t(EdgeIdType::Type::Temporal)].find(v) !=
          neighbours_[size_t(EdgeIdType::Type::Temporal)].end());
}

bool VertexBase::isNeighbour(const IdType& v,
                             const EdgeIdType::Type& etype) const {
  return (neighbours_[size_t(etype)].find(v) !=
          neighbours_[size_t(etype)].end());
}

VertexBase::IdType VertexBase::id() const { return id_; }

VertexBase::SimpleIdType VertexBase::simpleId() const {
  return SimpleIdType(id_);
}

bool VertexBase::isModified() const { return modified_; }

void VertexBase::setModified(bool modified) { modified_ = modified; }

void VertexBase::setTransform(const TransformType& T) {
  T_vertex_world_.reset(new TransformType(T));
  modified_ = true;
}

std::ostream& operator<<(std::ostream& out, const VertexBase& v) {
  return out << v.id();
}

}  // namespace pose_graph
}  // namespace vtr
