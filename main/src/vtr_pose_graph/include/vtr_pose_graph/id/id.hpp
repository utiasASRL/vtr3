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
 * \file base_id.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <stdint.h>
#include <iostream>

#include "vtr_common/utils/macros.hpp"

namespace vtr {
namespace pose_graph {

using BaseIdType = uint32_t;
static constexpr BaseIdType InvalidBaseId = BaseIdType(-1);
using CombinedIdType = uint64_t;
static constexpr CombinedIdType InvalidCombinedId = CombinedIdType(-1);

class VertexId {
 public:
  CONTAINER_TYPEDEFS(VertexId);

  static constexpr VertexId Invalid() { return VertexId(); }

  constexpr VertexId() = default;
  constexpr VertexId(const BaseIdType &major_id, const BaseIdType &minor_id)
      : major_id_(major_id), minor_id_(minor_id) {}
  constexpr VertexId(const CombinedIdType &combined_id)
      : major_id_(upper(combined_id)), minor_id_(lower(combined_id)) {}

  operator CombinedIdType() const { return combine(major_id_, minor_id_); }

  /** \brief Check if the id is valid */
  bool isValid() const {
    return (major_id_ != InvalidBaseId) && (minor_id_ != InvalidBaseId);
  }

  /** \brief Hash operator for use in stl containers */
  size_t hash() const {
    return std::hash<uint64_t>()(combine(major_id_, minor_id_));
  }

  /** \brief Comparison operators */
  bool operator==(const VertexId &rhs) const {
    return (major_id_ == rhs.major_id_) && (minor_id_ == rhs.minor_id_);
  }
  bool operator<(const VertexId &rhs) const {
    return (major_id_ < rhs.major_id_) ||
           (major_id_ == rhs.major_id_ && minor_id_ < rhs.minor_id_);
  }
  bool operator!=(const VertexId &rhs) const { return !(operator==(rhs)); }
  bool operator>(const VertexId &rhs) const { return (rhs < *this); }
  bool operator<=(const VertexId &rhs) const { return !(operator>(rhs)); }
  bool operator>=(const VertexId &rhs) const { return !(operator<(rhs)); }

  /** \brief Get the run id */
  BaseIdType majorId() const { return major_id_; }
  /** \brief Get the container id, within the run */
  BaseIdType minorId() const { return minor_id_; }

  VertexId &operator++() {
    ++minor_id_;
    return *this;
  }

  VertexId operator++(int) {
    VertexId tmp(*this);
    ++(*this);
    return tmp;
  }

  VertexId &operator--() {
    --minor_id_;
    return *this;
  }

  VertexId operator--(int) {
    VertexId tmp(*this);
    --(*this);
    return tmp;
  }

  /** \brief String output */
  friend std::ostream &operator<<(std::ostream &out, const VertexId &id) {
    return out << "<" << id.major_id_ << ',' << id.minor_id_ << ">";
  }

 private:
  static constexpr BaseIdType lower(const CombinedIdType &value) {
    return (BaseIdType)(value & 0x00000000FFFFFFFF);
  }

  static constexpr BaseIdType upper(const CombinedIdType &value) {
    return (BaseIdType)(value >> 32);
  }

  static constexpr CombinedIdType combine(const BaseIdType &upper_value,
                                          const BaseIdType &lower_value) {
    return (CombinedIdType(upper_value) << 32) | CombinedIdType(lower_value);
  }

 protected:
  BaseIdType major_id_ = InvalidBaseId;
  BaseIdType minor_id_ = InvalidBaseId;

 private:
  // Don't allow implicit conversions for combined ids, must be exact type
  template <typename T,
            typename std::enable_if_t<std::is_integral_v<T>, int> = 0>
  VertexId(const T &bad_combined_id);
};

class EdgeId {
 public:
  CONTAINER_TYPEDEFS(EdgeId);

  using PairedIdType = std::pair<VertexId, VertexId>;

  static constexpr EdgeId Invalid() { return EdgeId(); }

  constexpr EdgeId() = default;
  /// \note we order the ids in the constructor
  constexpr EdgeId(const VertexId &id1, const VertexId &id2)
      : id_(std::min(id1, id2), std::max(id1, id2)) {}

  operator PairedIdType() const { return id_; }

  /** \brief Check if the id is valid */
  bool isValid() const { return id_.first.isValid() && id_.second.isValid(); }

  /** \brief Hash operator for use in stl containers */
  size_t hash() const { return id_.first.hash() ^ id_.second.hash(); }

  /**
   * \brief Comparison operators
   * \details Many standard containers expect this function to exist.  We
   *          define (arbitrarily) that Ids are ordered first by type, then
   *          by major Id, then by minor Id.
   */
  bool operator<(const EdgeId &rhs) const { return id_ < rhs.id_; }
  bool operator==(const EdgeId &rhs) const { return id_ == rhs.id_; }
  bool operator!=(const EdgeId &rhs) const { return !operator==(rhs); }
  bool operator>(const EdgeId &rhs) const { return rhs < *this; }
  bool operator<=(const EdgeId &rhs) const { return !operator>(rhs); }
  bool operator>=(const EdgeId &rhs) const { return !operator<(rhs); }

  VertexId id1() const { return id_.first; }
  BaseIdType majorId1() const { return id_.first.majorId(); }
  BaseIdType minorId1() const { return id_.first.minorId(); }

  VertexId id2() const { return id_.second; }
  BaseIdType majorId2() const { return id_.second.majorId(); }
  BaseIdType minorId2() const { return id_.second.minorId(); }

  /** \brief String output */
  friend std::ostream &operator<<(std::ostream &out, const EdgeId &id) {
    return out << "{<" << id.majorId1() << ',' << id.minorId1() << ">,<"
               << id.majorId2() << "," << id.minorId2() << ">"
               << "}";
  }

 private:
  PairedIdType id_ = std::make_pair(VertexId::Invalid(), VertexId::Invalid());
};

}  // namespace pose_graph
}  // namespace vtr

EXTEND_HASH(vtr::pose_graph::VertexId);
EXTEND_HASH(vtr::pose_graph::EdgeId);