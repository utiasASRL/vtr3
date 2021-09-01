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
 * \file id_base.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <stdint.h>
#include <iostream>

#include <vtr_common/utils/macros.hpp>

#define LOWER(a) uint32_t(a & 0x00000000FFFFFFFF)
#define UPPER(a) uint32_t(a >> 32)
#define COMBINE(a, b) (uint64_t(a) << 32) | uint64_t(b)

namespace vtr {
namespace pose_graph {

using BaseIdType = uint32_t;
using CombinedIdType = uint64_t;

class BaseId {
 public:
  using BaseIdPairType = std::pair<BaseIdType, BaseIdType>;

  constexpr BaseId(BaseIdType majorId = 0, BaseIdType minorId = 0)
      : majorId_(majorId), minorId_(minorId) {}
  constexpr BaseId(uint64_t comboId)
      : majorId_(comboId >> 32), minorId_(comboId) {}
  explicit BaseId(BaseIdPairType id)
      : majorId_(std::get<0>(id)), minorId_(std::get<1>(id)) {}

  /** \brief Invalid id value use to check initialization */
  static const uint64_t InvalidSimple = -1;

  /** \brief Check if the id is valid */
  bool isValid() const { return *this != BaseId(InvalidSimple); }

  /** \brief Hash operator for use in stl containers */
  inline size_t hash() const {
    return std::hash<uint64_t>()((uint64_t(majorId_) << 32) |
                                 uint64_t(minorId_));
  }

  /** \brief Cast to a single uint64_t */
  inline operator uint64_t() const {
    return ((uint64_t(majorId_) << 32) | uint64_t(minorId_));
  }

  /** \brief Comparison operator */
  inline bool operator==(const BaseId &rhs) const {
    return (this->majorId_ == rhs.majorId_) && (this->minorId_ == rhs.minorId_);
  }

  /**
   * \brief Less than operator
   * \details Many standard containers expect this function to exist
   */
  inline bool operator<(const BaseId &rhs) const {
    return (this->majorId_ < rhs.majorId_) ||
           (this->majorId_ == rhs.majorId_ && this->minorId_ < rhs.minorId_);
  }

  /** The rest of the comparisons, because users will expect them */
  inline bool operator!=(const BaseId &rhs) const { return !(operator==(rhs)); }
  inline bool operator>(const BaseId &rhs) const { return (rhs < *this); }
  inline bool operator<=(const BaseId &rhs) const { return !(operator>(rhs)); }
  inline bool operator>=(const BaseId &rhs) const { return !(operator<(rhs)); }

  /** \brief Get the run id */
  virtual inline BaseIdType majorId() const { return majorId_; }

  /** \brief Get the container id, within the run */
  virtual inline BaseIdType minorId() const { return minorId_; }

  /** \brief Get both ids as a pair */
  BaseIdPairType id() const { return BaseIdPairType(majorId_, minorId_); }

  /** \brief Returns true if the id values have been initialized */
  inline bool isSet() const {
    return ((majorId_ != BaseIdType(-1)) && (minorId_ != BaseIdType(-1)));
  }

  /** \brief String output */
  friend std::ostream &operator<<(std::ostream &out, const BaseId &id) {
    return out << "<" << id.majorId() << ',' << id.minorId() << ">";
  }

 protected:
  /** \brief the run id of the vertex */
  BaseIdType majorId_;
  /** \brief The container id of the vertex within the run */
  BaseIdType minorId_;

 private:
  // Don't allow implicit conversions for combo ids, must be exact type
  template <typename T,
            typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
  BaseId(T badComboId);
};

}  // namespace pose_graph
}  // namespace vtr

EXTEND_HASH(vtr::pose_graph::BaseId)

/** \brief Creates a new Id */
#define DEFINE_ID(IdTypeName)                                                 \
  class IdTypeName : public vtr::pose_graph::BaseId {                         \
   public:                                                                    \
    using Base = vtr::pose_graph::BaseId;                                     \
    CONTAINER_TYPEDEFS(IdTypeName);                                           \
    IdTypeName() : BaseId() {}                                                \
    constexpr IdTypeName(uint64_t comboId) : BaseId(comboId) {}               \
    IdTypeName(vtr::pose_graph::BaseIdType majorId,                           \
               vtr::pose_graph::BaseIdType minorId)                           \
        : BaseId(majorId, minorId) {}                                         \
    IdTypeName(BaseIdPairType id) : BaseId(id) {}                             \
    static constexpr IdTypeName Invalid() {                                   \
      return IdTypeName(vtr::pose_graph::BaseId::InvalidSimple);              \
    }                                                                         \
    IdTypeName &operator++() {                                                \
      ++this->minorId_;                                                       \
      return (*this);                                                         \
    }                                                                         \
    IdTypeName operator++(int) {                                              \
      IdTypeName tmp(*this);                                                  \
      ++(*this);                                                              \
      return tmp;                                                             \
    }                                                                         \
    IdTypeName &operator--() {                                                \
      --this->minorId_;                                                       \
      return (*this);                                                         \
    }                                                                         \
    IdTypeName operator--(int) {                                              \
      IdTypeName tmp(*this);                                                  \
      --(*this);                                                              \
      return tmp;                                                             \
    }                                                                         \
                                                                              \
   private:                                                                   \
    template <typename T, typename std::enable_if<std::is_integral<T>::value, \
                                                  int>::type = 0>             \
    IdTypeName(T badComboId);                                                 \
  };

namespace vtr {
namespace pose_graph {

template <class ENUM>
class BaseTypedId {
 public:
  using BaseIdPairType = std::pair<BaseIdType, BaseIdType>;
  using Type = ENUM;

  static constexpr inline uint32_t NumTypes() {
    return uint32_t(ENUM::UNDEFINED);
  };

  BaseTypedId(BaseIdType majorId = 0, BaseIdType minorId = 0,
              Type type = Type::UNDEFINED)
      : majorId_(majorId), minorId_(minorId), type_(type) {}
  BaseTypedId(BaseIdPairType id, Type type)
      : majorId_(std::get<0>(id)), minorId_(std::get<1>(id)), type_(type) {}

  /** \brief Hash operator for use in stl containers */
  inline size_t hash() const {
    return (std::hash<BaseIdType>()(BaseIdType(type_) << 3)) ^
           (std::hash<uint64_t>()((uint64_t(majorId_) << 32) |
                                  uint64_t(minorId_)));
  }

  /** \brief Cast to a single uint64_t */
  inline operator uint64_t() const {
    return ((uint64_t(majorId_) << 32) | uint64_t(minorId_));
  }

  /** \brief Get the run id */
  inline BaseIdType majorId() const { return majorId_; }

  /** \brief Get the container id, within the run */
  inline BaseIdType minorId() const { return minorId_; }

  /** \brief Get both ids as a pair */
  BaseIdPairType id() const { return BaseIdPairType(majorId_, minorId_); }

  /** \brief Returns true if the id values have been initialized */
  inline bool isSet() const { return (majorId_ > 0) && (minorId_ > 0); }

  /** \brief Get the type */
  inline Type type() const { return type_; }

  /** \brief Get the type as a size_t for indexing */
  inline size_t idx() const { return size_t(type_); }

  /** \brief String output */
  friend std::ostream &operator<<(std::ostream &out, const BaseTypedId &id) {
    return out << "<" << id.majorId() << ',' << id.minorId() << "," << id.type()
               << ">";
  }

  /** \brief Comparison operator */
  inline bool operator==(const BaseTypedId &rhs) const {
    return (this->majorId_ == rhs.majorId_) &&
           (this->minorId_ == rhs.minorId_) && (this->type_ == rhs.type_);
  }

  /**
   * \brief Less than operator
   * \details Many standard containers expect this function to exist.  We
   *          define (arbitrarily) that Ids are ordered first by type, then
   *          by major Id, then by minor Id.
   */
  inline bool operator<(const BaseTypedId &rhs) const {
    return ((this->majorId_ == rhs.majorId_ && this->minorId_ < rhs.minorId_) ||
            (this->majorId_ < rhs.majorId_)) ||
           (this->type_ < rhs.type_);
  }

  /** The rest of the comparisons, because users will expect them */
  inline bool operator!=(const BaseTypedId &rhs) const {
    return !operator==(rhs);
  }
  inline bool operator>(const BaseTypedId &rhs) const { return rhs < *this; }
  inline bool operator<=(const BaseTypedId &rhs) const {
    return !operator>(rhs);
  }
  inline bool operator>=(const BaseTypedId &rhs) const {
    return !operator<(rhs);
  }

 protected:
  /** \brief the run id of the item */
  BaseIdType majorId_;
  /** \brief The container id of the item within the run */
  BaseIdType minorId_;
  /** \brief Id type, to avoid unecessary polymorphism */
  Type type_;
};

}  // namespace pose_graph
}  // namespace vtr

EXTEND_HASH_TEMPLATED(vtr::pose_graph::BaseTypedId, ENUM)

/** \brief Creates a new Typed Id with a given list of types */
#define DEFINE_TYPED_ID(IdTypeName, T1, ...)                            \
  enum IdTypeName##EnumType{T1 = 0, ##__VA_ARGS__, UNDEFINED};          \
  class IdTypeName                                                      \
      : public vtr::pose_graph::BaseTypedId<IdTypeName##EnumType> {     \
   public:                                                              \
    using BASE = vtr::pose_graph::BaseTypedId<IdTypeName##EnumType>;    \
    CONTAINER_TYPEDEFS(IdTypeName);                                     \
    using ListArray = std::array<List, IdTypeName::NumTypes()>;         \
    using SetArray = std::array<Set, IdTypeName::NumTypes()>;           \
    using VectorArray = std::array<Vector, IdTypeName::NumTypes()>;     \
    using UnorderedSetArray =                                           \
        std::array<UnorderedSet, IdTypeName::NumTypes()>;               \
                                                                        \
    IdTypeName() : BaseTypedId() {}                                     \
    IdTypeName(vtr::pose_graph::BaseIdType majorId,                     \
               vtr::pose_graph::BaseIdType minorId, Type type)          \
        : BaseTypedId(majorId, minorId, type) {}                        \
    IdTypeName(BaseIdPairType id, Type type) : BaseTypedId(id, type) {} \
    IdTypeName &operator++() {                                          \
      ++this->minorId_;                                                 \
      return (*this);                                                   \
    }                                                                   \
    IdTypeName operator++(int) {                                        \
      IdTypeName tmp(*this);                                            \
      ++(*this);                                                        \
      return tmp;                                                       \
    }                                                                   \
    IdTypeName &operator--() {                                          \
      --this->minorId_;                                                 \
      return (*this);                                                   \
    }                                                                   \
    IdTypeName operator--(int) {                                        \
      IdTypeName tmp(*this);                                            \
      --(*this);                                                        \
      return tmp;                                                       \
    }                                                                   \
  };

namespace vtr {
namespace pose_graph {

template <class ENUM>
class BasePairedId {
 public:
  using SimpleIdType = std::pair<CombinedIdType, CombinedIdType>;
  using Type = ENUM;

  static constexpr inline uint32_t NumTypes() {
    return uint32_t(ENUM::UNDEFINED);
  };

  BasePairedId() : id_(SimpleIdType(-1, -1)), type_(Type::UNDEFINED) {}
  BasePairedId(const SimpleIdType &id, const Type &type)
      : id_(id), type_(type) {}
  BasePairedId(const CombinedIdType &id1, const CombinedIdType &id2,
               const Type &type)
      : id_(std::min(id1, id2), std::max(id1, id2)), type_(type) {}
  BasePairedId(const BaseIdType &runId, const BaseIdType &toRunId,
               const CombinedIdType &minor, const Type &type)
      : id_(COMBINE(toRunId, UPPER(minor)), COMBINE(runId, LOWER(minor))),
        type_(type) {}

  /** \brief Hash operator for use in stl containers */
  inline size_t hash() const {
    auto h64 = std::hash<uint64_t>();
    auto h32 = std::hash<uint32_t>();
    return (((h32(BaseIdType(type_)) << 3) ^ h64(id_.first)) << 3) ^
           h64(id_.second);
  }

  /** \brief Cast to a single uint64_t */
  inline operator SimpleIdType() const { return id_; }

  /** \brief Get the run id \todo (yuchen) This does not look correct. */
  inline BaseIdType majorId() const { return BaseIdType(id_.second >> 32); }

  /** \brief Get the run id */
  inline BaseIdType majorId1() const { return BaseIdType(id_.first >> 32); }

  /** \brief Get the run id */
  inline BaseIdType majorId2() const { return BaseIdType(id_.second >> 32); }

  /** \brief Get the container id, within the run */
  inline BaseIdType minorId() const {
    return (id_.first << 32) | (id_.second & 0x00000000FFFFFFFF);
  }

  /** \brief Get the container id, within the run */
  inline BaseIdType minorId1() const {
    return (id_.first & 0x00000000FFFFFFFF);
  }

  /** \brief Get the container id, within the run */
  inline BaseIdType minorId2() const {
    return (id_.second & 0x00000000FFFFFFFF);
  }

  /** \brief Get first id as a number */
  CombinedIdType id1() const { return id_.first; }

  /** \brief Get second id as a number */
  CombinedIdType id2() const { return id_.second; }

  /** \brief Returns true if the id values have been initialized */
  inline bool isSet() const {
    return (id_.first != CombinedIdType(-1)) &&
           (id_.second != CombinedIdType(-1));
  }

  /** \brief Get the type */
  inline Type type() const { return type_; }

  /** \brief Get the type as a size_t for indexing */
  inline size_t idx() const { return size_t(type_); }

  /** \brief String output */
  friend std::ostream &operator<<(std::ostream &out, const BasePairedId &id) {
    auto i1 = id.id1();
    auto i2 = id.id2();
    return out << "{<" << UPPER(i1) << ',' << LOWER(i1) << ">,<" << UPPER(i2)
               << "," << LOWER(i2) << ">," << id.idx() << "}";
  }

  /** \brief Comparison operator */
  inline bool operator==(const BasePairedId &rhs) const {
    return (this->id_ == rhs.id_) && (this->type_ == rhs.type_);
  }

  /**
   * \brief Less than operator
   * \details Many standard containers expect this function to exist.  We
   *          define (arbitrarily) that Ids are ordered first by type, then
   *          by major Id, then by minor Id.
   */
  inline bool operator<(const BasePairedId &rhs) const {
    return ((this->type_ == rhs.type_) && (this->id_ < rhs.id_)) ||
           (this->type_ < rhs.type_);
  }

  /** \brief The rest of the comparisons, because users will expect them */
  inline bool operator!=(const BasePairedId &rhs) const {
    return !operator==(rhs);
  }
  inline bool operator>(const BasePairedId &rhs) const { return rhs < *this; }
  inline bool operator<=(const BasePairedId &rhs) const {
    return !operator>(rhs);
  }
  inline bool operator>=(const BasePairedId &rhs) const {
    return !operator<(rhs);
  }

 protected:
  /** \brief The two ids of the pair */
  SimpleIdType id_;

  /** \brief Id type, to avoid unecessary polymorphism */
  Type type_;
};

}  // namespace pose_graph
}  // namespace vtr

EXTEND_HASH_TEMPLATED(vtr::pose_graph::BasePairedId, ENUM)

/** \brief Creates a new Typed Id with a given list of types */
#define DEFINE_PAIRED_ID(IdTypeName, T1, ...)                                  \
  enum IdTypeName##EnumType{T1 = 0, ##__VA_ARGS__, UNDEFINED};                 \
  class IdTypeName                                                             \
      : public vtr::pose_graph::BasePairedId<IdTypeName##EnumType> {           \
   public:                                                                     \
    using Base = vtr::pose_graph::BasePairedId<IdTypeName##EnumType>;          \
    CONTAINER_TYPEDEFS(IdTypeName);                                            \
    using ListArray = std::array<List, IdTypeName::NumTypes()>;                \
    using SetArray = std::array<Set, IdTypeName::NumTypes()>;                  \
    using VectorArray = std::array<Vector, IdTypeName::NumTypes()>;            \
    using UnorderedSetArray =                                                  \
        std::array<UnorderedSet, IdTypeName::NumTypes()>;                      \
                                                                               \
    IdTypeName() : BasePairedId() {}                                           \
    IdTypeName(const vtr::pose_graph::CombinedIdType &id1,                     \
               const vtr::pose_graph::CombinedIdType &id2, const Type &type)   \
        : BasePairedId(id1, id2, type) {}                                      \
    IdTypeName(const vtr::pose_graph::BaseIdType &runId,                       \
               const vtr::pose_graph::BaseIdType &toRunId,                     \
               const vtr::pose_graph::CombinedIdType &minor, const Type &type) \
        : BasePairedId(runId, toRunId, minor, type) {}                         \
    IdTypeName(const typename Base::SimpleIdType &id, const Type &type)        \
        : BasePairedId(id, type) {}                                            \
    IdTypeName &operator++() {                                                 \
      ++this->id_.first;                                                       \
      return (*this);                                                          \
    }                                                                          \
    IdTypeName operator++(int) {                                               \
      ++this->id_.second;                                                      \
      return (*this);                                                          \
    }                                                                          \
    IdTypeName &operator--() {                                                 \
      --this->id_.first;                                                       \
      return (*this);                                                          \
    }                                                                          \
    IdTypeName operator--(int) {                                               \
      --this->id_.second;                                                      \
      return (*this);                                                          \
    }                                                                          \
  };
