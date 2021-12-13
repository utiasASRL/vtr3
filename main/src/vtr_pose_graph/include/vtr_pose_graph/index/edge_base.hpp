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
 * \file edge_base.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <shared_mutex>

#include "lgmath.hpp"
#include "vtr_pose_graph/id/id.hpp"

namespace vtr {
namespace pose_graph {

class EdgeBase {
 public:
  // Typedef the id and SimpleGraph id so that they are only hardcoded in one
  // place
  using IdType = EdgeId;
  using EnumType = EdgeId::Type;
  using VertexIdType = VertexId;
  using SimpleIdType = std::pair<uint64_t, uint64_t>;

  // If you need a different type of edge, change this and recompile. Making it
  // a template was just too gross.
  using TransformType = lgmath::se3::TransformationWithCovariance;
  const static int transform_rows = 4;
  const static int transform_cols = 4;
  const static int transform_vdim = 6;
  using TransformMatType =
      Eigen::Matrix<double, transform_cols, transform_rows>;
  using TransformVecType = Eigen::Matrix<double, transform_vdim, 1>;

  PTR_TYPEDEFS(EdgeBase)
  CONTAINER_TYPEDEFS(EdgeBase)

  /** \brief Pseudo constructors to generate a shared pointer */
  static Ptr MakeShared(const VertexIdType& from_id, const VertexIdType& to_id,
                        const EnumType& type, bool manual = false);
  static Ptr MakeShared(const VertexIdType& from_id, const VertexIdType& to_id,
                        const EnumType& type, const TransformType& T_to_from,
                        bool manual = false);

  /** \brief Default constructor */
  EdgeBase(const VertexIdType& from_id, const VertexIdType& to_id,
           const EnumType& type, bool manual = false);
  EdgeBase(const VertexIdType& from_id, const VertexIdType& to_id,
           const EnumType& type, const TransformType& T_to_from,
           bool manual = false);

  EdgeBase(const EdgeBase&) = default;
  EdgeBase(EdgeBase&&) = default;
  EdgeBase& operator=(const EdgeBase&) = default;
  EdgeBase& operator=(EdgeBase&&) = default;

  /** \brief Default constructor */
  virtual ~EdgeBase() = default;

  /** \brief Get the edge id */
  IdType id() const;

  /** \brief Get the edge id as a plain type */
  SimpleIdType simpleId() const;

  /** \brief Get the edge type */
  IdType::Type type() const;

  /** \brief Get the edge type as a size_t for indexing */
  size_t idx() const;

  /** \brief Get the ids of incident vertices */
  VertexIdType::Pair incident() const;

  /** \brief Get the id of the from vertex */
  VertexIdType from() const;

  /** \brief Get the id of the to vertex */
  VertexIdType to() const;

  /** \brief Get the edge transform */
  TransformType T() const;

  /** \brief Return true if the edge was manually driven */
  bool isManual() const;

  /** \brief Return true if the edge was driven autonomously */
  bool isAutonomous() const;

  /** \brief Return true if the edge is a temporal edge */
  bool isTemporal() const;

  /** \brief Return true if the edge is a spatial edge */
  bool isSpatial() const;

  /** \brief Return true if the vertex is incident on this edge */
  bool isIncident(const VertexIdType& v) const;

  /** \brief Set the edge transform */
  void setTransform(const TransformType& transform);

  /** \brief String output */
  friend std::ostream& operator<<(std::ostream& out, const EdgeBase& e);

 protected:
  /** \brief The edge Id, which must be consistent with from_ and to_ */
  const IdType id_;

  /** \brief The originating vertex Id */
  const VertexIdType from_;

  /** \brief The terminating vertex Id */
  const VertexIdType to_;

  /** \brief The transform that moves points in "from" to points in "to" */
  TransformType T_to_from_ = TransformType();

  /** \brief Whether this edge was manually driven or not */
  const bool manual_;

 protected:
  /** \brief protects all non-const class members including: T_to_from_ */
  mutable std::shared_mutex mutex_;
};
}  // namespace pose_graph
}  // namespace vtr
