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

enum class EdgeType { Temporal = 0, Spatial = 1, Undefined = 2 };
static constexpr size_t NumEdgeType = 2;

using EdgeTransform = lgmath::se3::TransformationWithCovariance;

class EdgeBase {
 public:
  PTR_TYPEDEFS(EdgeBase);
  CONTAINER_TYPEDEFS(EdgeBase);

  // If you need a different type of edge, change this and recompile. Making it
  // a template was just too gross.
  using Transform = EdgeTransform;
  using TransformMat = Eigen::Matrix<double, 4, 4>;
  static constexpr int transform_vdim = 6;
  using TransformVec = Eigen::Matrix<double, transform_vdim, 1>;

  /** \brief Pseudo constructors to generate a shared pointer */
  static Ptr MakeShared(const VertexId& from_id, const VertexId& to_id,
                        const EdgeType& type, const bool manual,
                        const EdgeTransform& T_to_from = EdgeTransform());

  EdgeBase(const VertexId& from_id, const VertexId& to_id, const EdgeType& type,
           const bool manual, const EdgeTransform& T_to_from = EdgeTransform());

  virtual ~EdgeBase() = default;

  /** \brief Get the edge id */
  EdgeId id() const;

  /** \brief Get the id of the from vertex */
  VertexId from() const;

  /** \brief Get the id of the to vertex */
  VertexId to() const;

  /** \brief Get the edge type */
  EdgeType type() const;

  /** \brief Get the edge type as a size_t for indexing */
  size_t idx() const;

  /** \brief Return true if the edge was manually driven */
  bool isManual() const;

  /** \brief Return true if the edge was driven autonomously */
  bool isAutonomous() const;

  /** \brief Return true if the edge is a temporal edge */
  bool isTemporal() const;

  /** \brief Return true if the edge is a spatial edge */
  bool isSpatial() const;

  /** \brief Get the edge transform */
  EdgeTransform T() const;

  /** \brief Set the edge transform */
  void setTransform(const EdgeTransform& transform);

  /** \brief String output */
  friend std::ostream& operator<<(std::ostream& out, const EdgeBase& e);

 protected:
  /** \brief The edge Id, which must be consistent with from_ and to_ */
  const EdgeId id_;

  /** \brief The originating vertex Id */
  const VertexId from_;

  /** \brief The terminating vertex Id */
  const VertexId to_;

  /** \brief The edge type */
  const EdgeType type_;

  /** \brief Whether this edge was manually driven or not */
  const bool manual_;

  /** \brief protects all non-const class members including: T_to_from_ */
  mutable std::shared_mutex mutex_;

  /** \brief The transform that moves points in "from" to points in "to" */
  EdgeTransform T_to_from_ = EdgeTransform();
};
}  // namespace pose_graph
}  // namespace vtr
