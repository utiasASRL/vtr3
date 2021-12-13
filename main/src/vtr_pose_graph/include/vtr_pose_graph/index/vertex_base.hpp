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
 * \file vertex_base.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <shared_mutex>

#include "lgmath.hpp"

#include "vtr_common/utils/macros.hpp"
#include "vtr_pose_graph/id/id.hpp"

namespace vtr {
namespace pose_graph {

class VertexBase {
 public:
  // This is how we allow the graph to add edges to this object, but
  // prevent users from doing the same outside of the graph context
  template <class V, class E, class R>
  friend class Graph;
  friend class RCGraph;

  // Typedef the Ids here so that it's only hard coded in a single place
  using IdType = VertexId;
  using EdgeIdType = EdgeId;
  using SimpleIdType = uint64_t;
  using VertexIdSetArray = std::array<IdType::Set, EdgeId::NumTypes()>;

  using TransformType = lgmath::se3::TransformationWithCovariance;
  const static int transform_rows = 4;
  const static int transform_cols = 4;
  const static int transform_vdim = 6;
  using TransformMatType =
      Eigen::Matrix<double, transform_cols, transform_rows>;
  using TransformVecType = Eigen::Matrix<double, transform_vdim, 1>;

  PTR_TYPEDEFS(VertexBase)
  CONTAINER_TYPEDEFS(VertexBase)

  static Ptr MakeShared(const IdType& id);

  explicit VertexBase(const IdType& id);

  VertexBase(const VertexBase&) = default;
  VertexBase(VertexBase&&) = default;
  VertexBase& operator=(const VertexBase&) = default;
  VertexBase& operator=(VertexBase&&) = default;

  virtual ~VertexBase() = default;

  /** \brief Get all incident edges */
  EdgeIdType::Set incident() const;

  /** \brief Get all incident edges, filtered by edge type */
  EdgeIdType::Set incident(const EdgeIdType::Type& type) const;

  /** \brief Get all temporal edges */
  EdgeIdType::Set temporalEdges() const;

  /** \brief Get all spatial edges */
  EdgeIdType::Set spatialEdges() const;

  /** \brief Determine if an edge is incident on this vertex */
  bool isIncident(const EdgeIdType& e) const;

  /** \brief Get all neighbouring vertices */
  IdType::Set neighbours() const;

  /** \brief Get all neighbouring vertices, filtered by edge type */
  IdType::Set neighbours(const EdgeIdType::Type& type) const;

  /** \brief Get all temporal neighbours */
  IdType::Set temporalNeighbours() const;

  /** \brief Get all spatial neighbours */
  IdType::Set spatialNeighbours() const;

  /** \brief Determine if a vertex is incident on this vertex */
  bool isNeighbour(const IdType& v) const;

  /** \brief Determine if a vertex is a spatial or temporal neighbour */
  bool isNeighbour(const IdType& v, const EdgeIdType::Type& etype) const;

  /** \brief Determine if a vertex is a spatial neighbour of this vertex */
  bool isSpatialNeighbour(const IdType& v) const;

  /** \brief Determine if a vertex is a temporal neighbour of  this vertex */
  bool isTemporalNeighbour(const IdType& v) const;

  /** \brief Get the vertex id */
  IdType id() const;

  /** \brief Get the vertex id as a plain type */
  SimpleIdType simpleId() const;

  /** \brief String output */
  friend std::ostream& operator<<(std::ostream& out, const VertexBase& v);

 protected:
  /**
   * \brief Add an edge to the incident edge list of this vertex
   * \details This method is private as the Graph class manages connectivity
   */
  void addEdge(const EdgeIdType& e);

  /**
   * \brief Add an edge to the incident edge list of this vertex
   * \details This method is private as the Graph class manages connectivity
   */
  void addEdge(const IdType& to, const EdgeIdType::Type& etype);

  /**
   * \brief Remove an edge from the incident edge list of this vertex by Id
   * \details This method is private as the Graph class manages connectivity
   * TODO: decide if we want to allow deletion at all
   */
  void deleteEdge(const EdgeIdType& e);

  /**
   * \brief Remove a vertex from the neighbour list of this vertex by Id
   * \details This method is private as the Graph class manages connectivity
   * TODO: decide if we want to allow deletion at all
   */
  void deleteEdge(const IdType& to, const EdgeIdType::Type& etype);

  /** \brief The vertex Id */
  const IdType id_;

  /** \brief protects non-const class members including: neighbours_ */
  mutable std::shared_mutex mutex_;

  /** \brief Array of neighbour sets: [temporal<...>, spatial<...>] */
  VertexIdSetArray neighbours_ = VertexIdSetArray();
};

}  // namespace pose_graph
}  // namespace vtr
