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
 * \file run_base.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <map>
#include <unordered_map>

#include "vtr_pose_graph/index/edge_base.hpp"
#include "vtr_pose_graph/index/vertex_base.hpp"

namespace vtr {
namespace pose_graph {

template <class V, class E>
class RunBase {
 public:
  PTR_TYPEDEFS(RunBase)
  // The Id type of this opject (uint32)
  using IdType = BaseIdType;

  using VertexType = V;
  using VertexPtr = typename V::Ptr;
  using VertexIdType = typename V::IdType;
  using SimpleVertexId = typename V::SimpleIdType;

  using EdgeType = E;
  using EdgePtr = typename E::Ptr;
  using EdgeIdType = typename E::IdType;
  using EdgeEnumType = typename E::IdType::Type;
  using SimpleEdgeId = typename E::SimpleIdType;
  using TransformType = typename E::TransformType;

  // Array to hold the current index of each edge type
  using CurrentEdgeArray = std::array<BaseIdType, EdgeIdType::NumTypes()>;
  // Internal vertex map
  using VertexPtrMap = std::map<VertexIdType, VertexPtr>;
  // Internal edge map
  using EdgePtrMap = std::unordered_map<EdgeIdType, EdgePtr>;
  using EdgePtrMapArray = std::array<EdgePtrMap, EdgeIdType::NumTypes()>;

  /** \brief Convenience constructor to create a shared pointer */
  static Ptr MakeShared(const IdType& id);

  /** \brief Default constructor, for completeness */
  RunBase(const IdType& id);

  /** \brief Default copy and move operations */
  RunBase(const RunBase&) = default;
  RunBase(RunBase&&) = default;
  RunBase& operator=(const RunBase&) = default;
  RunBase& operator=(RunBase&&) = default;

  /** \brief Destructor */
  virtual ~RunBase() = default;

  /** \brief Return a blank vertex with the next available Id */
  template <class... Args>
  VertexPtr addVertex(Args&&... args);

  template <class... Args>
  VertexPtr addVertex(const VertexIdType& v, Args&&... args);

  /** \brief Return an edge between two vertices, with the next available Id */
  template <class... Args>
  EdgePtr addEdge(const VertexIdType& from, const VertexIdType& to,
                  const EdgeEnumType& type, bool manual = false,
                  Args&&... args);

  /** \brief Return an edge between two vertices, with the next available Id */
  template <class... Args>
  EdgePtr addEdge(const VertexIdType& from, const VertexIdType& to,
                  const EdgeEnumType& type, const TransformType& T_to_from,
                  bool manual = false, Args&&... args);

  /** \brief Add an externally constructed edge */
  void addEdge(const EdgePtr& edge);

  /** Get the number of vertices in this run */
  unsigned int numberOfVertices() const {
    std::shared_lock lock(mutex_);
    return vertices_.size();
  }

  /** \brief Map interface for vertices */
  VertexPtr operator[](const VertexIdType& v) {
    std::shared_lock lock(mutex_);
    return vertices_[v];
  }

  /** \brief Map interface for edges */
  EdgePtr operator[](const EdgeIdType& e) {
    std::shared_lock lock(mutex_);
    return edges_[e.idx()][e];
  }

  /** \brief Map interface for edges */
  EdgePtr operator[](const SimpleEdgeId& e) {
    std::shared_lock lock(mutex_);

    VertexIdType v1(e.first), v2(e.second);
    VertexPtr v2p = vertices_.at(v2);

    if (v2p->isSpatialNeighbour(v1)) {
      return edges_[size_t(EdgeEnumType::Spatial)]
                   [EdgeIdType(v1, v2, EdgeEnumType::Spatial)];
    } else if (v2p->isTemporalNeighbour(v1)) {
      return edges_[size_t(EdgeEnumType::Temporal)]
                   [EdgeIdType(v1, v2, EdgeEnumType::Temporal)];
    } else {
      throw std::invalid_argument("Edge does not exist.");
    }
  }

  /** \brief Const map interface for vertices */
  VertexPtr at(const VertexIdType& v) const {
    std::shared_lock lock(mutex_);
    return vertices_.at(v);
  }

  /** \brief Const map interface for edges */
  EdgePtr at(const EdgeIdType& e) const {
    std::shared_lock lock(mutex_);
    return edges_[e.idx()].at(e);
  }

  /** \brief Map interface for edges */
  EdgePtr at(const SimpleEdgeId& e) const {
    std::shared_lock lock(mutex_);

    VertexIdType v1(e.first), v2(e.second);
    VertexPtr v2p = vertices_.at(v2);

    if (v2p->isSpatialNeighbour(v1)) {
      return edges_[size_t(EdgeEnumType::Spatial)].at(
          EdgeIdType(v1, v2, EdgeEnumType::Spatial));
    } else if (v2p->isTemporalNeighbour(v1)) {
      return edges_[size_t(EdgeEnumType::Temporal)].at(
          EdgeIdType(v1, v2, EdgeEnumType::Temporal));
    } else {
      throw std::invalid_argument("Edge does not exist.");
    }
  }

  /** \brief Get the run ID */
  IdType id() const { return id_; }

  /** \brief Query whether the run contains manual edges */
  bool isManual() const {
    std::shared_lock lock(mutex_);
    return manual_;
  }

 protected:
  /** \brief Recompute the manual status of the run (does not lock anything) */
  void computeManual();

 protected:
  /** \brief The run id, used for indexing and for loading/saving */
  const IdType id_;

  /**
   * \brief Internal vertex map
   * \details This preserves the mapping of VertexId --> VertexPtr and the
   *          mapping of RunId --> VertexPtr.  Without this secondary map,
   *          these properties would need to be rebuilt using a search.
   */
  VertexPtrMap vertices_ = VertexPtrMap();

  /** \brief Tracks the last vertex id in the run */
  BaseIdType current_vertex_ = -1;

  /**
   * \brief Internal edge map
   * \details This preserves the mapping of EdgeId --> EdgePtr and the
   *          mapping of RunId --> EdgePtr.  Without this secondary map,
   *          these properties would need to be rebuilt using a search.  This
   *          map also contains spatial edges we did not explicitly request,
   *          so that we do not overwrite the index file without them.
   */
  EdgePtrMapArray edges_ = EdgePtrMapArray();

  /** \brief Tracks the last edge id in the run, for each edge type */
  CurrentEdgeArray current_edge_ = CurrentEdgeArray();

  /** \brief Cache whether or not this run contains manual edges */
  bool manual_ = false;

 protected:
  /** \brief protects all non-const class members */
  mutable std::shared_mutex mutex_;
};

extern template class RunBase<VertexBase, EdgeBase>;
using BasicRun = RunBase<VertexBase, EdgeBase>;

}  // namespace pose_graph
}  // namespace vtr

#include "vtr_pose_graph/index/run_base.inl"
