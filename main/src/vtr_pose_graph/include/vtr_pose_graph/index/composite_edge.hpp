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
 * \file composite_edge.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/index/edge_base.hpp>
#include <vtr_pose_graph/path/path.hpp>

namespace vtr {
namespace pose_graph {

/** \brief Represents a meta-edge, formed from a linear chain of pose */
template <class G, class TF = typename G::EdgeType::TransformType>
class CompositeEdge : public EdgeBase, public Path<G, TF> {
 public:
  using PathType = Path<G, TF>;

  using GraphType = G;
  using GraphPtr = typename G::Ptr;
  using VertexIdType = typename G::VertexIdType;
  using EdgeIdType = typename G::EdgeIdType;
  using SequenceType = typename PathType::SequenceType;

  /** \brief Shared pointer definitions */
  PTR_TYPEDEFS(CompositeEdge);

  CompositeEdge(const GraphPtr& graph, const SequenceType& sequence);
  CompositeEdge(const CompositeEdge&) = default;
  CompositeEdge(CompositeEdge&&) = default;
  CompositeEdge& operator=(const CompositeEdge&) = default;
  CompositeEdge& operator=(CompositeEdge&&) = default;

  /** \brief Get the edge transform */
  virtual TransformType T() const;

  /** \brief Set the edge transform */
  virtual void setTransform(const TransformType& transform);

  virtual CompositeEdge& reverse() {
    PathType::reverse();

    auto tmp = this->from_;
    this->to_ = this->from_;
    this->from_ = tmp;
    this->T_to_from_ = this->T_to_from_.inverse();

    return *this;
  }

  CompositeEdge reversed() const {
    CompositeEdge rval(*this);
    rval.reverse();
    return rval;
  }

 protected:
  /** \brief Setup cached properties on sequence change */
  virtual void initSequence();
};

template <class G, class TF>
CompositeEdge<G, TF>::CompositeEdge(const GraphPtr& graph,
                                    const SequenceType& sequence)
    : EdgeBase(), PathType(graph) {
  this->setSequence(sequence);
  if (!this->verifySequence()) {
    throw std::invalid_argument(
        "[CompositeEdge] Requested vertex sequence does not form a chain.");
  }
}

template <class G, class TF>
auto CompositeEdge<G, TF>::T() const -> TransformType {
  return this->pose(this->sequence_.size() - 1);
}

template <class G, class TF>
void CompositeEdge<G, TF>::setTransform(const TransformType&) {
  LOG(WARNING) << "Attempting to set the transform of a composite edge... Are "
                  "you using the right type of graph?";
}

template <class G, class TF>
void CompositeEdge<G, TF>::initSequence() {
  // Initialize the parent
  PathType::initSequence();

  this->from_ = this->sequence_.front();
  this->to_ = this->sequence_.back();

  auto etype =
      this->sequence_.front().majorId() == this->sequence_.back().majorId()
          ? EdgeIdType::Type::Temporal
          : EdgeIdType::Type::Spatial;

  // Slightly ugly: the ID of a composite edge changes if you set the sequence
  // to something else
  this->id_ =
      EdgeIdType(this->sequence_.front(), this->sequence_.back(), etype);
  this->manual_ = true;
  this->modified_ = false;
}

}  // namespace pose_graph
}  // namespace vtr
