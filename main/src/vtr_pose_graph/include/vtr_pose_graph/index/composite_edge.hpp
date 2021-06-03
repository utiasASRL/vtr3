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
