#pragma once

#include <vtr_common/utils/macros.hpp>
#include <vtr_logging/logging.hpp>
#include <vtr_pose_graph/simple_graph/simple_iterator.hpp>

namespace vtr {
namespace pose_graph {

/**
 * \brief Generic, proxied iterator for vertices/edges
 */
template <class G>
class VertexIterator {  //: public std::iterator<std::forward_iterator_tag,
                        // typename G::VertexPtr> {
 public:
  using VertexType = typename G::VertexType;
  using VertexPtr = typename G::VertexPtr;
  using VertexIdType = typename G::VertexIdType;
  using SimpleVertexId = typename G::SimpleVertexId;

  using IterType = simple::SimpleGraph::VertexIter;

  VertexIterator(const G *graph, const IterType &internalIter);
  VertexIterator(const VertexIterator &) = default;
  VertexIterator(VertexIterator &&) = default;

  VertexIterator &operator=(const VertexIterator &) = default;
  VertexIterator &operator=(VertexIterator &&) = default;

  VertexPtr &operator*() const;
  VertexType *operator->() const;

  VertexIterator &operator++();
  VertexIterator operator++(int);

  //  VertexIterator &operator--();
  //  VertexIterator operator--(int);

  bool operator==(const VertexIterator &other) const;
  bool operator!=(const VertexIterator &other) const;

 private:
  const G *graph_;

  IterType internalIter_;
};

/**
 * \brief Generic, proxied iterator for vertices/edges
 */
template <class G>
class EdgeIterator {  //: public std::iterator<std::bidirectional_iterator_tag,
                      // typename G::EdgePtr> {
 public:
  using EdgeType = typename G::EdgeType;
  using EdgePtr = typename G::EdgePtr;
  using EdgeIdType = typename G::EdgeIdType;
  using SimpleEdgeId = typename G::SimpleEdgeId;

  using IterType = simple::SimpleGraph::EdgeIter;

  EdgeIterator(const G *graph, const IterType &internalIter);
  EdgeIterator(const EdgeIterator &) = default;
  EdgeIterator(EdgeIterator &&) = default;

  EdgeIterator &operator=(const EdgeIterator &) = default;
  EdgeIterator &operator=(EdgeIterator &&) = default;

  EdgePtr &operator*() const;
  EdgeType *operator->() const;

  EdgeIterator &operator++();
  EdgeIterator operator++(int);

  EdgeIterator &operator--();
  EdgeIterator operator--(int);

  bool operator==(const EdgeIterator &other) const;
  bool operator!=(const EdgeIterator &other) const;

 private:
  const G *graph_;

  IterType internalIter_;
};

/**
 * \brief Struct that serves as the "value type" for OrderedGraphIterator
 */
template <class G>
struct NodeParent {
  using VertexPtr = typename G::VertexPtr;
  using EdgePtr = typename G::EdgePtr;

  using VertexIdType = typename G::VertexIdType;
  using EdgeIdType = typename G::EdgeIdType;
  using SimpleVertexId = typename G::SimpleVertexId;
  using SimpleEdgeId = typename G::SimpleEdgeId;
#if 0
  using TransformType = typename G::EdgeType::TransformType;
#endif

  NodeParent(const G *graph = nullptr,
             const simple::NodeParent &top = simple::NodeParent())
      : graph_(graph), top_(top) {}
  NodeParent(const NodeParent &) = default;
  NodeParent(NodeParent &&) = default;

  NodeParent &operator=(const NodeParent &) = default;
  NodeParent &operator=(NodeParent &&) = default;

  /**
   * \brief Implicit conversion to VertexId/EdgeId for indexing convenience
   */
  inline operator VertexIdType() const { return VertexIdType(top_.v()); }
  inline operator EdgeIdType() const { return graph_->at(top_.e())->id(); }

  /**
   * \brief Implicit conversion to SimpleEdge/SimpleVertex for indexing
   * convenience
   */
  inline operator SimpleVertexId() const { return top_.v(); }
  inline operator SimpleEdgeId() const { return top_.e(); }

  /**
   * \brief Get the referenced Vertex pointer
   */
  const VertexPtr &v() const { return graph_->at(top_.v()); }

  /**
   * \brief Get the referenced Edge pointer
   */
  const EdgePtr &e() const { return graph_->at(top_.e()); }
#if 0
  /**
   * \brief Get the edge transform, properly oriented in the iterator
   * direction: T_prev_current
   */
  inline TransformType T() const {
    if (!from().isValid() || e() == nullptr) return TransformType(true);
    // The edge has T_to_from, and we want T_prev_current
    bool invert = from() == e()->from();
    // is edge from == iter prev? (then invert)
    return invert ? e()->T().inverse() : e()->T();
  }
#endif
  /**
   * \brief Get the ancestor VertexId
   */
  inline const VertexIdType from() const { return VertexIdType(top_.parent); }

  /**
   * \brief Get the child VertexId
   */
  inline const VertexIdType to() const { return VertexIdType(top_.child); }

  const G *graph_;
  simple::NodeParent top_;
};

/**
 * \brief Iterator to a graph in search order
 */
template <class G>
class OrderedGraphIterator
    : std::iterator<std::forward_iterator_tag, const NodeParent<G> > {
 public:
  using GraphType = G;
  OrderedGraphIterator(const G *graph,
                       const simple::SimpleGraphIterator &internalIter);
  OrderedGraphIterator(const OrderedGraphIterator &) = default;
  OrderedGraphIterator(OrderedGraphIterator &&) = default;

  OrderedGraphIterator &operator=(const OrderedGraphIterator &) = default;
  OrderedGraphIterator &operator=(OrderedGraphIterator &&) = default;
#if 0
  const NodeParent<G> &operator*() const;
  const NodeParent<G> *operator->() const;

  OrderedGraphIterator &operator++();
  OrderedGraphIterator operator++(int);

  bool operator==(const OrderedGraphIterator &other) const;
  bool operator!=(const OrderedGraphIterator &other) const;
#endif
 private:
  simple::SimpleGraphIterator internalIter_;

  NodeParent<G> data_;
};

}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/index/graph_iterator.inl>