#pragma once

#include <vtr_pose_graph/index/composite_edge.hpp>
#include <vtr_pose_graph/index/graph_base.hpp>

namespace vtr {
namespace pose_graph {

template <class G>
class CompositeGraph : public GraphBase<VertexBase, CompositeEdge<G>,
                                        RunBase<VertexBase, CompositeEdge<G>>> {
 public:
  using GraphType = G;
  using GraphPtr = typename GraphType::Ptr;
  using VertexIdType = typename G::VertexIdType;
  using SubgraphType = typename G::RType;
  using SubgraphPtr = typename SubgraphType::Ptr;

  using RunType = RunBase<VertexBase, CompositeEdge<G>>;
  using Base = GraphBase<VertexBase, CompositeEdge<G>, RunType>;

  using Base::edges_;
  using Base::graph_;
  using Base::id_;
  using Base::runs_;
  using Base::vertices_;

  using Base::makeSimple;

  PTR_TYPEDEFS(CompositeGraph);

  template <typename... Args>
  static std::shared_ptr<CompositeGraph> MakeShared(Args&&... args) {
    return std::shared_ptr<CompositeGraph>(
        new CompositeGraph(std::forward<Args>(args)...));
  }

  /** \brief Construct from a dense graph */
  CompositeGraph(const GraphPtr& graph);

  /** \brief Constructor to create subgraphs */
  CompositeGraph(const CompositeGraph& other, const SimpleGraph& graph)
      : Base(other, graph), denseGraph_(other.denseGraph_) {
  }

  /** \brief Constructor to create subgraphs, using move on the structure */
  CompositeGraph(const CompositeGraph& other, SimpleGraph&& graph)
      : Base(other, std::move(graph)),
        denseGraph_(std::move(other.denseGraph_)) {
  }

  CompositeGraph(const CompositeGraph&) = default;
  CompositeGraph(CompositeGraph&&) = default;
  CompositeGraph& operator=(const CompositeGraph&) = default;
  CompositeGraph& operator=(CompositeGraph&&) = default;

  /** \brief Return the dense graph being simplified */
  inline const GraphPtr& base() const {
    return denseGraph_;
  }

  /** \brief Flatten a composite graph back into a dense graph */
  SubgraphPtr flatten() const;

  /** \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges) */
  Ptr getSubgraph(const typename VertexIdType::Vector& nodes) const {
    return MakeShared(*this, graph_.getSubgraph(makeSimple(nodes)));
  }

  /** \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges) */
  Ptr getSubgraph(const VertexIdType& rootId,
                  const eval::Mask::Ptr& mask) const {
    return MakeShared(*this, graph_.getSubgraph(rootId, mask));
  }

  /** \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges) */
  Ptr getSubgraph(const eval::Mask::Ptr& mask) const {
    for (auto it = this->beginVertex(); it != this->endVertex(); ++it) {
      if (mask->operator[](it->id()))
        return this->getSubgraph(it->id(), mask);
    }
    return MakeShared(*this, SimpleGraph());
  }

  /** \brief Get the induced subgraph of another subgraph */
  inline Ptr induced(const CompositeGraph& subgraph) const {
    return Ptr(new CompositeGraph(*this, graph_.induced(subgraph.graph_)));
  }

  /** \brief Merge two graphs in place, as a set union */
  CompositeGraph& operator+=(const CompositeGraph& other) {
    graph_ += other.graph_;
    return *this;
  }

  /** \brief Merge two graphs, as a set union */
  friend CompositeGraph operator+(CompositeGraph lhs,
                                  const CompositeGraph& rhs) {
    lhs += rhs;
    return lhs;
  }

  /** \brief Merge two graphs, as a set union */
  Ptr setUnion(const Ptr& other) const {
    return Ptr(new CompositeGraph(*this, graph_ + other->graph_));
  }

  /** \brief Use dijkstra's algorithm to traverse up to a depth (weighted edges)
   */
  Ptr dijkstraTraverseToDepth(
      const VertexIdType& rootId, double maxDepth,
      const eval::Weight::Ptr& weights = eval::Weight::Const::MakeShared(1, 1),
      const eval::Mask::Ptr& mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const {
    return MakeShared(
        *this, graph_.dijkstraTraverseToDepth(rootId, maxDepth, weights, mask));
  }

  /** \brief Use dijkstra's algorithm to search for an id (weighted
   * edges) */
  Ptr dijkstraSearch(
      const VertexIdType& rootId, VertexIdType searchId,
      const eval::Weight::Ptr& weights = eval::Weight::Const::MakeShared(1, 1),
      const eval::Mask::Ptr& mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const {
    return MakeShared(*this,
                      graph_.dijkstraSearch(rootId, searchId, weights, mask));
  }

  /** \brief Use dijkstra's algorithm to search for multiple ids (weighted
   * edges) */
  Ptr dijkstraMultiSearch(
      const VertexIdType& rootId,
      const typename VertexIdType::Vector& searchIds,
      const eval::Weight::Ptr& weights = eval::Weight::Const::MakeShared(1, 1),
      const eval::Mask::Ptr& mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const {
    return MakeShared(*this, graph_.dijkstraMultiSearch(
                                 rootId, makeSimple(searchIds), weights, mask));
  }

  /** \brief Use breadth first traversal up to a depth */
  Ptr breadthFirstTraversal(const VertexIdType& rootId, double maxDepth) const {
    return MakeShared(*this, graph_.breadthFirstTraversal(rootId, maxDepth));
  }

  /** \brief Use breadth first search for an id */
  Ptr breadthFirstSearch(const VertexIdType& rootId,
                         VertexIdType searchId) const {
    return MakeShared(*this, graph_.breadthFirstSearch(rootId, searchId));
  }

  /** \brief Use breadth first search for multiple ids */
  Ptr breadthFirstMultiSearch(
      const VertexIdType& rootId,
      const typename VertexIdType::Vector& searchIds) const {
    return MakeShared(
        *this, graph_.breadthFirstMultiSearch(rootId, makeSimple(searchIds)));
  }
#if 0
  /** \brief Get minimal spanning tree */
  Ptr getMinimalSpanningTree(
      const eval::Weight::Ptr& weights,
      const eval::Mask::Ptr& mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const {
    return MakeShared(*this, graph_.getMinimalSpanningTree(weights, mask));
  }
#endif

 protected:
  /** \brief Internal edge addition function */
  void addEdge_(const std::list<VertexIdType>& seq);

  /** \brief Internal vertex addition function */
  void addVertex_(const VertexIdType& v);

  GraphPtr denseGraph_;
};

}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/index/composite_graph.inl>
