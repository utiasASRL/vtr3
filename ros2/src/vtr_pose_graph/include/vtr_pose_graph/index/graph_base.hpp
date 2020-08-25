#pragma once

#include <vtr_common/utils/macros.hpp>
#include <vtr_pose_graph/index/edge_base.hpp>
#include <vtr_pose_graph/index/graph_iterator.hpp>
#include <vtr_pose_graph/index/run_base.hpp>
#include <vtr_pose_graph/index/vertex_base.hpp>
#include <vtr_pose_graph/simple_graph/simple_graph.hpp>

namespace vtr {
namespace pose_graph {

using simple::SimpleGraph;

#if 0
template <class V, class E, class R>
class GraphBase;

using BasicGraphBase =
    GraphBase<VertexBase, EdgeBase, RunBase<VertexBase, EdgeBase>>;
#endif

template <class V, class E, class R>
class GraphBase {
 public:
  using Base = GraphBase<V, E, R>;
  using RType = GraphBase<V, E, R>;
  using IdType = BaseIdType;

  // Each subclass will change this typedef; it is used for managing casts
  using VertexType = V;
  using EdgeType = E;
  using RunType = R;

  // Edge/Vertex shared pointers; each subclass will change this
  using VertexPtr = typename V::Ptr;
  using EdgePtr = typename E::Ptr;
  using RunPtr = typename R::Ptr;

  // Proxied iterators
  using SelfType = GraphBase<V, E, R>;
  using VertexIter = VertexIterator<SelfType>;
  using EdgeIter = EdgeIterator<SelfType>;
  using OrderedIter = OrderedGraphIterator<SelfType>;

  // Edge/Vertex Id types
  using VertexIdType = typename V::IdType;
  using EdgeIdType = typename E::IdType;
  using EdgeTypeEnum = typename E::IdType::Type;
  using RunIdType = typename R::IdType;

  using SimpleVertexId = typename V::SimpleIdType;
  using SimpleEdgeId = typename E::SimpleIdType;

  // Internal mapping between SimpleGraph and our data types
  using VertexMap = std::unordered_map<SimpleVertexId, VertexPtr>;
  using EdgeMap = std::unordered_map<SimpleEdgeId, EdgePtr>;
  using RunMap = std::map<RunIdType, RunPtr>;

  using GraphComponent = simple::LinearComponent<VertexIdType>;
  using PtrComponent = simple::LinearComponent<VertexPtr>;
  using ComponentList = std::list<GraphComponent>;

  // Shared pointer declarations for our maps
  CONTAINER_NAMED_TYPEDEFS(VertexPtr);
  CONTAINER_NAMED_TYPEDEFS(EdgePtr);
  PTR_NAMED_TYPEDEFS(VertexMap);
  PTR_NAMED_TYPEDEFS(EdgeMap);
  PTR_NAMED_TYPEDEFS(RunMap);

  /**
   * \brief Shared pointer type definitions for this class
   */
  PTR_TYPEDEFS(GraphBase);

  /**
   * \brief Pseudo-constructor to make shared pointers
   */
  static Ptr MakeShared();
  static Ptr MakeShared(const IdType& id);
  static Ptr MakeShared(const GraphBase& other, const SimpleGraph& graph);
  static Ptr MakeShared(const GraphBase& other, SimpleGraph&& graph);

  /**
   * \brief Default Constructor
   */
  GraphBase();
  GraphBase(const GraphBase&) = default;
  GraphBase(GraphBase&&) = default;
  GraphBase& operator=(const GraphBase&) = default;
  GraphBase& operator=(GraphBase&&) = default;

  /**
   * \brief Empty graph constructor
   */
  GraphBase(const IdType& id);

  /**
   * \brief Constructor to create subgraphs
   */
  GraphBase(const GraphBase& other, const SimpleGraph& graph);

  /**
   * \brief Constructor to create subgraphs, using move on the structure
   */
  GraphBase(const GraphBase& other, SimpleGraph&& graph);

  /**
   * \brief Return the underlying subgraph structure
   */
  inline const SimpleGraph& subgraph() const { return graph_; }

  /**
   * \brief Return all vertices
   */
  inline const VertexMapPtr& vertices() const { return vertices_; }

  /**
   * \brief Return all edges
   */
  inline const EdgeMapPtr& edges() const { return edges_; }

  /** Get the number of vertices */
  inline unsigned int numberOfVertices() const {
    return graph_.numberOfNodes();
  }

  /** Get the number of vertices in a run */
  inline unsigned int numberOfVertices(const uint32_t& run_id) const {
    return run(run_id)->vertices().size();
  }

  /**
   * \brief Get the number of edges
   */
  inline unsigned int numberOfEdges() const { return graph_.numberOfEdges(); }

  /**
   * \brief Get the number of runs
   */
  inline unsigned int numberOfRuns() const { return runs_->size(); }

  /**
   * \brief Determine if this graph/subgraph contains a specific vertex
   */
  inline bool contains(const VertexIdType& v) const {
    return graph_.hasVertex(v);
  }

  /**
   * \brief Determine if this graph/subgraph contains a specific vertex
   */
  inline bool contains(const SimpleVertexId& v) const {
    return graph_.hasVertex(v);
  }

  /**
   * \brief Determine if this graph/subgraph contains a specific edge
   */
  inline bool contains(const EdgeIdType& e) const { return graph_.hasEdge(e); }

  /**
   * \brief Determine if this graph/subgraph contains a specific edge
   */
  inline bool contains(const SimpleEdgeId& e) const {
    return graph_.hasEdge(e);
  }

  /**
   * \brief Determine if this graph/subgraph contains a specific run
   */
  inline bool contains(const RunIdType& r) const {
    return runs_.get() != nullptr && runs_->find(r) != runs_->end();
  }

  /**
   * \brief Determine if this graph/subgraph contains a specific edge
   */
  inline bool contains(const VertexIdType& v1, const VertexIdType& v2) const {
    return graph_.hasEdge({SimpleVertexId(v1), SimpleVertexId(v2)});
  }

  /**
   * \brief Returns run with id run_id if exists.
   */
  inline const RunPtr& run(const RunIdType& run_id) const {
    try {
      return runs_->at(run_id);
    } catch (...) {
      std::stringstream error_msg;
      error_msg << "Could not find run " << run_id << " in the graph.\n";
#if 0
                << el::base::debug::StackTrace();
#endif
      throw std::range_error(error_msg.str());
    }
    return runs_->at(run_id);
  }

  inline const RunMap& runs() const { return *runs_; }

  /**
   * \brief Const map interface for vertices
   */
  inline const VertexPtr& at(const VertexIdType& v) const {
    try {
      return run(v.majorId())->at(v);
    } catch (...) {
      std::stringstream error_msg;
      error_msg << "Could not find " << v << " in the graph.\n";
#if 0
                << el::base::debug::StackTrace();
#endif
      throw std::range_error(error_msg.str());
    }
    // just so it compiles...
    return run(v.majorId())->at(v);
  }

  /**
   * \brief Const map interface for edges
   */
  inline const EdgePtr& at(const EdgeIdType& e) const {
    return at(SimpleEdgeId(e));
  }

  /**
   * \brief Const map interface for vertices
   */
  inline const VertexPtr& at(const SimpleVertexId& v) const {
    try {
      return run(uint32_t(v >> 32))->at(v);
    } catch (...) {
      std::stringstream error_msg;
      error_msg << "Could not find " << v << ": " << VertexIdType(v)
                << " in the graph.\n";
#if 0
                << el::base::debug::StackTrace();
#endif
      throw std::range_error(error_msg.str());
    }
    // just so it compiles...
    return run(uint32_t(v >> 32))->at(v);
  }

  /**
   * \brief Const map interface for edges
   */
  inline const EdgePtr& at(const SimpleEdgeId& e) const {
    return at(e.first, e.second);
  }

  /**
   * \brief Const map interface for edges
   */
  inline const EdgePtr& at(const SimpleVertexId& v1,
                           const SimpleVertexId& v2) const {
    try {
      return edges_->at(simple::SimpleGraph::getEdge(v1, v2));
    } catch (...) {
      std::stringstream error_msg;
      error_msg << "Could not find " << VertexIdType(v1) << ", "
                << VertexIdType(v2) << " in the graph.\n";
#if 0
                << el::base::debug::StackTrace();
#endif
      throw std::range_error(error_msg.str());
    }
    // just so it compiles...
    return edges_->at(simple::SimpleGraph::getEdge(v1, v2));
  }

  /**
   * \brief Const map interface for edges
   */
  inline const EdgePtr& at(const VertexIdType& v1,
                           const VertexIdType& v2) const {
    return at(SimpleVertexId(v1), SimpleVertexId(v2));
  }

  /**
   * \brief Const map interface for edges
   */
  inline const EdgePtr& at(const VertexPtr& v1, const VertexPtr& v2) const {
    return at(v1->id(), v2->id());
  }

  /**
   * \brief Get the neighbors of a vertex that are in this subgraph
   */
  inline VertexPtrSet neighbors(const VertexIdType& v) const {
    return neighbors(vertices_->at(v));
  }

  /**
   * \brief Get the neighbors of a vertex that are in this subgraph
   */
  VertexPtrSet neighbors(const VertexPtr& v) const;

  /**
   * \brief Get the incident edges of a vertex that are in this subgraph
   */
  inline EdgePtrSet incident(const SimpleVertexId& v) const {
    return incident(vertices_->at(v));
  }

  /**
   * \brief Get the incident edges of a vertex that are in this subgraph
   */
  EdgePtrSet incident(const VertexPtr& v) const;
#if 0
  /**
   * \brief Get an iterator for the graph.  Defaults to BFS over the entire
   * graph
   */
  inline OrderedIter begin(
      VertexIdType root = VertexIdType::Invalid(), double maxDepth = 0,
      const Eval::Mask::Ptr& mask = Eval::Mask::Const::MakeShared(true, true),
      const Eval::Weight::Ptr& weight =
          Eval::Weight::Const::MakeShared(1.f, 1.f)) const {
    return OrderedIter(this, graph_.begin(root, maxDepth, mask, weight));
  }

  /**
   * \brief Get a Breadth-First iterator for the graph
   */
  inline OrderedIter beginBfs(
      VertexIdType root = VertexIdType::Invalid(), double maxDepth = 0,
      const Eval::Mask::Ptr& mask = Eval::Mask::Const::MakeShared(true, true),
      const Eval::Weight::Ptr& weight =
          Eval::Weight::Const::MakeShared(1.f, 1.f)) const {
    return OrderedIter(this, graph_.beginBfs(root, maxDepth, mask, weight));
  }

  /**
   * \brief Get a Depth-First iterator for the graph
   */
  inline OrderedIter beginDfs(
      VertexIdType root = VertexIdType::Invalid(), double maxDepth = 0,
      const Eval::Mask::Ptr& mask = Eval::Mask::Const::MakeShared(true, true),
      const Eval::Weight::Ptr& weight =
          Eval::Weight::Const::MakeShared(1.f, 1.f)) const {
    return OrderedIter(this, graph_.beginDfs(root, maxDepth, mask, weight));
  }

  /**
   * \brief Get a Dijkstra iterator for the graph
   */
  inline OrderedIter beginDijkstra(
      VertexIdType root = VertexIdType::Invalid(), double maxDepth = 0,
      const Eval::Mask::Ptr& mask = Eval::Mask::Const::MakeShared(true, true),
      const Eval::Weight::Ptr& weight =
          Eval::Weight::Const::MakeShared(1.f, 1.f)) const {
    return OrderedIter(this,
                       graph_.beginDijkstra(root, maxDepth, mask, weight));
  }
  /**
   * \brief Get the end iterator for this graph
   */
  inline OrderedIter end() const { return OrderedIter(this, graph_.end()); }
#endif

  /**
   * \brief Iterator interface to all vertices in this subgraph
   */
  inline VertexIter beginVertex() const {
    return VertexIter(this, graph_.beginVertex());
  }

  /**
   * \brief End iterator for all vertices in this subgraph
   */
  inline VertexIter endVertex() const {
    return VertexIter(this, graph_.endVertex());
  }

  /**
   * \brief Iterator interface to all vertices in this subgraph
   */
  inline EdgeIter beginEdge() const {
    return EdgeIter(this, graph_.beginEdge());
  }

  /**
   * \brief End iterator for all vertices in this subgraph
   */
  inline EdgeIter endEdge() const { return EdgeIter(this, graph_.endEdge()); }

  /**
   * \brief Const map interface for vertices
   */
  std::vector<VertexPtr> at(const typename VertexIdType::Vector& v) const;

  /**
   * \brief Const map interface for LinearComponents
   */
  PtrComponent at(const GraphComponent& v) const;

  /**
   * \brief Const map interface for edges
   */
  std::vector<EdgePtr> at(const typename EdgeIdType::Vector& e) const;

  /**
   * \brief Const map interface for vertices
   */
  std::vector<VertexPtr> at(const std::vector<SimpleVertexId>& v) const;

  /**
   * \brief Const map interface for edges
   */
  std::vector<EdgePtr> at(const std::vector<SimpleEdgeId>& e) const;

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const typename VertexIdType::Vector& nodes) const;
#if 0
  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const VertexIdType& rootId,
                  const Eval::Mask::Ptr& mask) const;

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges) to a max depth
   */
  Ptr getSubgraph(const VertexIdType& rootId, double maxDepth,
                  const Eval::Mask::Ptr& mask) const;

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  Ptr getSubgraph(const Eval::Mask::Ptr& mask) const;

  /**
   * \brief Get subgraph containing a single run
   */
  Ptr getRunSubgraph(const RunIdType& runId) const;

  /**
   * \brief Convenience function to get a manual subgraph
   */
  //  Ptr getManualSubgraph() const;

  /**
   * \brief Get a map of run chains for all autonomous runs
   */
  std::map<RunIdType, Ptr> autonomousRuns() const;

  /**
   * \brief Get the induced subgraph of another subgraph
   * \detail Colloquially: G[H] is formed from H by adding every edge from G
   * that has both
   *         endpoints in H.  Mathematically: for V(H) ⊆ V(G), V(G[H]) = V(H)
   *         and uv ∈ E(G[H]) ⇔ uv ∈ E(G) ∀ u,v ∈ V(H).
   */
  inline Ptr induced(const GraphBase& subgraph) const {
    return Ptr(new GraphBase(*this, graph_.induced(subgraph.graph_)));
  }
#endif
  /**
   * \brief Merge two graphs in place, as a set union
   */
  GraphBase& operator+=(const GraphBase& other) {
    graph_ += other.graph_;
    return *this;
  }

  /**
   * \brief Merge two graphs, as a set union
   */
  friend SelfType operator+(SelfType lhs, const SelfType& rhs) {
    lhs += rhs;
    return lhs;
  }

  /**
   * \brief Merge two graphs, as a set union
   */
  Ptr setUnion(const Ptr& other) const {
    return Ptr(new GraphBase(*this, graph_ + other->graph_));
  }
#if 0
  /**
   * \brief Use dijkstra's algorithm to traverse up to a depth (weighted edges)
   */
  Ptr dijkstraTraverseToDepth(
      const VertexIdType& rootId, double maxDepth,
      const Eval::Weight::Ptr& weights = Eval::Weight::Const::MakeShared(1, 1),
      const Eval::Mask::Ptr& mask = Eval::Mask::Const::MakeShared(true,
                                                                  true)) const;

  /**
   * \brief Use dijkstra's algorithm to search for an id (weighted edges)
   */
  Ptr dijkstraSearch(
      const VertexIdType& rootId, VertexIdType searchId,
      const Eval::Weight::Ptr& weights = Eval::Weight::Const::MakeShared(1, 1),
      const Eval::Mask::Ptr& mask = Eval::Mask::Const::MakeShared(true,
                                                                  true)) const;

  /**
   * \brief Use dijkstra's algorithm to search for multiple ids (weighted
   * edges)
   */
  Ptr dijkstraMultiSearch(
      const VertexIdType& rootId,
      const typename VertexIdType::Vector& searchIds,
      const Eval::Weight::Ptr& weights = Eval::Weight::Const::MakeShared(1, 1),
      const Eval::Mask::Ptr& mask = Eval::Mask::Const::MakeShared(true,
                                                                  true)) const;
  /**
   * \brief Get a decomposition of the graph containing only linear, acyclic
   * components \returns A list of junction/dead end vertices
   */
  typename VertexIdType::UnorderedSet pathDecomposition(
      ComponentList* paths, ComponentList* cycles) const;

  /**
   * \brief Use breadth first traversal up to a depth
   */
  Ptr breadthFirstTraversal(const VertexIdType& rootId, double maxDepth) const;

  /**
   * \brief Use breadth first search for an id
   */
  Ptr breadthFirstSearch(const VertexIdType& rootId,
                         VertexIdType searchId) const;

  /**
   * \brief Use breadth first search for multiple ids
   */
  Ptr breadthFirstMultiSearch(
      const VertexIdType& rootId,
      const typename VertexIdType::Vector& searchIds) const;
  /**
   * \brief Get minimal spanning tree
   */
  Ptr getMinimalSpanningTree(
      const Eval::Weight::Ptr& weights,
      const Eval::Mask::Ptr& mask = Eval::Mask::Const::MakeShared(true,
                                                                  true)) const;
#endif
 protected:
  /**
   * \brief Convert our ids to SimpleGraph ids
   */
  static inline SimpleGraph::VertexVec makeSimple(
      const typename VertexIdType::Vector& v) {
    return SimpleGraph::VertexVec(v.begin(), v.end());
  }

  /**
   * \brief Graph id
   */
  IdType id_;

  /**
   * \brief SimpleGraph object to hold the structure
   */
  SimpleGraph graph_;

  /**
   * \brief Map of runs in the graph
   */
  RunMapPtr runs_;

  /**
   * \brief Map from SimpleVertexId to vertex object
   */
  VertexMapPtr vertices_;

  /**
   * \brief Map from SimpleEdgeId to edge object
   */
  EdgeMapPtr edges_;
};

}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/index/graph_base.inl>

// TODO: Find a way to make explicit instantiation work in debug mode
#if !defined(BASIC_GRAPH_NO_EXTERN) && defined(NDEBUG)
namespace asrl {
namespace pose_graph {

extern template class RunBase<VertexBase, EdgeBase>;
extern template class GraphBase<VertexBase, EdgeBase,
                                RunBase<VertexBase, EdgeBase>>;

EVAL_TYPED_DECLARE_EXTERN(double, BasicGraphBase)
EVAL_TYPED_DECLARE_EXTERN(bool, BasicGraphBase)

}  // namespace pose_graph
}  // namespace asrl
#endif
