#pragma once

#include <algorithm>
#include <cstdint>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <vtr_pose_graph/evaluator/mask_evaluator.hpp>
#include <vtr_pose_graph/evaluator/weight_evaluator.hpp>
#include <vtr_pose_graph/simple_graph/linear_component.hpp>

namespace vtr {
namespace pose_graph {
namespace simple {
using SimpleVertex = uint64_t;
using SimpleEdge = std::pair<SimpleVertex, SimpleVertex>;

constexpr SimpleVertex InvalidSimpleVertex = static_cast<SimpleVertex>(-1);

class SimpleGraphIterator;

/** \brief Header structure */
class SimpleGraph {
 public:
  using VertexVec = std::vector<SimpleVertex>;
  using VertexSet = std::unordered_set<SimpleVertex>;
  using VertexList = std::list<SimpleVertex>;
  using EdgeVec = std::vector<SimpleEdge>;

  /** \brief Simple node class that implements the adjacent list paradigm */
  class SimpleNode {
   public:
    SimpleNode(SimpleVertex id = -1) : id_(id){};

    SimpleNode(const SimpleNode &) = default;
    SimpleNode(SimpleNode &&) = default;

    SimpleNode &operator=(const SimpleNode &) = default;
    SimpleNode &operator=(SimpleNode &&) = default;

    SimpleVertex getId() const {
      return id_;
    }

    void addAdjacent(SimpleVertex id) {
      adjacent_.push_back(id);
    }

    const std::list<SimpleVertex> &getAdjacent() const {
      return adjacent_;
    }

   private:
    SimpleVertex id_;
    std::list<SimpleVertex> adjacent_;
  };

  using NodeMap = std::unordered_map<SimpleVertex, SimpleNode>;
  using BacktraceMap = std::unordered_map<SimpleVertex, SimpleVertex>;

  using VertexIter = NodeMap::const_iterator;
  using EdgeIter = std::list<SimpleEdge>::const_iterator;
  using OrderedIter = SimpleGraphIterator;

  using SimpleComponent = LinearComponent<SimpleVertex>;
  using ComponentList = std::list<SimpleComponent>;

  /** \brief Default constructor */
  SimpleGraph(){};

  /** \brief Construct from list of edges */
  SimpleGraph(const std::list<SimpleEdge> &edges);

  /** \brief Construct a path/cycle from a list of vertices */
  SimpleGraph(const std::list<SimpleVertex> &vertices, bool cyclic = false);

  SimpleGraph(const SimpleGraph &) = default;
  SimpleGraph(SimpleGraph &&) = default;

  SimpleGraph &operator=(const SimpleGraph &) = default;
  SimpleGraph &operator=(SimpleGraph &&) = default;

  /** \brief Add a vertex */
  void addVertex(const SimpleVertex &vertex);

  /**
   * \brief Add an edge (inserts leaf nodes, but can only insert both if the
   * graph is fresh)
   */
  void addEdge(const SimpleEdge &edge);
  void addEdge(SimpleVertex id1, SimpleVertex id2);

  /** \brief Get the number of nodes */
  unsigned int numberOfNodes() const {
    return nodeMap_.size();
  }

  /** \brief Get the number of edges */
  unsigned int numberOfEdges() const {
    return edges_.size();
  }

  /** \brief Get node */
  const SimpleNode &getNode(SimpleVertex id) const {
    return nodeMap_.at(id);
  };

  /** \brief Get a list of the node ids */
  VertexVec getNodeIds() const;

  /** \brief Get a list of the edges */
  EdgeVec getEdges() const;

  /** \brief Determine if the simplegraph contains a vertex or not */
  bool hasVertex(const SimpleVertex &v) const {
    return nodeMap_.find(v) != nodeMap_.end();
  }

  /** \brief Determine if the simplegraph contains an edge or not */
  bool hasEdge(const SimpleEdge &e) const {
    if (!this->hasVertex(e.first))
      return false;
    const std::list<SimpleVertex> &adj = nodeMap_.at(e.first).getAdjacent();
    return std::find(adj.begin(), adj.end(), e.second) != adj.end();
  }

  /**
   * \brief Get an iterator for the graph.  Defaults to BFS over the entire
   * graph
   */
  OrderedIter begin(
      SimpleVertex root = SimpleVertex(-1), double maxDepth = 0,
      const eval::Mask::Ptr &mask = eval::Mask::Const::MakeShared(true, true),
      const eval::Weight::Ptr &weight =
          eval::Weight::Const::MakeShared(1.f, 1.f)) const;

  /** \brief Get a Breadth-First iterator for the graph */
  OrderedIter beginBfs(
      SimpleVertex root = SimpleVertex(-1), double maxDepth = 0,
      const eval::Mask::Ptr &mask = eval::Mask::Const::MakeShared(true, true),
      const eval::Weight::Ptr &weight =
          eval::Weight::Const::MakeShared(1.f, 1.f)) const;

  /** \brief Get a Depth-First iterator for the graph */
  OrderedIter beginDfs(
      SimpleVertex root = SimpleVertex(-1), double maxDepth = 0,
      const eval::Mask::Ptr &mask = eval::Mask::Const::MakeShared(true, true),
      const eval::Weight::Ptr &weight =
          eval::Weight::Const::MakeShared(1.f, 1.f)) const;

  /** \brief Get a Dijkstra iterator for the graph */
  OrderedIter beginDijkstra(
      SimpleVertex root = SimpleVertex(-1), double maxDepth = 0,
      const eval::Mask::Ptr &mask = eval::Mask::Const::MakeShared(true, true),
      const eval::Weight::Ptr &weight =
          eval::Weight::Const::MakeShared(1.f, 1.f)) const;
  
  /** \brief Get the end iterator for this graph */
  OrderedIter end() const;

  /** \brief Get an iterator to the beginning of the vertex map */
  inline VertexIter beginVertex() const {
    return nodeMap_.begin();
  }

  /** \brief Get an iterator to the end of the vertex map */
  inline VertexIter endVertex() const {
    return nodeMap_.end();
  }

  /** \brief Get an iterator to the beginning of the edge map */
  inline EdgeIter beginEdge() const {
    return edges_.begin();
  }

  /** \brief Get an iterator to the end of the edge map */
  inline EdgeIter endEdge() const {
    return edges_.end();
  }

  /**
   * \brief Get a decomposition of the graph containing only linear, acyclic
   * components
   * \returns A list of junction/dead end vertices
   */
  std::unordered_set<SimpleVertex> pathDecomposition(
      ComponentList *paths, ComponentList *cycles) const;

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  SimpleGraph getSubgraph(const VertexVec &nodes,
                          const eval::Mask::Ptr &mask =
                              eval::Mask::Const::MakeShared(true, true)) const;

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  SimpleGraph getSubgraph(SimpleVertex rootId,
                          const eval::Mask::Ptr &mask) const;

  /**
   * \brief Get subgraph including all the specified nodes (and all
   * interconnecting edges)
   */
  SimpleGraph getSubgraph(SimpleVertex rootId, double maxDepth,
                          const eval::Mask::Ptr &mask) const;

  /** \brief Get the induced subgraph of another subgraph */
  SimpleGraph induced(const SimpleGraph &subgraph) const {
    return getSubgraph(subgraph.getNodeIds());
  }

  /** \brief Merge two graphs in place, as a set union */
  SimpleGraph &operator+=(const SimpleGraph &other);

  /** \brief Merge two graphs, as a set union */
  friend SimpleGraph operator+(SimpleGraph lhs, const SimpleGraph &rhs) {
    lhs += rhs;
    return lhs;
  }

  /**
   * \brief Use dijkstra's algorithm to traverse up to a depth (weighted edges)
   */
  SimpleGraph dijkstraTraverseToDepth(
      SimpleVertex rootId, double maxDepth,
      const eval::Weight::Ptr &weights = eval::Weight::Const::MakeShared(1, 1),
      const eval::Mask::Ptr &mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const;

  /** \brief Use dijkstra's algorithm to search for an id (weighted edges) */
  SimpleGraph dijkstraSearch(
      SimpleVertex rootId, SimpleVertex searchId,
      const eval::Weight::Ptr &weights = eval::Weight::Const::MakeShared(1, 1),
      const eval::Mask::Ptr &mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const;

  /**
   * \brief Use dijkstra's algorithm to search for multiple ids (weighted
   * edges)
   */
  SimpleGraph dijkstraMultiSearch(
      SimpleVertex rootId, const VertexVec &searchIds,
      const eval::Weight::Ptr &weights = eval::Weight::Const::MakeShared(1, 1),
      const eval::Mask::Ptr &mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const;

  /** \brief Use breadth first traversal up to a depth */
  SimpleGraph breadthFirstTraversal(SimpleVertex rootId, double maxDepth) const;

  /** \brief Use breadth first traversal up to a depth */
  SimpleGraph breadthFirstTraversal(SimpleVertex rootId, double maxDepth,
                                    const eval::Mask::Ptr &mask) const;

  /** \brief Use breadth first search for an id */
  SimpleGraph breadthFirstSearch(SimpleVertex rootId,
                                 SimpleVertex searchId) const;

  /** \brief Use breadth first search for multiple ids */
  SimpleGraph breadthFirstMultiSearch(SimpleVertex rootId,
                                      const VertexVec &searchIds) const;

  /** \brief Get minimal spanning tree */
  SimpleGraph getMinimalSpanningTree(
      const eval::Weight::Ptr &weights,
      const eval::Mask::Ptr &mask = eval::Mask::Const::MakeShared(true,
                                                                  true)) const;

  /** \brief Print the structure of the graph */
  void print() const;

  /** \brief Get ordered edge */
  static SimpleEdge getEdge(SimpleVertex id1, SimpleVertex id2);

 private:
  /**
   * \brief Backtrace edges to root by following parents; all edges taken are
   * appended to the list of edges.
   */
  static void backtraceEdgesToRoot(const BacktraceMap &nodeParents,
                                   SimpleVertex node,
                                   std::list<SimpleEdge> *edges);

  /** \brief Node database */
  NodeMap nodeMap_;

  /** \brief List of edges */
  std::list<SimpleEdge> edges_;
};

}  // namespace simple
}  // namespace pose_graph
}  // namespace vtr
