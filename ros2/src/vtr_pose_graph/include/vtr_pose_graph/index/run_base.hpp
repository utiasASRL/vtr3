#pragma once

#include <map>
#include <unordered_map>  // #include <boost/unordered_map.hpp>

#include <vtr_pose_graph/index/edge_base.hpp>
#include <vtr_pose_graph/index/vertex_base.hpp>

namespace vtr {
namespace pose_graph {

template <class V, class E>
class RunBase {
 public:
  PTR_TYPEDEFS(RunBase)
  // The Id type of this opject
  using IdType = BaseIdType;
  // Each subclass will change this typedef; it is used for managing casts
  using VertexType = V;
  using EdgeType = E;
  // Edge/Vertex shared pointers; each subclass will change this
  using VertexPtr = typename V::Ptr;
  using EdgePtr = typename E::Ptr;
  // Edge/Vertex Id types
  using VertexIdType = typename V::IdType;
  using EdgeIdType = typename E::IdType;
  using EdgeEnumType = typename E::IdType::Type;
  using SimpleVertexId = typename V::SimpleIdType;
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
  static Ptr MakeShared();

  /** \brief Convenience constructor to create a shared pointer */
  static Ptr MakeShared(const IdType& runId, const IdType& graphId);

  /** \brief Default constructor, for completeness */
  RunBase();
  RunBase(const RunBase&) = default;
  RunBase(RunBase&&) = default;

  /** \brief Construct a new run, with a given Id */
  RunBase(const IdType& runId, const IdType& graphId);

  /** \brief Destructor */
  virtual ~RunBase() = default;

  /** \brief Default copy and move operations */
  RunBase& operator=(const RunBase&) = default;
  RunBase& operator=(RunBase&&) = default;

  /** \brief Return a blank vertex with the next available Id */
  virtual const VertexPtr& addVertex(
      const VertexIdType& v = VertexIdType::Invalid());

  /** \brief Return an edge between two vertices, with the next available Id */
  const EdgePtr& addEdge(const VertexIdType& from, const VertexIdType& to,
                         const EdgeEnumType& type_ = EdgeEnumType::Temporal,
                         bool manual = false);

  /** \brief Return an edge between two vertices, with the next available Id */
  const EdgePtr& addEdge(const VertexIdType& from, const VertexIdType& to,
                         const TransformType& T_to_from,
                         const EdgeEnumType& type_ = EdgeEnumType::Temporal,
                         bool manual = false);

  /** \brief Add an externally constructed edge */
  void addEdge(const EdgePtr& edge);

  /** \brief Return all vertices */
  inline const VertexPtrMap& vertices() const {
    return vertices_;
  }

  /** \brief Return all edges */
  inline const EdgePtrMapArray& edges() const {
    return edges_;
  }

  /** \brief Return all edges of a given type */
  inline const EdgePtrMap& edges(const EdgeEnumType& etype) const {
    return edges_[size_t(etype)];
  }

  /** \brief Map interface for vertices */
  inline VertexPtr& operator[](const VertexIdType& v) {
    return vertices_[v];
  }

  /** \brief Map interface for edges */
  inline EdgePtr& operator[](const EdgeIdType& e) {
    return edges_[e.idx()][e];
  }

  /** \brief Map interface for edges */
  inline EdgePtr& operator[](const SimpleEdgeId& e) {
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
  inline const VertexPtr& at(const VertexIdType& v) const {
    return vertices_.at(v);
  }

  /** \brief Const map interface for edges */
  inline const EdgePtr& at(const EdgeIdType& e) const {
    return edges_[e.idx()].at(e);
  }

  /** \brief Map interface for edges */
  inline const EdgePtr& at(const SimpleEdgeId& e) const {
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
  inline IdType id() const {
    return id_;
  }

  /** \brief Get the graph ID */
  inline IdType graphId() const {
    return graphId_;
  }

  /** \brief Query whether the run contains manual edges */
  inline bool isManual() const {
    return manual_;
  }

  /** \brief Recompute the manual status of the run */
  void computeManual();

 protected:
  /** \brief The run id, used for indexing and for loading/saving */
  IdType id_;

  /** \brief The graph id, used for ensuring data consistency */
  IdType graphId_;

  /**
   * \brief Internal vertex map
   * \details This preserves the mapping of VertexId --> VertexPtr and the
   *          mapping of RunId --> VertexPtr.  Without this secondary map,
   *          these properties would need to be rebuilt using a search.
   */
  VertexPtrMap vertices_;

  /** \brief Tracks the last vertex id in the run */
  BaseIdType currentVertex_;

  /**
   * \brief Internal edge map
   * \details This preserves the mapping of EdgeId --> EdgePtr and the
   *          mapping of RunId --> EdgePtr.  Without this secondary map,
   *          these properties would need to be rebuilt using a search.  This
   *          map also contains spatial edges we did not explicitly request,
   *          so that we do not overwrite the index file without them.
   */
  EdgePtrMapArray edges_;

  /** \brief Tracks the last edge id in the run, for each edge type */
  CurrentEdgeArray currentEdge_;

  /** \brief Cache whether or not this run contains manual edges */
  bool manual_;
};

using BasicRun = RunBase<VertexBase, EdgeBase>;

}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/index/run_base.inl>
