#pragma once

#include <lgmath/se3/TransformationWithCovariance.hpp>
#include <vtr_pose_graph/id/graph_id.hpp>

namespace vtr {
namespace pose_graph {

class EdgeBase {
 public:
  // Typedef the id and SimpleGraph id so that they are only hardcoded in one
  // place
  using IdType = EdgeId;
  using EnumType = EdgeId::Type;
  using VertexIdType = VertexId;
  using SimpleIdType = std::pair<uint64_t, uint64_t>;

  // If you need a different type of edge, change this and recompile. Making it
  // a template was just too gross.
  using TransformType = lgmath::se3::TransformationWithCovariance;
  const static int transform_rows = 4;
  const static int transform_cols = 4;
  const static int transform_vdim = 6;
  using TransformMatType =
      Eigen::Matrix<double, transform_cols, transform_rows>;
  using TransformVecType = Eigen::Matrix<double, transform_vdim, 1>;

  PTR_TYPEDEFS(EdgeBase)
  CONTAINER_TYPEDEFS(EdgeBase)

  /** \brief Pseudo constructors to generate a shared pointer */
  static Ptr MakeShared();
  static Ptr MakeShared(const IdType& id);
  static Ptr MakeShared(const IdType& id, const VertexIdType& fromId,
                        const VertexIdType& toId, bool manual = false);
  static Ptr MakeShared(const IdType& id, const VertexIdType& fromId,
                        const VertexIdType& toId,
                        const TransformType& T_to_from, bool manual = false);

  /** \brief Default constructor */
  EdgeBase();
  explicit EdgeBase(const IdType& id);
  EdgeBase(const IdType id, const VertexIdType& fromId,
           const VertexIdType& toId, bool manual = false);
  EdgeBase(const IdType id, const VertexIdType& fromId,
           const VertexIdType& toId, const TransformType& T_to_from,
           bool manual = false);
  EdgeBase(const EdgeBase&) = default;
  EdgeBase(EdgeBase&&) = default;

  /** \brief Default constructor */
  virtual ~EdgeBase() = default;

  EdgeBase& operator=(const EdgeBase&) = default;
  EdgeBase& operator=(EdgeBase&&) = default;

  /** \brief Get the edge id */
  IdType id() const;

  /** \brief Get the edge id as a plain type */
  SimpleIdType simpleId() const;

  /** \brief Get the edge type */
  IdType::Type type() const;

  /** \brief Get the edge type as a size_t for indexing */
  size_t idx() const;

  /** \brief Get the ids of incident vertices */
  VertexIdType::Pair incident() const;

  /** \brief Get the id of the from vertex */
  VertexIdType from() const;

  /** \brief Get the id of the to vertex */
  VertexIdType to() const;

  /** \brief Get the edge transform */
  virtual TransformType T() const;

  /** \brief Set the edge transform */
  virtual void setTransform(const TransformType& transform);

  /** \brief Return true if the edge was manually driven */
  bool isManual() const;

  /** \brief Flag this edge as manually driven */
  void setManual(bool manual = true);

  /** \brief Return true if the edge was driven autonomously */
  bool isAutonomous() const;

  /** \brief Flag this edge as autonomously driven */
  void setAutonomous(bool autonomous = true);

  /** \brief Return true if the edge is a temporal edge */
  bool isTemporal() const;

  /** \brief Return true if the edge is a spatial edge */
  bool isSpatial() const;

  /** \brief Return true if the vertex is incident on this edge */
  bool isIncident(const VertexIdType& v) const;

  /**
   * \brief Return true if the edge has been modified since it was last
   * serialized
   */
  bool isModified() const;

  /** \brief Flag the vertex as needing to be saved */
  void setModified(bool modified = true);

  /** \brief String output */
  friend std::ostream& operator<<(std::ostream& out, const EdgeBase& e);

 protected:
  /**
   * \brief Set the id of the to vertex
   * \details This method is private as the Graph class manages connectivity
   */
  void setTo(const VertexIdType& toId);

  /**
   * \brief Set the id of the from vertex
   * \details This method is private as the Graph class manages connectivity
   */
  void setFrom(const VertexIdType& fromId);

  /** \brief The edge Id */
  IdType id_;

  /** \brief The originating vertex Id */
  VertexIdType from_;

  /** \brief The terminating vertex Id */
  VertexIdType to_;

  /** \brief The transform that moves points in "from" to points in "to" */
  TransformType T_to_from_;

  /** \brief Whether this edge was manually driven or not */
  bool manual_;

  /** \brief Whether or not the edge has been modified */
  bool modified_;
};
}  // namespace pose_graph
}  // namespace vtr
