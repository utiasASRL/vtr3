#pragma once

#include <vtr_messages/msg/graph_edge.hpp>
#include <vtr_messages/msg/graph_edge_header.hpp>
#include <vtr_pose_graph/index/edge_base.hpp>
#include <vtr_pose_graph/interface/rc_point_interface.hpp>

#if 0
#include <stdexcept>

#include <asrl/pose_graph/interface/RCStreamInterface.hpp>

#include <asrl/common/utils/CommonMacros.hpp>
#include <asrl/common/utils/ContainerTools.hpp>

#include <asrl/messages/Edge.pb.h>
#include <asrl/messages/Utility.pb.h>
#endif
namespace vtr {
namespace pose_graph {

class RCEdge : public EdgeBase, public RCPointInterface {
 public:
  // Helper typedef to find the base class corresponding to edge data
  using Base = EdgeBase;

  // Message typedefs, used for retreiving a message type for an arbitrary graph
  // object
  using Msg = vtr_messages::msg::GraphEdge;
  using HeaderMsg = vtr_messages::msg::GraphEdgeHeader;

  /** \brief Typedefs for shared pointers to edges */
  PTR_TYPEDEFS(RCEdge)

  /**
   * \brief Interface to downcast base class pointers
   * \details This allows us to do DerivedPtrType = Type::Cast(BasePtrType)
   */
  PTR_DOWNCAST_OPS(RCEdge, EdgeBase)

  /** \brief Typedefs for containers of edges */
  CONTAINER_TYPEDEFS(RCEdge)

  /** \brief Pseudo-constructors for making shared pointers to edges */
  static Ptr MakeShared() { return Ptr(new RCEdge()); }
  static Ptr MakeShared(const IdType& id) { return Ptr(new RCEdge(id)); }
  static Ptr MakeShared(const IdType& id, const VertexId& fromId,
                        const VertexId& toId, bool manual = false) {
    return Ptr(new RCEdge(id, fromId, toId, manual));
  }
#if 0
  static Ptr MakeShared(const IdType& id, const VertexId& fromId,
                        const VertexId& toId, const TransformType& T_to_from,
                        bool manual = false) {
    return Ptr(new RCEdge(id, fromId, toId, T_to_from, manual));
  }
  static Ptr MakeShared(
      const asrl::graph_msgs::Edge& msg, BaseIdType runId,
      const LockableFieldMapPtr& streamNames,
      const RCStreamInterface::LockableStreamMapPtr& streamMap) {
    return Ptr(new RCEdge(msg, runId, streamNames, streamMap));
  }
#endif
  /** \brief Default constructor */
  RCEdge() = default;
  explicit RCEdge(const IdType& id) : EdgeBase(id), RCPointInterface() {}
  RCEdge(const IdType id, const VertexId& fromId, const VertexId& toId,
         bool manual = false)
      : EdgeBase(id, fromId, toId, manual), RCPointInterface() {}
#if 0
  RCEdge(const IdType& id, const VertexId& fromId, const VertexId& toId,
         const TransformType& T_to_from, bool manual = false)
      : EdgeBase(id, fromId, toId, T_to_from, manual), RCPointInterface() {}
  RCEdge(const asrl::graph_msgs::Edge& msg, BaseIdType runId,
         const LockableFieldMapPtr& streamNames,
         const RCStreamInterface::LockableStreamMapPtr& streamMap);
#endif
  /** \brief Default constructor */
  virtual ~RCEdge() = default;

  /** \brief String name for file saving */
  const std::string name() const;

  /** \brief Serialize to a ros message, as a temporal edge */
  // void toProtobuf(asrl::graph_msgs::Edge* msg);
  Msg toRosMsg();

#if 0
  /** \brief Helper for run filtering while loading */
  static inline bool MeetsFilter(const Msg& m,
                                 const std::unordered_set<BaseIdType>& r) {
    return !m.has_torunid() || asrl::common::utils::contains(r, m.torunid());
  }
#endif
};
}  // namespace pose_graph
}  // namespace vtr
