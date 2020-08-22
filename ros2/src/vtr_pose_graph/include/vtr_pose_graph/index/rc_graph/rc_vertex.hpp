#pragma once

#include <vtr_pose_graph/index/vertex_base.hpp>
#include <vtr_pose_graph/interface/rc_point_interface.hpp>
#include <vtr_pose_graph/interface/rc_stream_interface.hpp>
#if 0
#include <asrl/messages/Persistent.pb.h>
#include <asrl/messages/Utility.pb.h>
#include <asrl/messages/Vertex.pb.h>
#endif

namespace vtr {
namespace pose_graph {

#if 0
class RCGraph;
template <typename V, typename E, typename R>
class Graph;
#endif

class RCVertex : public VertexBase,
                 public RCPointInterface,
                 public RCStreamInterface {
 public:
  // Helper typedef to find the base class corresponding to edge data
  using Base = VertexBase;

#if 0
  // Message typedefs, used for retreiving a message type for an arbitrary graph
  // object
  using Msg = asrl::graph_msgs::Vertex;
  using HeaderMsg = asrl::graph_msgs::VertexHeader;

  // We must explicitly pull one copy of these typedefs from the parent
  // namespaces to prevent ambiguity
  typedef RCPointInterface::LockableFieldMap LockableFieldMap;
  typedef RCPointInterface::LockableFieldMapPtr LockableFieldMapPtr;
  typedef RCPointInterface::LockableStreamMapPtr LockableStreamMapPtr;
#endif
  /**
   * \brief Typedefs for shared pointers to vertices
   */
  PTR_TYPEDEFS(RCVertex)

  /**
   * \brief Interface to downcast base class pointers
   * \details This allows us to do DerivedPtrType = Type::Cast(BasePtrType)
   */
  PTR_DOWNCAST_OPS(RCVertex, VertexBase)

  /**
   * \brief Typedefs for containers of vertices
   */
  CONTAINER_TYPEDEFS(RCVertex)
  /**
   * \brief Pseudo-constructor for making shared pointers to vertices
   */
  static Ptr MakeShared();
  static Ptr MakeShared(const IdType& id);
#if 0
  static Ptr MakeShared(const asrl::graph_msgs::Vertex& msg,
                        const BaseIdType& runId,
                        const LockableFieldMapPtr& streamNames,
                        const LockableStreamMapPtr& streamMap);
#endif

  /**
   * \brief Default constructor
   */
  RCVertex();
  RCVertex(const IdType& id);
#if 0
  RCVertex(const Msg& msg, const BaseIdType& runId,
           const LockableFieldMapPtr& streamNames,
           const LockableStreamMapPtr& streamMap);
#endif
  /**
   * \brief Destructor
   */
  virtual ~RCVertex() = default;
#if 0

  /**
   * \brief Serialize to a protobuf message
   */
  void toProtobuf(Msg* msg);
#endif
  /**
   * \brief String name for file saving
   */
  const std::string name() const;
#if 0
  /**
   * \brief Helper for run filtering while loading
   */
  static inline bool MeetsFilter(const Msg&,
                                 const std::unordered_set<BaseIdType>&) {
    return true;
  }

  // Get the persistent id that can survive graph refactors
  const graph_msgs::PersistentId persistentId() const { return persistent_id_; }

  // \brief Sets the persistent id that can survive graph refactors
  void setPersistentId(const uint64_t& stamp, const uint32_t& robot);

 protected:
  // The persistent vertex id that can survive graph refactors
  graph_msgs::PersistentId persistent_id_;

  friend class RCGraph;
  template <typename V, typename E, typename R>
  friend class Graph;
#endif
};

}  // namespace pose_graph
}  // namespace vtr
