#include <vtr_logging/logging.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_edge.hpp>

namespace vtr {
namespace pose_graph {

RCEdge::Ptr RCEdge::MakeShared() { return Ptr(new RCEdge()); }

RCEdge::Ptr RCEdge::MakeShared(const IdType& id) { return Ptr(new RCEdge(id)); }
RCEdge::Ptr RCEdge::MakeShared(const IdType& id, const VertexId& fromId,
                               const VertexId& toId, bool manual) {
  return Ptr(new RCEdge(id, fromId, toId, manual));
}
#if 0
RCEdge::Ptr RCEdge::MakeShared(const IdType& id, const VertexId& fromId,
                               const VertexId& toId,
                               const TransformType& T_to_from, bool manual) {
  return Ptr(new RCEdge(id, fromId, toId, T_to_from, manual));
}

RCEdge::Ptr RCEdge::MakeShared(
    const asrl::graph_msgs::Edge& msg, BaseIdType runId,
    const LockableFieldMapPtr& streamNames,
    const RCStreamInterface::LockableStreamMapPtr& streamMap) {
  return Ptr(new RCEdge(msg, runId, streamNames, streamMap));
}
#endif
RCEdge::RCEdge(const IdType& id) : EdgeBase(id), RCPointInterface() {}

RCEdge::RCEdge(const IdType id, const VertexId& fromId, const VertexId& toId,
               bool manual)
    : EdgeBase(id, fromId, toId, manual), RCPointInterface() {}
#if 0
RCEdge::RCEdge(const IdType& id, const VertexId& fromId, const VertexId& toId,
               const TransformType& T_to_from, bool manual)
    : EdgeBase(id, fromId, toId, T_to_from, manual), RCPointInterface() {}

RCEdge::RCEdge(const asrl::graph_msgs::Edge& msg, BaseIdType runId,
               const LockableFieldMapPtr& streamNames,
               const RCPointInterface::LockableStreamMapPtr& streamMap)
    : EdgeBase(
          IdType(COMBINE(msg.has_torunid() ? msg.torunid() : runId, msg.toid()),
                 COMBINE(runId, msg.fromid()),
                 (msg.has_torunid() ? IdType::Type::Spatial
                                    : IdType::Type::Temporal)),
          VertexId(runId, msg.fromid()),
          VertexId(msg.has_torunid() ? msg.torunid() : runId, msg.toid()),
          msg.mode() == asrl::graph_msgs::Mode::MANUAL),
      RCPointInterface(streamNames, streamMap, msg.pointidx()) {
  if (msg.has_t_to_from()) {
    const auto& transform = msg.t_to_from();
    if (transform.entries().size() != transform_vdim) {
      LOG(ERROR) << "Expected serialized transform vector to be of size "
                 << transform_vdim << " actual: " << transform.entries().size();
      return;
    }
    if (msg.has_t_to_from_cov()) {
      Eigen::Matrix<double, transform_vdim, transform_vdim> cov;
      const auto& proto_cov = msg.t_to_from_cov();
      if (proto_cov.entries().size() != cov.size()) {
        LOG(ERROR) << "Expected serialized covariance to be of size "
                   << cov.size();
        return;
      }
      for (int row = 0; row < transform_vdim; ++row) {
        for (int col = 0; col < transform_vdim; ++col) {
          cov(row, col) = proto_cov.entries().Get(row * transform_vdim + col);
        }
      }
      setTransform(
          TransformType(TransformVecType(transform.entries().data()), cov));
    } else {
      setTransform(TransformType(TransformVecType(transform.entries().data())));
    }
  }
}

void RCEdge::toProtobuf(asrl::graph_msgs::Edge* msg) {
  msg->Clear();

  //  msg->set_id(id_.minorId());
  msg->set_mode(manual_ ? asrl::graph_msgs::Mode::MANUAL
                        : asrl::graph_msgs::Mode::AUTONOMOUS);
  msg->set_fromid(from_.minorId());
  msg->set_toid(to_.minorId());
  serializePoints(msg->mutable_pointidx());

  if (id_.type() == IdType::Type::Spatial) {
    msg->set_torunid(to_.majorId());
  }

  // set the transform
  auto proto_transform = msg->mutable_t_to_from();
  TransformVecType vec(T_to_from_.vec());

  // TODO: make this an eigen map somehow...
  for (int row = 0; row < transform_vdim; ++row) {
    proto_transform->add_entries(vec(row));
  }

  // save the covariance
  if (T_to_from_.covarianceSet() == true) {
    auto proto_cov = msg->mutable_t_to_from_cov();
    for (int row = 0; row < 6; row++) {
      for (int col = 0; col < 6; col++) {
        proto_cov->add_entries(T_to_from_.cov()(row, col));
      }
    }
  }

  // Assume the user intends to save the message...
  modified_ = false;
}
#endif

const std::string RCEdge::name() const {
  if (id_.type() == IdType::Type::Temporal)
    return "temporal";
  else if (id_.type() == IdType::Type::Spatial)
    return "spatial";
  else
    return "unknown";
}

}  // namespace pose_graph
}  // namespace vtr
