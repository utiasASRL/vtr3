#include <vtr_pose_graph/index/rc_graph/rc_vertex.hpp>

namespace vtr {
namespace pose_graph {

RCVertex::Ptr RCVertex::MakeShared() { return Ptr(new RCVertex()); }

RCVertex::Ptr RCVertex::MakeShared(const IdType &id) {
  return Ptr(new RCVertex(id));
}

#if 0
RCVertex::Ptr RCVertex::MakeShared(const asrl::graph_msgs::Vertex &msg,
                                   const BaseIdType &runId,
                                   const LockableFieldMapPtr &streamNames,
                                   const LockableStreamMapPtr &streamMap) {
  return Ptr(new RCVertex(msg, runId, streamNames, streamMap));
}
#endif
RCVertex::RCVertex() : VertexBase(), RCPointInterface(), RCStreamInterface() {}

RCVertex::RCVertex(const IdType &id)
    : VertexBase(id), RCPointInterface(), RCStreamInterface() {}
#if 0
RCVertex::RCVertex(const asrl::graph_msgs::Vertex &msg, const BaseIdType &runId,
                   const LockableFieldMapPtr &streamNames,
                   const LockableStreamMapPtr &streamMap)
    : VertexBase(IdType(runId, msg.id())),
      RCPointInterface(streamNames, streamMap, msg.pointidx()),
      RCStreamInterface(msg.streamtime(), streamNames, streamMap,
                        msg.streamidx()) {
  if (msg.has_t_vertex_world()) {
    const auto &transform = msg.t_vertex_world();
    if (transform.entries().size() != transform_vdim) {
      LOG(ERROR) << "Expected serialized transform vector to be of size "
                 << transform_vdim << " actual: " << transform.entries().size();
      return;
    }
    if (msg.has_t_vertex_world_cov()) {
      Eigen::Matrix<double, transform_vdim, transform_vdim> cov;
      const auto &proto_cov = msg.t_vertex_world_cov();
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

void RCVertex::toProtobuf(Msg *msg) {
  msg->Clear();

  msg->set_id(id_.minorId());
  serializePoints(msg->mutable_pointidx());
  serializeStreams(msg->mutable_streamtime(), msg->mutable_streamidx());

  // set the transform
  if (T_vertex_world_ != nullptr) {
    auto proto_transform = msg->mutable_t_vertex_world();
    TransformVecType vec(T_vertex_world_->vec());

    // TODO: make this an eigen map somehow...
    for (int row = 0; row < transform_vdim; ++row) {
      proto_transform->add_entries(vec(row));
    }

    // save the covariance
    if (T_vertex_world_->covarianceSet() == true) {
      auto proto_cov = msg->mutable_t_vertex_world_cov();
      for (int row = 0; row < 6; row++) {
        for (int col = 0; col < 6; col++) {
          proto_cov->add_entries(T_vertex_world_->cov()(row, col));
        }
      }
    }
  }

  modified_ = false;
}
#endif

void RCVertex::setPersistentId(const uint64_t &stamp, const uint32_t &robot) {
  persistent_id_.stamp = stamp;
  persistent_id_.robot = robot;
}

const std::string RCVertex::name() const { return "vertex"; }
}  // namespace pose_graph
}  // namespace vtr
