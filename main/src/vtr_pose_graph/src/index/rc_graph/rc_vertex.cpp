#include <vtr_pose_graph/index/rc_graph/rc_vertex.hpp>

namespace vtr {
namespace pose_graph {

RCVertex::RCVertex(const Msg &msg, const BaseIdType &runId,
                   const LockableFieldMapPtr &streamNames,
                   const LockableDataStreamMapPtr &streamMap)
    : VertexBase(IdType(runId, msg.id)),
      RCStreamInterface(msg.stream_time, streamNames, streamMap,
                        msg.stream_idx) {
  const auto &transform = msg.t_vertex_world;
  if (!transform.entries.size()) return;
  if (transform.entries.size() != transform_vdim) {
    LOG(ERROR) << "Expected serialized transform vector to be of size "
               << transform_vdim << " actual: " << transform.entries.size();
    return;
  }

  if (!msg.t_vertex_world_cov.entries.size()) {
    setTransform(TransformType(TransformVecType(transform.entries.data())));
    return;
  }

  const auto &transform_cov = msg.t_vertex_world_cov;
  Eigen::Matrix<double, transform_vdim, transform_vdim> cov;
  if (transform_cov.entries.size() != (unsigned)cov.size()) {
    LOG(ERROR) << "Expected serialized covariance to be of size " << cov.size();
    return;
  }
  for (int row = 0; row < transform_vdim; ++row)
    for (int col = 0; col < transform_vdim; ++col)
      cov(row, col) = transform_cov.entries[row * transform_vdim + col];
  setTransform(TransformType(TransformVecType(transform.entries.data()), cov));
}

RCVertex::Msg RCVertex::toRosMsg() {
  Msg msg;

  msg.id = id_.minorId();

  /// serializeStreams(msg->mutable_streamtime(), msg->mutable_streamidx());
  auto [stream_time, stream_idx] = serializeStreams();
  msg.stream_time = stream_time;
  msg.stream_idx = stream_idx;

  // set the transform
  if (T_vertex_world_ != nullptr) {
    TransformVecType vec(T_vertex_world_->vec());
    // \todo make this an eigen map somehow...
    for (int row = 0; row < transform_vdim; ++row)
      msg.t_vertex_world.entries.push_back(vec(row));

    // save the covariance
    if (T_vertex_world_->covarianceSet() == true) {
      for (int row = 0; row < 6; row++)
        for (int col = 0; col < 6; col++)
          msg.t_vertex_world_cov.entries.push_back(
              T_vertex_world_->cov()(row, col));
    }

    modified_ = false;
  }
  return msg;
}

}  // namespace pose_graph
}  // namespace vtr
