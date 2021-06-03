#include <vtr_logging/logging.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_edge.hpp>

namespace vtr {
namespace pose_graph {

RCEdge::RCEdge(const vtr_messages::msg::GraphEdge& msg, BaseIdType runId,
               const LockableFieldMapPtr&, const LockableDataStreamMapPtr&)
    : EdgeBase(IdType(COMBINE(msg.to_run_id == -1 ? runId : msg.to_run_id,
                              msg.to_id),
                      COMBINE(runId, msg.from_id),
                      msg.to_run_id == -1 ? IdType::Type::Temporal
                                          : IdType::Type::Spatial),
               VertexId(runId, msg.from_id),
               VertexId(msg.to_run_id == -1 ? runId : msg.to_run_id, msg.to_id),
               msg.mode.mode == vtr_messages::msg::GraphEdgeMode::MANUAL) {
  const auto& transform = msg.t_to_from;
  if (!transform.entries.size()) return;
  if (transform.entries.size() != transform_vdim) {
    LOG(ERROR) << "Expected serialized transform vector to be of size "
               << transform_vdim << " actual: " << transform.entries.size();
    return;
  }

  if (!msg.t_to_from_cov.entries.size()) {
    setTransform(TransformType(TransformVecType(transform.entries.data())));
    return;
  }

  const auto& transform_cov = msg.t_to_from_cov;
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

RCEdge::Msg RCEdge::toRosMsg() {
  Msg msg;

  //  msg->set_id(id_.minorId());
  msg.mode.mode = manual_ ? vtr_messages::msg::GraphEdgeMode::MANUAL
                          : vtr_messages::msg::GraphEdgeMode::AUTONOMOUS;
  msg.from_id = from_.minorId();
  msg.to_id = to_.minorId();

  if (id_.type() == IdType::Type::Spatial) msg.to_run_id = to_.majorId();

  // set the transform
  TransformVecType vec(T_to_from_.vec());
  // TODO: make this an eigen map somehow...
  for (int row = 0; row < transform_vdim; ++row) {
    msg.t_to_from.entries.push_back(vec(row));
  }

  // save the covariance
  if (T_to_from_.covarianceSet() == true) {
    for (int row = 0; row < 6; row++)
      for (int col = 0; col < 6; col++)
        msg.t_to_from_cov.entries.push_back(T_to_from_.cov()(row, col));
  }

  // Assume the user intends to save the message...
  modified_ = false;

  return msg;
}

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
