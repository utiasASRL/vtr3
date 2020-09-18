#include <vtr_logging/logging.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_edge.hpp>

namespace vtr {
namespace pose_graph {

#if 0
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
#endif
/// void RCEdge::toProtobuf(asrl::graph_msgs::Edge* msg) {
///   msg->Clear();
///
///   //  msg->set_id(id_.minorId());
///   msg->set_mode(manual_ ? asrl::graph_msgs::Mode::MANUAL
///                         : asrl::graph_msgs::Mode::AUTONOMOUS);
///   msg->set_fromid(from_.minorId());
///   msg->set_toid(to_.minorId());
///   serializePoints(msg->mutable_pointidx());
///
///   if (id_.type() == IdType::Type::Spatial) {
///     msg->set_torunid(to_.majorId());
///   }
///
///   // set the transform
///   auto proto_transform = msg->mutable_t_to_from();
///   TransformVecType vec(T_to_from_.vec());
///
///   // TODO: make this an eigen map somehow...
///   for (int row = 0; row < transform_vdim; ++row) {
///     proto_transform->add_entries(vec(row));
///   }
///
///   // save the covariance
///   if (T_to_from_.covarianceSet() == true) {
///     auto proto_cov = msg->mutable_t_to_from_cov();
///     for (int row = 0; row < 6; row++) {
///       for (int col = 0; col < 6; col++) {
///         proto_cov->add_entries(T_to_from_.cov()(row, col));
///       }
///     }
///   }
///
///   // Assume the user intends to save the message...
///   modified_ = false;
/// }
RCEdge::Msg RCEdge::toRosMsg() {
  Msg msg;

  //  msg->set_id(id_.minorId());
  msg.mode.mode = manual_ ? vtr_messages::msg::GraphEdgeMode::MANUAL
                          : vtr_messages::msg::GraphEdgeMode::AUTONOMOUS;
  msg.from_id = from_.minorId();
  msg.to_id = to_.minorId();
#if 0
  serializePoints(msg->mutable_pointidx());
#endif
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
