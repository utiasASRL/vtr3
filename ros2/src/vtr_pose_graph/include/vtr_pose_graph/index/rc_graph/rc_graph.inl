#pragma once

#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

namespace vtr {
namespace pose_graph {

template <typename MessageType>
void RCGraph::registerVertexStream(const RCGraph::RunIdType& run_id,
                                   const std::string& stream_name,
                                   bool points_to_data,
                                   const RegisterMode& mode) {
  if (runs_ != nullptr && runs_->find(run_id) != runs_->end()) {
    auto& run = runs_->at(run_id);
    run->registerVertexStream<MessageType>(stream_name, points_to_data, mode);
  } else {
    LOG(WARNING) << "[RCGraph::registerVertexStream] Run " << run_id
                 << " was not in the run map.";
  }
}

}  // namespace pose_graph
}  // namespace vtr
