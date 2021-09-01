// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file rc_graph.inl
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
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
