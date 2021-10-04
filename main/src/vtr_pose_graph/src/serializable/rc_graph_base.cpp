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
 * \file rc_graph_base.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_pose_graph/serializable/rc_graph_base.hpp>

namespace vtr {
namespace pose_graph {

auto RCGraphBase::toPersistent(const VertexIdType& vid) const
    -> PersistentIdType {
  return at(vid)->persistentId();
}

auto RCGraphBase::fromPersistent(const PersistentIdType& pid) const
    -> VertexIdType {
  try {
    return persistent_map_->locked().get().at(pid);
  } catch (...) {
    LOG(ERROR) << "Could not find persistent id: " << pid << ".\n"
               << el::base::debug::StackTrace();
    throw;
  }
  return VertexIdType::Invalid();  // Should not get here
}

#if false
RCGraphBase::Ptr RCGraphBase::getManualSubgraph() {
  using PrivEvalType = typename eval::Mask::Privileged<RCGraphBase>::Caching;
  typename PrivEvalType::Ptr manualMask(new PrivEvalType());
  manualMask->setGraph(this);
  return getSubgraph(manualMask);
}
#endif

}  // namespace pose_graph
}  // namespace vtr
